/**
 ******************************************************************************
 * @file    usart1_control.c
 * @brief   上位机串口控制实现 (USART1 + DMA+IDLE)
 *
 * 协议摘要:
 *   帧头(0xDF)
 *   目标地址 (U8)
 *   本机地址 (U8)
 *   类型 A (U8)
 *   类型 B (U8)
 *   数据长度 (U8) = 0x06
 *   数据区 6 字节:
 *     [0..1] S16 小端: X 方向速度 (右为正, mm/s)
 *     [2..3] S16 小端: Y 方向速度 (前为正, mm/s)
 *     [4..5] S16 小端: 旋转速度 (顺时针为正, mm/s)
 *   帧尾(0xFD)
 *   校验 (U8): 从帧头到帧尾(含)累加和取低 8 位
 *
 * 本模块职责:
 *   - 通过 HAL_UARTEx_ReceiveToIdle_DMA 启动 USART1 的 DMA+IDLE 接收
 *   - 在 HAL_UARTEx_RxEventCallback 中按协议解析数据
 *   - 将解析出的 vx/vy/w (mm/s) 填入 Uart1_ControlCmd_t 供上层查询
 *   - 收到合法指令时回传 0x43 作为应答
 *
 * 测试指令 (14 字节, 波特率 115200, 十六进制发送):
 *
 *   前进 300 mm/s:   DF 01 97 02 67 06 00 00 2C 01 00 00 FD 10
 *   后退 300 mm/s:   DF 01 97 02 67 06 00 00 D4 FE 00 00 FD B5
 *   右直行 300 mm/s:  DF 01 97 02 67 06 2C 01 00 00 00 00 FD 10
 *   左直行 300 mm/s:  DF 01 97 02 67 06 D4 FE 00 00 00 00 FD B5
 *   顺时针旋转 300:   DF 01 97 02 67 06 00 00 00 00 2C 01 FD 10
 *   停止:            DF 01 97 02 67 06 00 00 00 00 00 00 FD E3
 *
 *   解析成功时单片机回传 0x43
 ******************************************************************************
 */

#include "usart1_control.h"
#include "usart.h"
#include <stdint.h>
#include <string.h>

/* USART1 DMA 接收缓冲区配置: 存放 HAL DMA+IDLE 收到的原始字节流 */
#define UART1_RX_BUF_SIZE 128

/* 协议常量 (当前仅实现: 匀速移动指令 A=0x02/B=0x67) */
#define UART1_FRAME_HEAD 0xDF
#define UART1_FRAME_TAIL 0xFD

#define UART1_ADDR_MCU 0x01
#define UART1_ADDR_PC 0x97

#define UART1_TYPE_A_MOVE 0x02
#define UART1_TYPE_B_VELOCITY 0x67     /* 持续速度控制 */
#define UART1_TYPE_B_DISPLACEMENT 0x64 /* 定点匀速位移 */

#define UART1_DATA_LEN_VELOCITY 0x06
#define UART1_DATA_LEN_DISPLACEMENT 14

#define UART1_FRAME_LEN_VELOCITY                                               \
  (1U /* head */ + 1U /* dst */ + 1U /* src */ + 2U /* typeA/typeB */ +        \
   1U /* len */ + UART1_DATA_LEN_VELOCITY /* payload */ + 1U /* tail */ +      \
   1U /* checksum */)

#define UART1_FRAME_LEN_DISPLACEMENT                                           \
  (1U /* head */ + 1U /* dst */ + 1U /* src */ + 2U /* typeA/typeB */ +        \
   1U /* len */ + UART1_DATA_LEN_DISPLACEMENT /* payload */ + 1U /* tail */ +  \
   1U /* checksum */)

/* 速度缩放: 协议中 S16 一位对应多少 mm/s (由上位机约定) */
#define UART1_SPEED_SCALE_MMPS_PER_LSB 1.0f

/* DMA 接收缓冲区 (只在本模块内部可见) */
static uint8_t s_uart1_rx_buf[UART1_RX_BUF_SIZE];

/* 最近一次解析成功的控制命令
 * 标记为 volatile 是为了防止编译器在中断与主循环之间优化掉读写
 */
static volatile Uart1_ControlCmd_t s_latest_cmd;

/* 将两个小端无符号字节组合成有符号 16 位整数 */
static int16_t BytesToS16LE(uint8_t low, uint8_t high) {
  return (int16_t)((uint16_t)low | ((uint16_t)high << 8));
}

/* 将两个小端无符号字节组合成无符号 16 位整数 */
static uint16_t BytesToU16LE(uint8_t low, uint8_t high) {
  return (uint16_t)((uint16_t)low | ((uint16_t)high << 8));
}

/* 将四个小端字节组合成有符号 32 位整数 */
static int32_t BytesToS32LE(const uint8_t *p) {
  return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                   ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

/* 尝试解析速度控制帧 (0x67) */
static uint8_t ParseVelocityFrame(const uint8_t *buf, uint16_t size) {
  if (size != (uint16_t)UART1_FRAME_LEN_VELOCITY)
    return 0U;
  if (buf[0] != UART1_FRAME_HEAD || buf[12] != UART1_FRAME_TAIL)
    return 0U;
  if (buf[1] != UART1_ADDR_MCU || buf[2] != UART1_ADDR_PC)
    return 0U;
  if (buf[3] != UART1_TYPE_A_MOVE || buf[4] != UART1_TYPE_B_VELOCITY)
    return 0U;
  if (buf[5] != UART1_DATA_LEN_VELOCITY)
    return 0U;

  uint8_t sum = 0U;
  for (uint8_t i = 0; i <= 12; i++)
    sum = (uint8_t)(sum + buf[i]);
  if (sum != buf[13])
    return 0U;

  Uart1_ControlCmd_t cmd = {0};
  cmd.type = UART1_CMD_VELOCITY;
  cmd.vx_mmps =
      (float)BytesToS16LE(buf[6], buf[7]) * UART1_SPEED_SCALE_MMPS_PER_LSB;
  cmd.vy_mmps =
      (float)BytesToS16LE(buf[8], buf[9]) * UART1_SPEED_SCALE_MMPS_PER_LSB;
  cmd.w_mmps =
      (float)BytesToS16LE(buf[10], buf[11]) * UART1_SPEED_SCALE_MMPS_PER_LSB;
  cmd.valid = 1U;

  s_latest_cmd = cmd;
  return 1U;
}

/* 尝试解析位移控制帧 (0x64) */
static uint8_t ParseDisplacementFrame(const uint8_t *buf, uint16_t size) {
  if (size != (uint16_t)UART1_FRAME_LEN_DISPLACEMENT)
    return 0U;
  if (buf[0] != UART1_FRAME_HEAD || buf[20] != UART1_FRAME_TAIL)
    return 0U;
  if (buf[1] != UART1_ADDR_MCU || buf[2] != UART1_ADDR_PC)
    return 0U;
  if (buf[3] != UART1_TYPE_A_MOVE || buf[4] != UART1_TYPE_B_DISPLACEMENT)
    return 0U;
  if (buf[5] != UART1_DATA_LEN_DISPLACEMENT)
    return 0U;

  uint8_t sum = 0U;
  for (uint8_t i = 0; i <= 20; i++)
    sum = (uint8_t)(sum + buf[i]);
  if (sum != buf[21])
    return 0U;

  Uart1_ControlCmd_t cmd = {0};
  cmd.type = UART1_CMD_DISPLACEMENT;
  /*
   * 协议修正: 接收的是 S32 整数 (米 * 100,000)
   * 目标单位是 mm: (int / 100,000) * 1,000 = int / 100
   * 例如: 输入 0.2m -> 协议发送 20,000 (20 4E 00 00) -> 转换后为 200.0 mm
   */
  cmd.target_x_mm = (float)BytesToS32LE(&buf[6]) / 100.0f;
  cmd.target_y_mm = (float)BytesToS32LE(&buf[10]) / 100.0f;
  cmd.target_z_mm = (float)BytesToS32LE(&buf[14]) / 100.0f;

  // 限速单位是 cm/s，放大 100 倍存入 U16，转换为 mm/s 需要: (raw / 100) * 10 =
  // raw / 10
  cmd.target_speed_mmps = (float)BytesToU16LE(buf[18], buf[19]) / 10.0f;
  cmd.valid = 1U;

  s_latest_cmd = cmd;
  return 1U;
}

void Uart1_Control_Init(void) {
  /* 初始化最近命令为无效状态 */
  s_latest_cmd.type = UART1_CMD_NONE;
  s_latest_cmd.vx_mmps = 0.0f;
  s_latest_cmd.vy_mmps = 0.0f;
  s_latest_cmd.w_mmps = 0.0f;
  s_latest_cmd.valid = 0U;

  /* 启动 USART1 的 DMA+IDLE 接收
   * - huart1: 来自 usart.c 的 UART1 句柄
   * - s_uart1_rx_buf: DMA 接收缓冲区
   * - UART1_RX_BUF_SIZE: 最大接收长度
   */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_uart1_rx_buf, UART1_RX_BUF_SIZE);
  /* 关闭半传输中断，避免产生过多中断回调 */
  if (huart1.hdmarx != NULL) {
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }
}

void Uart1_Control_GetLatestCmd(Uart1_ControlCmd_t *out) {
  if (out == NULL) {
    return;
  }
  /* 简单的“临界区”: 复制结构体时短暂关中断, 防止中断同时修改 s_latest_cmd */
  __disable_irq();
//  *out = (Uart1_ControlCmd_t)s_latest_cmd;
  __enable_irq();
}

void Uart1_Control_ClearCmd(void) {
  __disable_irq();
  s_latest_cmd.valid = 0U;
  s_latest_cmd.type = UART1_CMD_NONE;
  __enable_irq();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  /* 调试：翻转 LED 确认回调被调用 */
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  
  if (huart == NULL) {
    return;
  }
  if (huart->Instance != USART1) {
    return;
  }

  /* 尝试在本次数据块中寻找并解析控制帧
   * 允许前面有噪声或一次收到多帧：匹配到的帧都会更新 s_latest_cmd，最后一帧生效
   * 每次解析成功则回传 0x43 作为应答
   */
  /* 尝试在本次数据块中寻找并解析控制帧 */
  if (Size >= UART1_FRAME_LEN_VELOCITY) {
    for (uint16_t i = 0; i <= Size - 14; i++) {
      if (s_uart1_rx_buf[i] == UART1_FRAME_HEAD) {
        if (ParseVelocityFrame(&s_uart1_rx_buf[i], UART1_FRAME_LEN_VELOCITY)) {
          const uint8_t ack = 0x43U;
          HAL_UART_Transmit(&huart1, (uint8_t *)&ack, 1, 10);
        }
      }
    }
  }

  /* 寻找位移帧 (0x64) */
  if (Size >= UART1_FRAME_LEN_DISPLACEMENT) {
    for (uint16_t i = 0; i <= Size - 22; i++) {
      if (s_uart1_rx_buf[i] == UART1_FRAME_HEAD) {
        if (ParseDisplacementFrame(&s_uart1_rx_buf[i],
                                   UART1_FRAME_LEN_DISPLACEMENT)) {
          const uint8_t ack = 0x43U;
          HAL_UART_Transmit(&huart1, (uint8_t *)&ack, 1, 10);
        }
      }
    }
  }

  /* 重新启动 DMA+IDLE 接收 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_uart1_rx_buf, UART1_RX_BUF_SIZE);
  if (huart1.hdmarx != NULL) {
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }
}
