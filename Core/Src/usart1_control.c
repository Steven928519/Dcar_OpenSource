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

/* USART1 DMA 接收缓冲区配置: 存放 HAL DMA+IDLE 收到的原始字节流 */
#define UART1_RX_BUF_SIZE 128

/* 协议常量 (当前仅实现: 匀速移动指令 A=0x02/B=0x67) */
#define UART1_FRAME_HEAD 0xDF
#define UART1_FRAME_TAIL 0xFD

#define UART1_ADDR_MCU 0x01
#define UART1_ADDR_PC 0x97

#define UART1_TYPE_A_MOVE 0x02
#define UART1_TYPE_B_MOVE 0x67

#define UART1_DATA_LEN_MOVE 0x06

#define UART1_FRAME_LEN_MOVE                                                   \
  (1U /* head */ + 1U /* dst */ + 1U /* src */ + 2U /* typeA/typeB */ +        \
   1U /* len */ + UART1_DATA_LEN_MOVE /* payload */ + 1U /* tail */ +          \
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

/* 尝试将 buf 指向的一帧“匀速移动命令”解析为 Uart1_ControlCmd_t
 * 返回值: 1=解析成功并更新 s_latest_cmd, 0=解析失败
 */
static uint8_t ParseMoveFrameAndUpdateCmd(const uint8_t *buf, uint16_t size) {
  if (size != (uint16_t)UART1_FRAME_LEN_MOVE) {
    return 0U;
  }

  if (buf[0] != UART1_FRAME_HEAD || buf[12] != UART1_FRAME_TAIL) {
    return 0U;
  }

  const uint8_t dst = buf[1];
  const uint8_t src = buf[2];
  if (dst != UART1_ADDR_MCU || src != UART1_ADDR_PC) {
    return 0U;
  }

  const uint8_t typeA = buf[3];
  const uint8_t typeB = buf[4];
  if (typeA != UART1_TYPE_A_MOVE || typeB != UART1_TYPE_B_MOVE) {
    return 0U;
  }

  const uint8_t data_len = buf[5];
  if (data_len != UART1_DATA_LEN_MOVE) {
    return 0U;
  }

  /* 按累加和规则校验: 从帧头到帧尾(含)逐字节求和, 结果应等于校验字节 */
  uint8_t sum = 0U;
  for (uint8_t i = 0; i <= 12; i++) { /* 从帧头到帧尾 (含) */
    sum = (uint8_t)(sum + buf[i]);
  }
  if (sum != buf[13]) {
    return 0U;
  }

  const int16_t vx_raw = BytesToS16LE(buf[6], buf[7]);
  const int16_t vy_raw = BytesToS16LE(buf[8], buf[9]);
  const int16_t w_raw = BytesToS16LE(buf[10], buf[11]);

  Uart1_ControlCmd_t cmd;
  /* 将协议中的 S16 原始值按比例换算为物理速度 (mm/s) */
  cmd.vx_mmps = (float)vx_raw * UART1_SPEED_SCALE_MMPS_PER_LSB;
  cmd.vy_mmps = (float)vy_raw * UART1_SPEED_SCALE_MMPS_PER_LSB;
  cmd.w_mmps = (float)w_raw * UART1_SPEED_SCALE_MMPS_PER_LSB;
  cmd.valid = 1U;

  s_latest_cmd = cmd;
  return 1U;
}

void Uart1_Control_Init(void) {
  /* 初始化最近命令为无效状态 */
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
  *out = (Uart1_ControlCmd_t)s_latest_cmd;
  __enable_irq();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
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
  if (Size >= UART1_FRAME_LEN_MOVE) {
    const uint16_t max_i = (uint16_t)(Size - UART1_FRAME_LEN_MOVE);
    for (uint16_t i = 0; i <= max_i; i++) {
      if (s_uart1_rx_buf[i] == UART1_FRAME_HEAD) {
        if (ParseMoveFrameAndUpdateCmd(&s_uart1_rx_buf[i],
                                       (uint16_t)UART1_FRAME_LEN_MOVE)) {
          const uint8_t ack = 0x43U;
          (void)HAL_UART_Transmit(&huart1, (uint8_t *)&ack, 1, 10);
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
