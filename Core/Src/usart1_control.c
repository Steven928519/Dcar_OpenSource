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

/* 持久化缓存: 防止无线透传时的断帧分包丢数据 */
static uint8_t s_frame_buf[UART1_RX_BUF_SIZE * 2];
static uint16_t s_frame_len = 0;

/* 协议常量 */
#define UART1_FRAME_HEAD 0xDF
#define UART1_FRAME_TAIL 0xFD

#define UART1_ADDR_MCU 0x01
#define UART1_ADDR_PC 0x97

#define UART1_TYPE_A_MOVE 0x02
#define UART1_TYPE_B_VELOCITY 0x67     /* 持续速度控制 */
#define UART1_TYPE_B_DISPLACEMENT 0x68 /* 相对位移控制 */

#define UART1_DATA_LEN_VELOCITY 0x06
#define UART1_DATA_LEN_DISPLACEMENT 0x06

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

/**
 * @brief 尝试解析速度控制帧并更新全局指令
 * @return 1=成功, 0=失败
 */
static uint8_t ParseVelocityFrame(const uint8_t *buf, uint16_t size) {
  if (size != (uint16_t)UART1_FRAME_LEN_VELOCITY) {
    return 0U;
  }
  if (buf[0] != UART1_FRAME_HEAD || buf[12] != UART1_FRAME_TAIL) {
    return 0U;
  }
  if (buf[1] != UART1_ADDR_MCU || buf[2] != UART1_ADDR_PC) {
    return 0U;
  }
  if (buf[3] != UART1_TYPE_A_MOVE || buf[4] != UART1_TYPE_B_VELOCITY) {
    return 0U;
  }
  if (buf[5] != UART1_DATA_LEN_VELOCITY) {
    return 0U;
  }

  uint8_t sum = 0U;
  for (uint8_t i = 0; i <= 12; i++) {
    sum = (uint8_t)(sum + buf[i]);
  }
  if (sum != buf[13]) {
    return 0U;
  }

  const int16_t vx_raw = BytesToS16LE(buf[6], buf[7]);
  const int16_t vy_raw = BytesToS16LE(buf[8], buf[9]);
  const int16_t w_raw = BytesToS16LE(buf[10], buf[11]);

  Uart1_ControlCmd_t cmd;
  cmd.vx_mmps = (float)vx_raw * UART1_SPEED_SCALE_MMPS_PER_LSB;
  cmd.vy_mmps = (float)vy_raw * UART1_SPEED_SCALE_MMPS_PER_LSB;
  cmd.w_mmps = (float)w_raw * UART1_SPEED_SCALE_MMPS_PER_LSB;
  cmd.valid = 1U;
  cmd.type = UART1_CMD_VELOCITY;

  s_latest_cmd = cmd;
  return 1U;
}

void Uart1_Control_Init(void) {
  s_latest_cmd.vx_mmps = 0.0f;
  s_latest_cmd.vy_mmps = 0.0f;
  s_latest_cmd.w_mmps = 0.0f;
  s_latest_cmd.valid = 0U;

  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_uart1_rx_buf, UART1_RX_BUF_SIZE);
  if (huart1.hdmarx != NULL) {
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }
}

void Uart1_Control_GetLatestCmd(Uart1_ControlCmd_t *out) {
  if (out == NULL)
    return;
  __disable_irq();
  *out = (Uart1_ControlCmd_t)s_latest_cmd;
  __enable_irq();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart == NULL || huart->Instance != USART1) {
    return;
  }

  /* 1. 将本次收到的数据追加到持久缓存 */
  for (uint16_t i = 0; i < Size; i++) {
    if (s_frame_len < sizeof(s_frame_buf)) {
      s_frame_buf[s_frame_len++] = s_uart1_rx_buf[i];
    } else {
      memmove(s_frame_buf, s_frame_buf + 1, sizeof(s_frame_buf) - 1);
      s_frame_buf[sizeof(s_frame_buf) - 1] = s_uart1_rx_buf[i];
    }
  }

  /* 2. 循环解析缓存中的数据 */
  while (s_frame_len >= UART1_FRAME_LEN_VELOCITY) {
    uint16_t head_idx = 0xFFFF;
    for (uint16_t i = 0; i <= s_frame_len - UART1_FRAME_LEN_VELOCITY; i++) {
      if (s_frame_buf[i] == UART1_FRAME_HEAD) {
        head_idx = i;
        break;
      }
    }

    if (head_idx != 0xFFFF) {
      if (ParseVelocityFrame(&s_frame_buf[head_idx], UART1_FRAME_LEN_VELOCITY)) {
        const uint8_t ack = 0x43U;
        (void)HAL_UART_Transmit(&huart1, (uint8_t *)&ack, 1, 10);
        uint16_t remove_len = head_idx + UART1_FRAME_LEN_VELOCITY;
        memmove(s_frame_buf, s_frame_buf + remove_len, s_frame_len - remove_len);
        s_frame_len -= remove_len;
      } else {
        memmove(s_frame_buf, s_frame_buf + head_idx + 1, s_frame_len - head_idx - 1);
        s_frame_len -= (head_idx + 1);
      }
    } else {
      uint16_t remove_len = s_frame_len - UART1_FRAME_LEN_VELOCITY + 1;
      memmove(s_frame_buf, s_frame_buf + remove_len, s_frame_len - remove_len);
      s_frame_len -= remove_len;
      break;
    }
  }

  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_uart1_rx_buf, UART1_RX_BUF_SIZE);
  if (huart1.hdmarx != NULL) {
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }
}
