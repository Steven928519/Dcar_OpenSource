/**
  ******************************************************************************
  * @file    ps2_receiver.c
  * @brief   PS2 遥控器信号接收模块
  *          通过 PA3(USART2_RX) 接收 SBUS 信号，解析后直接存入变量 (0~2047)
  ******************************************************************************
  */
#include "ps2_receiver.h"
#include "usart.h"

/* SBUS 协议常量 */
#define SBUS_HEADER        0x0F
#define SBUS_FOOTER        0x00
#define SBUS_PACKET_LENGTH 25

/* SBUS 协议状态 */
typedef enum {
  SBUS_STATE_WAIT_HEADER = 0,
  SBUS_STATE_RECEIVING   = 1
} SBUS_State_TypeDef;

/* 全局共享 PS2 数据 */
static PS2_Data_TypeDef ps2_data;

/* SBUS 接收缓冲区 */
static uint8_t  sbus_rx_buffer[SBUS_PACKET_LENGTH];
static uint8_t  sbus_rx_index;
static SBUS_State_TypeDef sbus_state;

/* 解析 SBUS 数据包并提取 16 通道 */
static uint8_t SBUS_Parse_Packet(uint8_t *packet);

/* 将 SBUS 通道存入 ps2_data (范围 0~2047 原始值) */
static void SBUS_StoreChannels(uint16_t *channels);

/**
  * @brief  解析 SBUS 数据包
  */
static uint8_t SBUS_Parse_Packet(uint8_t *packet)
{
  uint16_t channels[16];

  if (packet[0] != SBUS_HEADER || packet[24] != SBUS_FOOTER)
    return 0;

  /* 解析 16 通道 (每通道 11 位) */
  channels[0]  = ((uint16_t)packet[1]) | ((uint16_t)packet[2] << 8);
  channels[0] &= 0x07FF;

  channels[1]  = ((uint16_t)packet[2] >> 3) | ((uint16_t)packet[3] << 5);
  channels[1] &= 0x07FF;

  channels[2]  = ((uint16_t)packet[3] >> 6) | ((uint16_t)packet[4] << 2) | ((uint16_t)(packet[5] & 0x01) << 10);
  channels[2] &= 0x07FF;

  channels[3]  = ((uint16_t)packet[5] >> 1) | ((uint16_t)packet[6] << 7);
  channels[3] &= 0x07FF;

  channels[4]  = ((uint16_t)packet[6] >> 4) | ((uint16_t)packet[7] << 4);
  channels[4] &= 0x07FF;

  channels[5]  = ((uint16_t)packet[7] >> 7) | ((uint16_t)packet[8] << 1) | ((uint16_t)packet[9] << 9);
  channels[5] &= 0x07FF;

  channels[6]  = ((uint16_t)packet[9] >> 2) | ((uint16_t)packet[10] << 6);
  channels[6] &= 0x07FF;

  channels[7]  = ((uint16_t)packet[10] >> 5) | ((uint16_t)packet[11] << 3);
  channels[7] &= 0x07FF;

  channels[8]  = ((uint16_t)packet[12]) | ((uint16_t)packet[13] << 8);
  channels[8] &= 0x07FF;

  channels[9]  = ((uint16_t)packet[13] >> 3) | ((uint16_t)packet[14] << 5);
  channels[9] &= 0x07FF;

  channels[10] = ((uint16_t)packet[14] >> 6) | ((uint16_t)packet[15] << 2) | ((uint16_t)packet[16] << 10);
  channels[10] &= 0x07FF;

  channels[11] = ((uint16_t)packet[16] >> 1) | ((uint16_t)packet[17] << 7);
  channels[11] &= 0x07FF;

  channels[12] = ((uint16_t)packet[17] >> 4) | ((uint16_t)packet[18] << 4);
  channels[12] &= 0x07FF;

  channels[13] = ((uint16_t)packet[18] >> 7) | ((uint16_t)packet[19] << 1) | ((uint16_t)packet[20] << 9);
  channels[13] &= 0x07FF;

  channels[14] = ((uint16_t)packet[20] >> 2) | ((uint16_t)packet[21] << 6);
  channels[14] &= 0x07FF;

  channels[15] = ((uint16_t)packet[21] >> 5) | ((uint16_t)packet[22] << 3);
  channels[15] &= 0x07FF;

  SBUS_StoreChannels(channels);
  return 1;
}

/**
  * @brief  SBUS 通道数据存入 (范围 0~2047 原始值)
  *         CH1->rx, CH2->ry, CH3->ly, CH4->lx
  */
static void SBUS_StoreChannels(uint16_t *channels)
{
  ps2_data.rx = channels[0];   /* CH1 -> 右摇杆 X */
  ps2_data.ry = channels[1];   /* CH2 -> 右摇杆 Y */
  ps2_data.ly = channels[2];   /* CH3 -> 左摇杆 Y */
  ps2_data.lx = channels[3];   /* CH4 -> 左摇杆 X */
  ps2_data.ch6 = channels[5];  /* CH6 -> O 键等 */
  ps2_data.connected = 1;
}

/**
  * @brief  解析单个 SBUS 字节 (由 UART 接收回调调用)
  */
void PS2_Receiver_ParseByte(uint8_t byte)
{
  switch (sbus_state) {
    case SBUS_STATE_WAIT_HEADER:
      if (byte == SBUS_HEADER) {
        sbus_rx_buffer[0] = byte;
        sbus_rx_index = 1;
        sbus_state = SBUS_STATE_RECEIVING;
      }
      break;

    case SBUS_STATE_RECEIVING:
      sbus_rx_buffer[sbus_rx_index++] = byte;

      if (sbus_rx_index >= SBUS_PACKET_LENGTH) {
        if (sbus_rx_buffer[24] == SBUS_FOOTER) {
          SBUS_Parse_Packet(sbus_rx_buffer);
        }
        sbus_state = SBUS_STATE_WAIT_HEADER;
        sbus_rx_index = 0;
      }
      break;

    default:
      sbus_state = SBUS_STATE_WAIT_HEADER;
      sbus_rx_index = 0;
      break;
  }
}

/* UART 接收中断接收缓冲 SBUS */
static uint8_t sbus_uart_rx_byte;

void PS2_Receiver_Init(void)
{
  /* 初始化中位值 (SBUS 中心 1024) */
  ps2_data.lx        = 1024;
  ps2_data.ly        = 1024;
  ps2_data.rx        = 1024;
  ps2_data.ry        = 1024;
  ps2_data.ch6       = 1024;
  ps2_data.buttons   = 0;
  ps2_data.connected = 1;   /* 初始化已连接 */

  sbus_rx_index = 0;
  sbus_state    = SBUS_STATE_WAIT_HEADER;

  /* 启动 USART2 接收中断 (SBUS) */
  HAL_UART_Receive_IT(&huart2, &sbus_uart_rx_byte, 1);
}

/**
  * @brief  UART 接收完成回调，解析 SBUS 数据
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) {
    PS2_Receiver_ParseByte(sbus_uart_rx_byte);
    HAL_UART_Receive_IT(&huart2, &sbus_uart_rx_byte, 1);
  }
}

void PS2_Receiver_GetData(PS2_Data_TypeDef *data)
{
  if (data == NULL) return;

  data->lx        = ps2_data.lx;
  data->ly        = ps2_data.ly;
  data->rx        = ps2_data.rx;
  data->ry        = ps2_data.ry;
  data->ch6       = ps2_data.ch6;
  data->buttons   = ps2_data.buttons;
  data->connected = ps2_data.connected;
}
