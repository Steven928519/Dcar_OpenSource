/**
  ******************************************************************************
  * @file    ps2_receiver.h
  * @brief   PS2 遥控器信号接收模块
  *          通过 PA3(USART2_RX) 接收 SBUS 信号，解析后直接存入变量 (0~2047)
  ******************************************************************************
  */
#ifndef __PS2_RECEIVER_H__
#define __PS2_RECEIVER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* PS2 数据接收引脚: PA3 (定义于 main.h) */

/**
  * @brief  SBUS 摇杆数据结构 (保留 SBUS 原始范围 0~2047)
  */
typedef struct {
  uint16_t lx;           /* 左摇杆 X 轴 (CH4) 0~2047 */
  uint16_t ly;           /* 左摇杆 Y 轴 (CH3) 0~2047 */
  uint16_t rx;           /* 右摇杆 X 轴 (CH1) 0~2047 */
  uint16_t ry;           /* 右摇杆 Y 轴 (CH2) 0~2047 */
  uint16_t buttons;      /* 按键位图 */
  uint8_t  connected;    /* 连接状态: 0=未连接, 1=已连接 */
} PS2_Data_TypeDef;

/**
  * @brief  PS2 接收模块初始化
  * @note   初始化 SBUS 解析状态，启动 USART2 接收
  */
void PS2_Receiver_Init(void);

/**
  * @brief  解析单个 SBUS 字节 (由 UART 接收回调调用)
  * @param  byte: 接收到的字节
  */
void PS2_Receiver_ParseByte(uint8_t byte);

/**
  * @brief  获取当前 PS2 数据 (预留接口)
  * @param  data: 指向 PS2_Data_TypeDef 的指针，用于接收数据
  */
void PS2_Receiver_GetData(PS2_Data_TypeDef *data);

#ifdef __cplusplus
}
#endif

#endif /* __PS2_RECEIVER_H__ */
