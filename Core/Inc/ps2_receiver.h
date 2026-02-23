/**
  ******************************************************************************
  * @file    ps2_receiver.h
  * @brief   PS2 遥控器信号接收模块
  *          通过 PA3 接收 PS2 遥控器数据
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
  * @brief  PS2 摇杆/按键数据结构 (预留，具体逻辑后续实现)
  */
typedef struct {
  uint8_t  lx;           /* 左摇杆 X 轴 0~255 */
  uint8_t  ly;           /* 左摇杆 Y 轴 0~255 */
  uint8_t  rx;           /* 右摇杆 X 轴 0~255 */
  uint8_t  ry;           /* 右摇杆 Y 轴 0~255 */
  uint16_t buttons;      /* 按键位图 */
  uint8_t  connected;    /* 连接状态: 0=未连接, 1=已连接 */
} PS2_Data_TypeDef;

/**
  * @brief  PS2 接收模块初始化
  * @note   配置 PA3 为输入，初始化相关变量
  */
void PS2_Receiver_Init(void);

/**
  * @brief  获取当前 PS2 数据 (预留接口)
  * @param  data: 指向 PS2_Data_TypeDef 的指针，用于接收数据
  */
void PS2_Receiver_GetData(PS2_Data_TypeDef *data);

#ifdef __cplusplus
}
#endif

#endif /* __PS2_RECEIVER_H__ */
