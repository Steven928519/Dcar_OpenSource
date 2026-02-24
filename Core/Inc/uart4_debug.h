/**
  ******************************************************************************
  * @file    uart4_debug.h
  * @brief   UART4 波形调试输出 (PC10 TX)，用于串口波形显示工具
  ******************************************************************************
  */
#ifndef __UART4_DEBUG_H__
#define __UART4_DEBUG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
  * @brief  发送 PS2 摇杆四通道数据 (lx, ly, rx, ry)
  * @param  lx: 左摇杆 X 轴 (0~2047)
  * @param  ly: 左摇杆 Y 轴 (0~2047)
  * @param  rx: 右摇杆 X 轴 (0~2047)
  * @param  ry: 右摇杆 Y 轴 (0~2047)
  * @note   字符串格式: "lx,ly,rx,ry\n" (CSV)，供波形显示工具解析
  */
void UART4_Debug_SendPS2Joystick(uint16_t lx, uint16_t ly, uint16_t rx, uint16_t ry);

#ifdef __cplusplus
}
#endif

#endif /* __UART4_DEBUG_H__ */
