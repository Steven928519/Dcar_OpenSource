/**
  ******************************************************************************
  * @file    uart4_debug.c
  * @brief   UART4 波形调试输出实现 (PC10 TX)，使用 CubeMX 生成的 huart4
  *          发送 PS2 摇杆 lx/ly/rx/ry 四通道数据，供波形显示工具查看
  ******************************************************************************
  */
#include "uart4_debug.h"
#include "usart.h"
#include <stdio.h>

void UART4_Debug_SendPS2Joystick(uint16_t lx, uint16_t ly, uint16_t rx, uint16_t ry)
{
  char buf[48];
  int len = snprintf(buf, sizeof(buf), "%u,%u,%u,%u\n",
                    (unsigned int)lx, (unsigned int)ly,
                    (unsigned int)rx, (unsigned int)ry);
  if (len > 0) {
    HAL_UART_Transmit(&huart4, (uint8_t *)buf, (uint16_t)len, 10);
  }
}
