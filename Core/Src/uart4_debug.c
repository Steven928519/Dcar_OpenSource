/**
  ******************************************************************************
  * @file    uart4_debug.c
  * @brief   UART4 波形调试输出实现 (PC10 TX)，使用 CubeMX 生成的 huart4
  ******************************************************************************
  */
#include "uart4_debug.h"
#include "usart.h"
#include <stdio.h>

void UART4_Debug_SendWaveform(float target_speed, float current_speed,
                              float error, float pwm_output)
{
  char buf[64];
  int len = snprintf(buf, sizeof(buf), "%.2f,%.2f,%.2f,%.2f\n",
                    (double)target_speed, (double)current_speed,
                    (double)error, (double)pwm_output);
  if (len > 0) {
    HAL_UART_Transmit(&huart4, (uint8_t *)buf, (uint16_t)len, 10);
  }
}
