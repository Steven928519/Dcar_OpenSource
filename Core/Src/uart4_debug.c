/**
  ******************************************************************************
  * @file    uart4_debug.c
  * @brief   UART4 波形调试输出实现 (PC10 TX)
  ******************************************************************************
  */
#include "uart4_debug.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

/* UART4 句柄 */
static UART_HandleTypeDef huart4;

void UART4_Debug_Init(void)
{
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart4) != HAL_OK) {
    /* 初始化失败可在此处理 */
  }
}

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
