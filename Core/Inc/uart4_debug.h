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
  * @brief  发送 M1 电机 PID 波形数据 (每 10ms 调用一次)
  * @param  target_speed: 目标 (脉冲/10ms)
  * @param  current_speed: 实际 (脉冲/10ms)
  * @param  error: 误差 (target - current)
  * @param  pwm_output: PID 输出 (PWM 值)
  * @note   字符串格式: "target,current,error,pwm\n" (CSV)
  */
void UART4_Debug_SendWaveform(float target_speed, float current_speed,
                              float error, float pwm_output);

#ifdef __cplusplus
}
#endif

#endif /* __UART4_DEBUG_H__ */
