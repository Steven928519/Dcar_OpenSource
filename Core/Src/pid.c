/**
  ******************************************************************************
  * @file    pid.c
  * @brief   PID 控制器实现
  ******************************************************************************
  */
#include "pid.h"
#include "encoder.h"
#include <string.h>

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float out_max, float out_min)
{
  if (pid == NULL) return;
  memset(pid, 0, sizeof(PID_TypeDef));
  pid->Kp = kp;
  pid->Ki = ki;
  pid->Kd = kd;
  pid->out_max = out_max;
  pid->out_min = out_min;
  pid->integral_max = out_max;  /* 默认积分限幅等于输出限幅 */
}

float PID_Calc(PID_TypeDef *pid, float target, float current)
{
  if (pid == NULL) return 0.0f;

  float error = target - current;
  float dt = (float)ENCODER_SAMPLE_MS / 1000.0f;  /* 采样周期(秒) */

  /* P */
  float p_out = pid->Kp * error;

  /* I */
  pid->integral += error * dt;
  if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
  if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
  float i_out = pid->Ki * pid->integral;

  /* D */
  float derivative = (error - pid->last_error) / dt;
  pid->last_error = error;
  float d_out = pid->Kd * derivative;

  float output = p_out + i_out + d_out;

  /* 输出限幅 */
  if (output > pid->out_max) output = pid->out_max;
  if (output < pid->out_min) output = pid->out_min;

  return output;
}

void PID_Reset(PID_TypeDef *pid)
{
  if (pid == NULL) return;
  pid->integral = 0.0f;
  pid->last_error = 0.0f;
}

void PID_SetIntegralLimit(PID_TypeDef *pid, float limit)
{
  if (pid == NULL) return;
  pid->integral_max = limit;
}
