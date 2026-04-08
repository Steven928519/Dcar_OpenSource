/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   四路电机编码器读取实现
  ******************************************************************************
  */
#include "encoder.h"
#include "tim.h"
#include <string.h>

/* 编码器计数值 (16位有符号，处理溢出) */
static int32_t encoder_count[MOTOR_NUM];
static int32_t encoder_count_last[MOTOR_NUM];
static float encoder_speed[MOTOR_NUM];

/* 将 16 位无符号计数值转为有符号增量 (处理 0xFFFF -> 0 的溢出) */
static inline int16_t count_to_delta(uint16_t now, uint16_t last)
{
  return (int16_t)(now - last);
}

void Encoder_Init(void)
{
  memset(encoder_count, 0, sizeof(encoder_count));
  memset(encoder_count_last, 0, sizeof(encoder_count_last));
  memset(encoder_speed, 0, sizeof(encoder_speed));

  /* 启动编码器接口 */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
}

uint16_t Encoder_GetCount(uint8_t motor_id)
{
  if (motor_id >= MOTOR_NUM) return 0;

  switch (motor_id) {
    case ENC_M1: return (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    case ENC_M2: return (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    case ENC_M3: return (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    case ENC_M4: return (uint16_t)__HAL_TIM_GET_COUNTER(&htim5);
    default: return 0;
  }
}

void Encoder_UpdateSpeed(void)
{
  uint16_t raw[MOTOR_NUM];
  raw[ENC_M1] = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
  raw[ENC_M2] = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  raw[ENC_M3] = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
  raw[ENC_M4] = (uint16_t)__HAL_TIM_GET_COUNTER(&htim5);

  for (uint8_t i = 0; i < MOTOR_NUM; i++) {
    int16_t delta = count_to_delta(raw[i], (uint16_t)encoder_count_last[i]);
    encoder_count[i] += delta;
    encoder_count_last[i] = raw[i];
    /* 速度 = 增量 / 时间(s)，应用编码器极性 */
    encoder_speed[i] = (float)(delta * DF_WHEEL_POLARITY) * 1000.0f / (float)ENCODER_SAMPLE_MS;
  }
}

float Encoder_GetSpeed(uint8_t motor_id)
{
  if (motor_id >= MOTOR_NUM) return 0.0f;
  return encoder_speed[motor_id];
}

float Encoder_GetSpeedMMps(uint8_t motor_id)
{
  if (motor_id >= MOTOR_NUM) return 0.0f;
  /* 计数/秒 -> mm/s: 除以每 mm 的计数 */
  return encoder_speed[motor_id] / COUNTS_PER_MM;
}

void Encoder_ClearCount(uint8_t motor_id)
{
  if (motor_id >= MOTOR_NUM) return;
  encoder_count[motor_id] = 0;
  encoder_count_last[motor_id] = 0;
  encoder_speed[motor_id] = 0.0f;
  switch (motor_id) {
    case ENC_M1: __HAL_TIM_SET_COUNTER(&htim2, 0); break;
    case ENC_M2: __HAL_TIM_SET_COUNTER(&htim3, 0); break;
    case ENC_M3: __HAL_TIM_SET_COUNTER(&htim4, 0); break;
    case ENC_M4: __HAL_TIM_SET_COUNTER(&htim5, 0); break;
    default: break;
  }
}

void Encoder_ClearAll(void)
{
  for (uint8_t i = 0; i < MOTOR_NUM; i++) {
    Encoder_ClearCount(i);
  }
}
