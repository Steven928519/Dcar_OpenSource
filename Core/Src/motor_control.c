/**
  ******************************************************************************
  * @file    motor_control.c
  * @brief   四电机 PID 速度闭环控制实现
  ******************************************************************************
  */
#include "motor_control.h"
#include "encoder.h"
#include "pid.h"
#include "tim.h"
#include "gpio.h"
#include "uart4_debug.h"

/* 电机方向引脚: IN1_Pin, IN2_Pin */
static const uint16_t motor_in1_pin[] = { AIN1_Pin, BIN1_Pin, CIN1_Pin, DIN1_Pin };
static const uint16_t motor_in2_pin[] = { AIN2_Pin, BIN2_Pin, CIN2_Pin, DIN2_Pin };
static GPIO_TypeDef *motor_gpio_port = GPIOD;

/* 目标速度 */
static float target_speed[4] = {0};

/* PID 控制器 */
static PID_TypeDef motor_pid[4];

/* M1、M4 方向取反（电机安装方向与 M2、M3 相反） */
#define MOTOR_DIR_INVERT(m)  (((m) == 0 || (m) == 3) ? -1 : 1)

/* 速度(脉冲/秒) -> 每周期脉冲(脉冲/10ms): 除以 100 */
#define SPEED_TO_DELTA(s)   ((s) / 100.0f)

/* 设置电机 PWM 占空比 (0~MOTOR_PWM_PERIOD)，并控制方向 */
static void Motor_SetPWM(uint8_t motor_id, int16_t pwm_val)
{
  if (motor_id >= 4) return;

  pwm_val *= MOTOR_DIR_INVERT(motor_id);

  uint16_t pwm_abs;
  if (pwm_val >= 0) {
    pwm_abs = (pwm_val > MOTOR_PWM_PERIOD) ? MOTOR_PWM_PERIOD : (uint16_t)pwm_val;
    HAL_GPIO_WritePin(motor_gpio_port, motor_in1_pin[motor_id], GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor_gpio_port, motor_in2_pin[motor_id], GPIO_PIN_RESET);
  } else {
    pwm_abs = (-pwm_val > MOTOR_PWM_PERIOD) ? MOTOR_PWM_PERIOD : (uint16_t)(-pwm_val);
    HAL_GPIO_WritePin(motor_gpio_port, motor_in1_pin[motor_id], GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor_gpio_port, motor_in2_pin[motor_id], GPIO_PIN_SET);
  }

  switch (motor_id) {
    case 0: __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwm_abs); break;
    case 1: __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwm_abs); break;
    case 2: __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, pwm_abs); break;
    case 3: __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, pwm_abs); break;
    default: break;
  }
}

void MotorControl_Init(void)
{
  Encoder_Init();

  /* 启动 TIM6 定时中断 (1ms)，用于 10ms 周期调用 MotorControl_Update */
  HAL_TIM_Base_Start_IT(&htim6);

  /* 启动 PWM */
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

  /* 初始化 PID，输出范围 -999 ~ 999 (支持正反转) */
  for (uint8_t i = 0; i < 4; i++) {
    PID_Init(&motor_pid[i], MOTOR_PID_KP_DEFAULT, MOTOR_PID_KI_DEFAULT,
             MOTOR_PID_KD_DEFAULT, (float)MOTOR_PWM_PERIOD, -(float)MOTOR_PWM_PERIOD);
    target_speed[i] = 0.0f;
  }

  /* UART4 波形调试输出初始化 (PC10, 115200) */
  UART4_Debug_Init();
}

void MotorControl_SetTargetSpeed(uint8_t motor_id, float target_speed_val)
{
  if (motor_id < 4) {
    target_speed[motor_id] = target_speed_val;
  }
}

void MotorControl_SetAllTargetSpeed(float s1, float s2, float s3, float s4)
{
  target_speed[0] = s1;
  target_speed[1] = s2;
  target_speed[2] = s3;
  target_speed[3] = s4;
}

void MotorControl_SetAllTargetSpeedMMps(float mm_s1, float mm_s2, float mm_s3, float mm_s4)
{
  target_speed[0] = mm_s1 * COUNTS_PER_MM;
  target_speed[1] = mm_s2 * COUNTS_PER_MM;
  target_speed[2] = mm_s3 * COUNTS_PER_MM;
  target_speed[3] = mm_s4 * COUNTS_PER_MM;
}

void MotorControl_Update(void)
{
  Encoder_UpdateSpeed();

  for (uint8_t i = 0; i < 4; i++) {
    float current_speed = Encoder_GetSpeed(i) * MOTOR_DIR_INVERT(i);
    /* 转为每周期脉冲 (脉冲/10ms)，与参考工程量纲一致 */
    float target_delta = SPEED_TO_DELTA(target_speed[i]);
    float current_delta = SPEED_TO_DELTA(current_speed);
    float out = PID_Calc(&motor_pid[i], target_delta, current_delta);
    Motor_SetPWM(i, (int16_t)out);

    /* M1 电机波形数据通过 UART4(PC10) 每 10ms 发送一帧 (单位: 脉冲/10ms) */
    if (i == 0) {
      float err = target_delta - current_delta;
      UART4_Debug_SendWaveform(target_delta, current_delta, err, out);
    }
  }
}

void MotorControl_SetPID(uint8_t motor_id, float kp, float ki, float kd)
{
  if (motor_id < 4) {
    PID_Init(&motor_pid[motor_id], kp, ki, kd,
             (float)MOTOR_PWM_PERIOD, -(float)MOTOR_PWM_PERIOD);
  }
}

void MotorControl_StopAll(void)
{
  for (uint8_t i = 0; i < 4; i++) {
    target_speed[i] = 0.0f;
    PID_Reset(&motor_pid[i]);
    Motor_SetPWM(i, 0);
  }
}

float MotorControl_GetSpeed(uint8_t motor_id)
{
  if (motor_id >= 4) return 0.0f;
  return Encoder_GetSpeed(motor_id) * MOTOR_DIR_INVERT(motor_id);
}

float MotorControl_GetSpeedMMps(uint8_t motor_id)
{
  if (motor_id >= 4) return 0.0f;
  return Encoder_GetSpeedMMps(motor_id) * MOTOR_DIR_INVERT(motor_id);
}
