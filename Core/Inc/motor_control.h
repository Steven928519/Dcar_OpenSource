/**
  ******************************************************************************
  * @file    motor_control.h
  * @brief   四电机 PID 速度闭环控制模块
  *          整合编码器 + PID + PWM 输出
  ******************************************************************************
  */
#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "encoder.h"

/* PWM 周期 (与 TIM8 Period 一致) */
#define MOTOR_PWM_PERIOD     999

/* 默认 PID 参数 (与参考工程一致：Error 单位=脉冲/10ms，KP 映射到 0~999 PWM) */
#define MOTOR_PID_KP_DEFAULT  3.0f
#define MOTOR_PID_KI_DEFAULT  0.0f
#define MOTOR_PID_KD_DEFAULT  0.0f

/**
  * @brief  电机控制模块初始化
  * @note   会初始化编码器并启动 PWM，设置默认 PID 参数
  */
void MotorControl_Init(void);

/**
  * @brief  设置指定电机的目标速度 (编码器计数/秒)
  * @param  motor_id: 电机编号 0~3
  * @param  target_speed: 目标速度，正为正向，负为反向
  */
void MotorControl_SetTargetSpeed(uint8_t motor_id, float target_speed);

/**
  * @brief  设置四个电机的目标速度 (编码器计数/秒)
  */
void MotorControl_SetAllTargetSpeed(float s1, float s2, float s3, float s4);

/**
  * @brief  设置四个电机的目标线速度 (mm/s)
  */
void MotorControl_SetAllTargetSpeedMMps(float mm_s1, float mm_s2, float mm_s3, float mm_s4);

/**
  * @brief  PID 速度环更新，需在固定周期调用 (如 10ms 定时器中断)
  * @note   内部会调用 Encoder_UpdateSpeed，再对每个电机做 PID 并输出 PWM
  */
void MotorControl_Update(void);

/**
  * @brief  设置指定电机的 PID 参数
  */
void MotorControl_SetPID(uint8_t motor_id, float kp, float ki, float kd);

/**
  * @brief  停止所有电机
  */
void MotorControl_StopAll(void);

/**
  * @brief  获取指定电机当前速度 (编码器计数/秒)
  */
float MotorControl_GetSpeed(uint8_t motor_id);

/**
  * @brief  获取指定电机当前线速度 (mm/s)
  */
float MotorControl_GetSpeedMMps(uint8_t motor_id);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H__ */
