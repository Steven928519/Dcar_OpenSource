/**
  ******************************************************************************
  * @file    pid.h
  * @brief   PID 控制器模块，用于电机速度闭环
  ******************************************************************************
  */
#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* PID 结构体 */
typedef struct {
  float Kp;           /* 比例系数 */
  float Ki;           /* 积分系数 */
  float Kd;           /* 微分系数 */
  float integral;     /* 积分累积 */
  float last_error;   /* 上次误差 */
  float out_max;      /* 输出上限 */
  float out_min;      /* 输出下限 */
  float integral_max; /* 积分限幅，抗饱和 */
} PID_TypeDef;

/**
  * @brief  PID 参数初始化
  * @param  pid: PID 结构体指针
  * @param  kp, ki, kd: PID 参数
  * @param  out_max: 输出上限 (如 PWM 最大值 999)
  * @param  out_min: 输出下限 (如 0 或 -999 支持反转)
  */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float out_max, float out_min);

/**
  * @brief  PID 计算
  * @param  pid: PID 结构体指针
  * @param  target: 目标值 (期望速度)
  * @param  current: 当前值 (反馈速度)
  * @retval PID 输出
  */
float PID_Calc(PID_TypeDef *pid, float target, float current);

/**
  * @brief  重置 PID 内部状态
  */
void PID_Reset(PID_TypeDef *pid);

/**
  * @brief  设置积分限幅，防止积分饱和
  */
void PID_SetIntegralLimit(PID_TypeDef *pid, float limit);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H__ */
