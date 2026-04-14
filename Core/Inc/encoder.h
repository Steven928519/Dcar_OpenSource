/**
 ******************************************************************************
 * @file    encoder.h
 * @brief   四路电机编码器读取模块
 *          定时器编码器模式：TIM2(M1), TIM3(M2), TIM4(M3), TIM5(M4)
 ******************************************************************************
 */
#ifndef __ENCODER_H__
#define __ENCODER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 电机编号 */
#define MOTOR_NUM 4

/* 编码器与电机对应关系 */
#define ENC_M1 0 /* TIM2 - PA5, PB3 */
#define ENC_M2 1 /* TIM3 - PA6, PA7 */
#define ENC_M3 2 /* TIM4 - PB6, PB7 */
#define ENC_M4 3 /* TIM5 - PA0, PA1 */

/* DF 电机参数 */
#define DF_ENCODER_LINES 448    /* 编码器线数（每圈脉冲数） */
#define DF_MOTOR_REDUCTION 38   /* 电机减速比 */
#define DF_WHEEL_DIAMETER_MM 97 /* 轮子直径 (mm) */
#define DF_WHEEL_POLARITY 1     /* 编码器极性（1 正 / -1 反） */

/* 编码器参数（由 DF 参数派生） */
#define ENCODER_PPR DF_ENCODER_LINES
#define ENCODER_MULTIPLE 4 /* 4倍频 (AB相正交) */
#define ENCODER_COUNTS_REV                                                     \
  (ENCODER_PPR * ENCODER_MULTIPLE) /* 电机轴每圈总计数 */

/* 轮子相关：轮子转一圈的编码器计数 = 电机轴计数 * 减速比 */
#define WHEEL_COUNTS_REV (ENCODER_COUNTS_REV * DF_MOTOR_REDUCTION)
/* 轮周长 (mm) */
#define WHEEL_CIRCUMFERENCE_MM (3.14159265f * (float)DF_WHEEL_DIAMETER_MM)
/* 每 mm 对应的编码器计数 */
#define COUNTS_PER_MM ((float)WHEEL_COUNTS_REV / WHEEL_CIRCUMFERENCE_MM)

/* 速度计算周期 (ms)，需与 PID 调用周期一致 */
#define ENCODER_SAMPLE_MS 10

/**
 * @brief  编码器模块初始化，启动编码器
 */
void Encoder_Init(void);

/**
 * @brief  获取指定电机的编码器原始计数值
 * @param  motor_id: 电机编号 0~3 (M1~M4)
 * @retval 定时器 CNT 寄存器原始值，范围 0 ~ 65535
 */
uint16_t Encoder_GetCount(uint8_t motor_id);

/**
 * @brief  获取指定电机的速度 (编码器计数/秒，已应用极性)
 * @param  motor_id: 电机编号 0~3
 * @retval 速度值，正为正向，负为反向
 */
float Encoder_GetSpeed(uint8_t motor_id);

/**
 * @brief  获取轮子线速度 (mm/s)
 * @param  motor_id: 电机编号 0~3
 */
float Encoder_GetSpeedMMps(uint8_t motor_id);

/**
 * @brief  速度计算更新，需在固定周期调用 (如 10ms 定时器中断)
 * @note   根据上次计数值计算速度
 */
void Encoder_UpdateSpeed(void);

/**
 * @brief  清除指定电机编码器计数
 */
void Encoder_ClearCount(uint8_t motor_id);

/**
 * @brief  清除所有电机编码器计数
 */
void Encoder_ClearAll(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H__ */
