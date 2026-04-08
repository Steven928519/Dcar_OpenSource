/**
 * @file    odometry.h
 * @brief   里程计估计算法实现
 *          利用四轮编码器及 IMU Yaw 轴进行航位推算
 */
#ifndef __ODOMETRY_H__
#define __ODOMETRY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * @brief  里程计数据结构
 */
typedef struct {
  float x;   /* 世界坐标系 X (mm) */
  float y;   /* 世界坐标系 Y (mm) */
  float yaw; /* 世界坐标系航向角 (degree) */

  float vx; /* 机器人体系 X 速度 (mm/s) */
  float vy; /* 机器人体系 Y 速度 (mm/s) */
  float w;  /* 机器人体系旋转角速度 (计算值) */
} Odometry_TypeDef;

/**
 * @brief  初始化里程计
 */
void Odometry_Init(void);

/**
 * @brief  更新里程计 (建议在 100Hz 频率调用)
 * @param  imu_yaw: 当前 IMU 提供的航向角 (degree)
 * @param  dt: 两次更新之间的时间间隔 (s)
 */
void Odometry_Update(float imu_yaw, float dt);

/**
 * @brief  获取当前里程计数据
 */
void Odometry_GetData(Odometry_TypeDef *data);

/**
 * @brief  重置里程计坐标
 */
void Odometry_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __ODOMETRY_H__ */
