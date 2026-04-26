/**
 * @file    odometry.c
 * @brief   里程计估计算法实现
 */
#include "odometry.h"
#include "motor_control.h"
#include <math.h>

static Odometry_TypeDef odo = {0};

/**
 * @brief  初始化里程计
 */
void Odometry_Init(void) { Odometry_Reset(); }

/**
 * @brief  更新里程计 (建议在 100Hz 频率调用)
 * @param  imu_yaw: 当前 IMU 提供的航向角 (degree)
 * @param  dt: 两次更新之间的时间间隔 (s)
 */
void Odometry_Update(float imu_yaw, float dt) {
  /* 1. 获取四个电机的实际线速度 (mm/s) */
  /* 这里 MotorControl_GetSpeedMMps 已经处理了电机极性和方向 */
  float s1 = MotorControl_GetSpeedMMps(0); /* 左前 */
  float s2 = MotorControl_GetSpeedMMps(1); /* 右前 */
  float s3 = MotorControl_GetSpeedMMps(2); /* 右后 */
  float s4 = MotorControl_GetSpeedMMps(3); /* 左后 */

  /* 2. 四轮编码器反解 Vx, Vy, w (机器人坐标系) */
  /* Vy: 前后 (+为前)
   * Vx: 左右 (+为右)
   * w:  旋转 (+为顺时针旋转，对应 Motion_Decouple 定义)
   * 注意：这些公式是 remote_to_motion.c 中解耦公式的逆运算
   */
  odo.vy = (s1 + s2 + s3 + s4) / 4.0f;
  odo.vx = (s1 - s2 + s3 - s4) / 4.0f;
  odo.w = (s1 - s2 - s3 + s4) / 4.0f;

  /* 3. 更新航向角 (直接取 IMU 值) */
  odo.yaw = imu_yaw;

  /* 4. 车体系到世界系变换并积分 (右手系定义：+Y 向前, +X 向右)
   * 当 Yaw = 0 时: dx = vx, dy = vy
   * 当 Yaw > 0 (顺时针旋转) 时: 车头向右偏，前进方向的分量会投影到 X 轴
   */
  float yaw_rad = odo.yaw * 0.01745329f; /* deg to rad */
  float cos_y = cosf(yaw_rad);
  float sin_y = sinf(yaw_rad);

  float dx = (odo.vx * cos_y + odo.vy * sin_y) * dt;
  float dy = (-odo.vx * sin_y + odo.vy * cos_y) * dt;

  odo.x += dx;
  odo.y += dy;
}

/**
 * @brief  获取当前里程计数据
 */
void Odometry_GetData(Odometry_TypeDef *data) {
  if (data) {
    *data = odo;
  }
}

/**
 * @brief  重置里程计坐标
 */
void Odometry_Reset(void) {
  odo.x = 0;
  odo.y = 0;
  odo.yaw = 0;
  odo.vx = 0;
  odo.vy = 0;
  odo.w = 0;
}
