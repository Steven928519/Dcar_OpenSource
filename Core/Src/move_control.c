/**
 * @file    move_control.c
 * @brief   位置环闭环控制逻辑实现
 */
#include "move_control.h"
#include "motor_control.h"
#include "odometry.h"
#include "pid.h"
#include <math.h>
#include <stdio.h>

/* 定制 PID 参数 */
#define POS_KP 10.0f
#define POS_KI 0.0f
#define POS_KD 0.1f
#define YAW_KP 10.0f /* 航向保持 KP */

#define MAX_SPEED 500.0f      /* 最大平移速度 mm/s */
#define MAX_W 100.0f          /* 最大旋转速度 mm/s (对应轮端) */
#define ARRIVE_DEADZONE 10.0f /* 到达死区 10mm */

static MoveState_t current_state = MOVE_IDLE;
static float target_xw = 0, target_yw = 0, target_yaw = 0;
static float speed_limit = MAX_SPEED; // 默认使用最大速度
static PID_TypeDef pid_x, pid_y, pid_w;

void MoveControl_Init(void) {
  /* 初始化位置 PID: 输入误差(mm) -> 输出速度(mm/s) */
  PID_Init(&pid_x, POS_KP, POS_KI, POS_KD, MAX_SPEED, -MAX_SPEED);
  PID_Init(&pid_y, POS_KP, POS_KI, POS_KD, MAX_SPEED, -MAX_SPEED);
  /* 初始化航向 PID: 输入误差(deg) -> 输出旋转分量(mm/s) */
  PID_Init(&pid_w, YAW_KP, 0, 0, MAX_W, -MAX_W);

  current_state = MOVE_IDLE;
}

void MoveControl_SetRelativeTarget(float rel_x, float rel_y,
                                   float target_yaw_deg,
                                   float speed_limit_mmps) {
  Odometry_TypeDef odo;
  Odometry_GetData(&odo);

  /* 设置目标世界坐标 */
  target_xw = odo.x + rel_x;
  target_yw = odo.y + rel_y;
  target_yaw = target_yaw_deg;

  /* 设置速度上限 */
  if (speed_limit_mmps > 0) {
    speed_limit = (speed_limit_mmps > MAX_SPEED) ? MAX_SPEED : speed_limit_mmps;
  } else {
    speed_limit = MAX_SPEED;
  }

  /* 重置 PID 并更新速度限制 */
  PID_Init(&pid_x, POS_KP, POS_KI, POS_KD, speed_limit, -speed_limit);
  PID_Init(&pid_y, POS_KP, POS_KI, POS_KD, speed_limit, -speed_limit);
  PID_Reset(&pid_x);
  PID_Reset(&pid_y);
  PID_Reset(&pid_w);

  current_state = MOVE_EXECUTING;
  printf("Move Start: Target X:%.1f Y:%.1f Yaw:%.1f, Speed Limit:%.1f\n",
         target_xw, target_yw, target_yaw, speed_limit);
}

static void Motion_Decouple(float Vy, float Vx, float w) {
  /* 电机物理位置: M1=左前, M2=右前, M3=右后, M4=左后 */
  float s1 = Vy + Vx + w;
  float s2 = Vy - Vx - w;
  float s3 = Vy + Vx - w;
  float s4 = Vy - Vx + w;

  MotorControl_SetAllTargetSpeedMMps(s1, s2, s3, s4);
}

void MoveControl_Update(void) {
  if (current_state != MOVE_EXECUTING)
    return;

  Odometry_TypeDef odo;
  Odometry_GetData(&odo);

  /* 1. 计算世界系误差 */
  float err_xw = target_xw - odo.x;
  float err_yw = target_yw - odo.y;
  float err_yaw = target_yaw - odo.yaw;

  /* 2. 检查是否到达 */
  float dist = sqrtf(err_xw * err_xw + err_yw * err_yw);
  if (dist < ARRIVE_DEADZONE) {
    current_state = MOVE_FINISHED;
    Motion_Decouple(0, 0, 0);
    printf("Move Finished at X:%.1f Y:%.1f\n", odo.x, odo.y);
    return;
  }

  /* 3. 世界系下的 PID 计算目标速度 */
  float v_xw = PID_Calc(&pid_x, target_xw, odo.x);
  float v_yw = PID_Calc(&pid_y, target_yw, odo.y);
  float w_out = PID_Calc(&pid_w, target_yaw, odo.yaw);

  /* 4. 将世界系速度投影到机器人坐标系 */
  float yaw_rad = odo.yaw * 0.01745329f;
  float cos_y = cosf(yaw_rad);
  float sin_y = sinf(yaw_rad);

  /* 旋转矩阵逆变换: x_r = x_w*cos - y_w*sin, y_r = x_w*sin + y_w*cos */
  float vx_r = v_xw * cos_y - v_yw * sin_y;
  float vy_r = v_xw * sin_y + v_yw * cos_y;

  /* 5. 下发电机控制 */
  Motion_Decouple(vy_r, vx_r, w_out);
}

MoveState_t MoveControl_GetState(void) { return current_state; }

void MoveControl_Stop(void) {
  current_state = MOVE_IDLE;
  Motion_Decouple(0, 0, 0);
}
