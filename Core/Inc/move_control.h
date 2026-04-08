/**
 * @file    move_control.h
 * @brief   位置环闭环控制模块 (相对位移)
 */
#ifndef __MOVE_CONTROL_H__
#define __MOVE_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * @brief  移动任务状态
 */
typedef enum {
  MOVE_IDLE,      /* 空闲 */
  MOVE_EXECUTING, /* 执行中 */
  MOVE_FINISHED   /* 已完成 */
} MoveState_t;

/**
 * @brief  初始化移动控制模块
 */
void MoveControl_Init(void);

/**
 * @brief  设置相对位移目标
 * @param  rel_x: 相对 X 位移 (mm, +为右)
 * @param  rel_y: 相对 Y 位移 (mm, +为前)
 * @param  target_yaw: 目标航向角 (degree)，通常传当前角度以维持直行
 */
void MoveControl_SetRelativeTarget(float rel_x, float rel_y, float target_yaw,
                                   float speed_limit_mmps);

/**
 * @brief  主更新循环 (建议 100Hz 频率调用)
 */
void MoveControl_Update(void);

/**
 * @brief  获取当前移动状态
 */
MoveState_t MoveControl_GetState(void);

/**
 * @brief  紧急停止移动任务
 */
void MoveControl_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOVE_CONTROL_H__ */
