/**
 ******************************************************************************
 * @file    remote_to_motion.h
 * @brief   遥控信号转车体目标速度
 *
 * 控制对应关系:
 *   LY  -> Vy  -> 前后移动 (正=前，负=后)
 *   LX  -> Vx  -> 左右移动 (正=右，负=左)
 *   RX  -> w   -> 旋转     (正=顺时针)
 *
 * 速度映射: SBUS 范围 240~1807，中位 1024 映射为 0，两端映射到
 *±REMOTE_MAX_SPEED mm/s
 ******************************************************************************
 */
#ifndef __REMOTE_TO_MOTION_H__
#define __REMOTE_TO_MOTION_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* SBUS 输入范围 */
#define REMOTE_SBUS_MIN 240
#define REMOTE_SBUS_MAX 1807

/* SBUS 中位值 (240~1807 中心) */
#define REMOTE_CENTER 1024

/* 有效半程 (中位到最小值的距离，用于速度缩放) */
#define REMOTE_RANGE (REMOTE_CENTER - REMOTE_SBUS_MIN) /* 784 */

/* 死区范围，摇杆在 [1024-REMOTE_DEADZONE, 1024+REMOTE_DEADZONE] 内视为居中 */
#define REMOTE_DEADZONE 200

/* 速度最大值 (mm/s)，摇杆推满时 Vy/Vx/w 的幅值 */
#define REMOTE_MAX_SPEED 800.0f

/**
 * @brief  遥控转运动模块初始化
 */
void RemoteToMotion_Init(void);

/**
 * @brief  根据遥控信号更新车体目标速度
 * @note   需在固定周期调用 (如 10ms)，内部会设置四电机目标线速度 (mm/s)
 */
void RemoteToMotion_Update(float current_yaw, uint8_t is_remote_enabled);

#ifdef __cplusplus
}
#endif

#endif /* __REMOTE_TO_MOTION_H__ */
