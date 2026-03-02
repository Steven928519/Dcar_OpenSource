#include "remote_to_motion.h"
#include "motor_control.h"
#include "pid.h"
#include "ps2_receiver.h"

/* Yaw 轴锁定相关变量 */
static PID_TypeDef yaw_pid;
static float target_yaw = 0.0f;
static uint8_t is_yaw_locked = 0;

/**
 * @brief  原始值转速度，带死区
 * @param  raw: 摇杆原始值 240~1807 (SBUS 实际范围)
 * @return 速度 (mm/s)，死区内为 0，240->-800，1807->+800
 */
static inline float RawToSpeedWithDeadzone(uint16_t raw) {
  if (raw >= REMOTE_CENTER - REMOTE_DEADZONE &&
      raw <= REMOTE_CENTER + REMOTE_DEADZONE) {
    return 0.0f;
  }
  return ((float)((int32_t)raw - REMOTE_CENTER)) * REMOTE_MAX_SPEED /
         REMOTE_RANGE;
}

/**
 * @brief  解耦计算四电机目标速度 (mm/s)
 * @param  Vy: 前后速度 (LY 控制，正=前，负=后)
 * @param  Vx: 左右速度 (LX 控制，正=右，负=左)
 * @param  w:  旋转速度 (RX 控制，正=顺时针)
 */
static void Motion_Decouple(float Vy, float Vx, float w) {
  /* 电机物理位置: s1=右前(PC6), s2=右后(PC7), s3=左后(PC8), s4=左前(PC9) */
  float s1 = Vy + Vx + w; /* 右前 */
  float s2 = Vy - Vx - w; /* 右后 */
  float s3 = Vy + Vx - w; /* 左后 */
  float s4 = Vy - Vx + w; /* 左前 */

  MotorControl_SetAllTargetSpeedMMps(s1, s2, s3, s4);
}

void RemoteToMotion_Init(void) {
  /* Yaw PID: Kp=60, Ki=0, Kd=0.0, 输出限幅 ±800 mm/s */
  PID_Init(&yaw_pid, 60.0f, 0.7f, 0.0f, 800.0f, -800.0f);

  /* 初始静止 */
  MotorControl_SetAllTargetSpeedMMps(0.0f, 0.0f, 0.0f, 0.0f);
}

void RemoteToMotion_Update(float current_yaw, uint8_t is_remote_enabled) {
  PS2_Data_TypeDef data;

  PS2_Receiver_GetData(&data);

  float Vy = 0.0f;
  float Vx = 0.0f;
  float w = 0.0f;

  /* 只有在遥控模式开启且遥控器已连接时，才读取前后左右指令 */
  if (is_remote_enabled && data.connected) {
    Vy = RawToSpeedWithDeadzone(data.ly); /* LY: 前后 */
    Vx = RawToSpeedWithDeadzone(data.lx); /* LX: 左右 */
    w = RawToSpeedWithDeadzone(data.rx);  /* RX: 旋转 */
  }

  /* 锁头逻辑实现 (核心：只要没有手动旋转指令 w，就自动维持当前 yaw) */
  if (w != 0.0f) {
    /* 用户手动主动转弯中，清除偏差状态 */
    is_yaw_locked = 0;
    target_yaw = current_yaw;
    PID_Reset(&yaw_pid);
  } else {
    /* 旋转指令为 0：进入/维持航向保持模式 */
    if (!is_yaw_locked) {
      is_yaw_locked = 1;
      target_yaw = current_yaw;
      PID_Reset(&yaw_pid);
    }

    /* 无死区：全程由 PID 持续纠偏 */
    w = PID_Calc(&yaw_pid, target_yaw, current_yaw);
  }

  /* 最终解耦并下发到电机 */
  Motion_Decouple(Vy, Vx, w);
}
