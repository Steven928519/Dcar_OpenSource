/**
  ******************************************************************************
  * @file    remote_to_motion.c
  * @brief   遥控信号转车体目标速度实现
  *          解耦公式: Vy=前后(负=后), Vx=左右(负=左)
  ******************************************************************************
  */
#include "remote_to_motion.h"
#include "ps2_receiver.h"
#include "motor_control.h"

/**
  * @brief  原始值转速度，带死区
  * @param  raw: 摇杆原始值 240~1807 (SBUS 实际范围)
  * @return 速度 (mm/s)，死区内为 0，240->-800，1807->+800
  */
static inline float RawToSpeedWithDeadzone(uint16_t raw)
{
  if (raw >= REMOTE_CENTER - REMOTE_DEADZONE && raw <= REMOTE_CENTER + REMOTE_DEADZONE) {
    return 0.0f;
  }
  return ((float)((int32_t)raw - REMOTE_CENTER)) * REMOTE_MAX_SPEED / REMOTE_RANGE;
}

/**
  * @brief  解耦计算四电机目标速度 (mm/s)
  * @param  Vy: 前后速度 (LY 控制，正=前，负=后)
  * @param  Vx: 左右速度 (LX 控制，正=右，负=左)
  * @param  w:  旋转速度 (RX 控制，正=顺时针)
  */
static void Motion_Decouple(float Vy, float Vx, float w)
{
  /* 电机物理位置: s1=右前(PC6), s2=右后(PC7), s3=左后(PC8), s4=左前(PC9) */
  float s1 =  Vy + Vx + w;   /* 右前 */
  float s2 =  Vy - Vx - w;   /* 右后 */
  float s3 =  Vy + Vx - w;   /* 左后 */
  float s4 =  Vy - Vx + w;   /* 左前 */

  MotorControl_SetAllTargetSpeedMMps(s1, s2, s3, s4);
}

void RemoteToMotion_Init(void)
{
  /* 初始静止 */
  MotorControl_SetAllTargetSpeedMMps(0.0f, 0.0f, 0.0f, 0.0f);
}

void RemoteToMotion_Update(void)
{
  PS2_Data_TypeDef data;

  PS2_Receiver_GetData(&data);

  if (!data.connected) {
    MotorControl_StopAll();
    return;
  }

  /* SBUS 240~1807 映射，死区 1024±200 视为居中，两端映射到 ±REMOTE_MAX_SPEED mm/s */
  float Vy = RawToSpeedWithDeadzone(data.ly);   /* LY: 前后 (正=前，负=后) */
  float Vx = RawToSpeedWithDeadzone(data.lx);   /* LX: 左右 (正=右，负=左) */
  float w  = RawToSpeedWithDeadzone(data.rx);   /* RX: 旋转 */

  Motion_Decouple(Vy, Vx, w);
}
