/**
  ******************************************************************************
  * @file    remote_to_motion.c
  * @brief   遥控信号转车体目标速度实现
  *          解耦公式: M1=Vx+Vy+w, M2=Vx-Vy+w, M3=-Vx-Vy+w, M4=-Vx+Vy+w
  ******************************************************************************
  */
#include "remote_to_motion.h"
#include "ps2_receiver.h"
#include "motor_control.h"

/**
  * @brief  原始值转速度，带死区
  * @param  raw: 摇杆原始值 0~2047
  * @return 速度 (mm/s)，死区内为 0
  */
static inline float RawToSpeedWithDeadzone(uint16_t raw)
{
  if (raw >= REMOTE_CENTER - REMOTE_DEADZONE && raw <= REMOTE_CENTER + REMOTE_DEADZONE) {
    return 0.0f;
  }
  return ((float)(int16_t)((int32_t)raw - REMOTE_CENTER)) * REMOTE_MAX_SPEED / REMOTE_CENTER;
}

/**
  * @brief  解耦计算四电机目标速度 (mm/s)
  * @param  Vx: 前后速度 (LY 控制，正=前)
  * @param  Vy: 左右速度 (LX 控制，正=右)
  * @param  w:  旋转速度 (RX 控制，正=顺时针)
  */
static void Motion_Decouple(float Vx, float Vy, float w)
{
  float s1 =  Vx + Vy + w;   /* M1 */
  float s4 =  Vx - Vy + w;   /* M4 */
  float s3 = -(-Vx - Vy + w);   /* M3 */
  float s2 = -(-Vx + Vy + w);   /* M2 */

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

  /* 死区 1024±200 视为居中，摇杆推满映射到 ±REMOTE_MAX_SPEED mm/s */
  float Vx = RawToSpeedWithDeadzone(data.ly);   /* LY: 前后 */
  float Vy = RawToSpeedWithDeadzone(data.lx);   /* LX: 左右 */
  float w  = RawToSpeedWithDeadzone(data.rx);   /* RX: 旋转 */

  Motion_Decouple(Vx, Vy, w);
}
