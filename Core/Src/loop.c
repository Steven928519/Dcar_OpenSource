/**
 ******************************************************************************
 * @file    loop.c
 * @brief   主循环任务分频调度实现
 *
 *  分频结构 (基于 HAL_GetTick 软件计时):
 *  ┌─────────────┬──────────┬──────────────────────────────────────────┐
 *  │  函数        │  周期    │  任务                                    │
 *  ├─────────────┼──────────┼──────────────────────────────────────────┤
 *  │ LOOP_1000HZ │  1  ms   │ IMU 读取 + Mahony 姿态解算               │
 *  │ LOOP_100HZ  │  10 ms   │ 遥控模式判断 + Yaw PID + 运动解耦        │
 *  │ LOOP_50HZ   │  20 ms   │ UART4 调试输出 (VOFA+)                   │
 *  └─────────────┴──────────┴──────────────────────────────────────────┘
 *
 *  注: 电机速度 PID (100Hz) 由 TIM6 硬件中断独立驱动，不在此调度，
 *      见 stm32f4xx_it.c → HAL_TIM_PeriodElapsedCallback。
 ******************************************************************************
 */
#include "loop.h"

#include "gpio.h"
#include "imu.h"
#include "main.h"
#include "motor_control.h"
#include "ps2_receiver.h"
#include "remote_to_motion.h"
#include "uart4_debug.h"
#include "usart1_control.h"

/* -------------------------------------------------------------------------- */
/* 模块内共享状态 */
/* -------------------------------------------------------------------------- */
static ICM20602_Data_TypeDef imu_raw;
static ICM20602_Attitude_TypeDef imu_angles = {0};
static IMU_InitState_t imu_status = IMU_STATE_RESET;
static uint8_t remote_mode_enabled = 0;

/* -------------------------------------------------------------------------- */
/* 1000 Hz — IMU 读取 + Mahony 姿态解算                                       */
/* -------------------------------------------------------------------------- */
static void LOOP_1000HZ(void) {
  imu_status = ICM20602_Init_NonBlocking();

  if (imu_status == IMU_STATE_READY) {
    ICM20602_ReadData(&imu_raw);
    ICM20602_UpdateAttitude(&imu_raw, &imu_angles);
  }
}

/* -------------------------------------------------------------------------- */
/* 100 Hz — 遥控/串口模式判断 + Yaw PID + 运动解耦                            */
/* -------------------------------------------------------------------------- */
static void LOOP_100HZ(void) {
  if (imu_status == IMU_STATE_READY) {
    Uart1_ControlCmd_t uart_cmd;
    Uart1_Control_GetLatestCmd(&uart_cmd);

    /* 串口有效命令优先：直接解耦下发到电机 (默认串口模式，LED 灭) */
    if (uart_cmd.valid) {
      float Vy = uart_cmd.vy_mmps;
      float Vx = uart_cmd.vx_mmps;
      float w  = uart_cmd.w_mmps;
      /* 电机物理位置: s1=右前, s2=右后, s3=左后, s4=左前 */
      float s1 = Vy + Vx + w;
      float s2 = Vy - Vx - w;
      float s3 = Vy + Vx - w;
      float s4 = Vy - Vx + w;
      MotorControl_SetAllTargetSpeedMMps(s1, s2, s3, s4);
      HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET); /* 串口模式 LED 灭 */
    } else {
      /* 无串口命令时走遥控逻辑 */
      PS2_Data_TypeDef data;
      PS2_Receiver_GetData(&data);

      if (data.ch6 >= 550 && data.ch6 <= 650) {
        remote_mode_enabled = 1;
      }

      RemoteToMotion_Update(imu_angles.yaw, remote_mode_enabled);

      HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin,
                        remote_mode_enabled ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
  } else {
    /* IMU 未就绪：强制停止电机并熄灭 LED */
    MotorControl_StopAll();
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
  }
}

/* -------------------------------------------------------------------------- */
/* 50 Hz — UART4 调试输出 (VOFA+)                                             */
/* -------------------------------------------------------------------------- */
static void LOOP_50HZ(void) {
  if (imu_status == IMU_STATE_READY) {
    UART4_Debug_SendIMUData(&imu_angles);
  }
}

/* -------------------------------------------------------------------------- */
/* 主调度入口 — 在 while(1) 中持续调用 */
/* -------------------------------------------------------------------------- */
void Loop_Run(void) {
  static uint32_t last_1000hz = 0;
  static uint32_t last_100hz = 0;
  static uint32_t last_50hz = 0;

  uint32_t now = HAL_GetTick();

  if (now - last_1000hz >= 1) {
    last_1000hz = now;
    LOOP_1000HZ();
  }

  if (now - last_100hz >= 10) {
    last_100hz = now;
    LOOP_100HZ();
  }

  if (now - last_50hz >= 20) {
    last_50hz = now;
    LOOP_50HZ();
  }
}
