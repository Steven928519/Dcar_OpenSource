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
#include "move_control.h"
#include "odometry.h"
#include "ps2_receiver.h"
#include "remote_to_motion.h"
#include "uart4_debug.h"
#include "usart1_control.h"
#include <stdio.h>

/* -------------------------------------------------------------------------- */
/* 模块内共享状态 */
/* -------------------------------------------------------------------------- */
static ICM20602_Data_TypeDef imu_raw;
static ICM20602_Attitude_TypeDef imu_angles = {0};
static IMU_InitState_t imu_status = IMU_STATE_RESET;
static uint8_t remote_mode_enabled = 0;
static uint32_t total_ticks = 0;

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

    /* 1. 串口命令处理 */
    if (uart_cmd.valid) {
      if (uart_cmd.type == UART1_CMD_VELOCITY) {
        /* 持续速度模式：直接下发给电机 (类似遥控) */
        float Vy = uart_cmd.vy_mmps;
        float Vx = uart_cmd.vx_mmps;
        float w = uart_cmd.w_mmps;
        float s1 = Vy + Vx + w;
        float s2 = Vy - Vx - w;
        float s3 = Vy + Vx - w;
        float s4 = Vy - Vx + w;
        MotorControl_SetAllTargetSpeedMMps(s1, s2, s3, s4);
        HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
        // 速度模式不需要 ClearCmd，因为上位机会持续发送。如果停止发送，valid
        // 会维持直到超时
      } else if (uart_cmd.type == UART1_CMD_DISPLACEMENT) {
        /* 位移模式：触发一次性定点移动任务 */
        MoveControl_SetRelativeTarget(uart_cmd.target_x_mm,
                                      uart_cmd.target_y_mm, imu_angles.yaw,
                                      uart_cmd.target_speed_mmps);
        /* 标记已触发，防止重复触发 */
        Uart1_Control_ClearCmd();
      }
    }
    /* 2. 只有在没有串口命令 且 没有自动移动任务 时，才处理遥控逻辑 */
    else if (MoveControl_GetState() != MOVE_EXECUTING) {
      PS2_Data_TypeDef data;
      PS2_Receiver_GetData(&data);
      if (data.ch6 >= 550 && data.ch6 <= 650) {
        remote_mode_enabled = 1;
      }
      HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin,
                        remote_mode_enabled ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }

    /* 3. 如果正在执行位移任务，调用位置环更新，任务中 LED 常亮 */
    if (MoveControl_GetState() == MOVE_EXECUTING) {
      MoveControl_Update();
      HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
    }

    /* 更新里程计 (100Hz, dt=0.01s) */
    Odometry_Update(imu_angles.yaw, 0.01f);
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

    /* 打印里程计数据，验证方向正负 */
    Odometry_TypeDef odo_data;
    Odometry_GetData(&odo_data);
    printf("ODO: x:%.1f y:%.1f yaw:%.1f vx:%.1f vy:%.1f\n", odo_data.x,
           odo_data.y, odo_data.yaw, odo_data.vx, odo_data.vy);
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
