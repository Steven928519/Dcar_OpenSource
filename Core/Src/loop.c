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
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <usart.h>
#include <string.h>
/* -------------------------------------------------------------------------- */
/* 模块内共享状态 */
/* -------------------------------------------------------------------------- */
static ICM20602_Data_TypeDef imu_raw;
static ICM20602_Attitude_TypeDef imu_angles = {0};
static IMU_InitState_t imu_status = IMU_STATE_RESET;
static uint8_t remote_mode_enabled = 0;
//static uint32_t total_ticks = 0;

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
  Uart1_ControlCmd_t uart_cmd;
  float vx = 0, vy = 0, w = 0;
  uint8_t manual_active = 0;

  if (imu_status == IMU_STATE_READY) {
    Uart1_Control_GetLatestCmd(&uart_cmd);

    /* 1. 串口处理 */
    if (uart_cmd.valid) {
      if (uart_cmd.type == UART1_CMD_VELOCITY) {
        vx = uart_cmd.vx_mmps;
        vy = uart_cmd.vy_mmps;
        w = uart_cmd.w_mmps;
        manual_active = 1;
        HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
      } else if (uart_cmd.type == UART1_CMD_DISPLACEMENT) {
        MoveControl_SetRelativeTarget(uart_cmd.target_x_mm,
                                      uart_cmd.target_y_mm, imu_angles.yaw,
                                      uart_cmd.target_speed_mmps);
        Uart1_Control_ClearCmd();
      }
    }

    /* 2. 遥控处理 (如果没有串口速度指令) */
    if (!manual_active && MoveControl_GetState() != MOVE_EXECUTING) {
      PS2_Data_TypeDef data;
      PS2_Receiver_GetData(&data);

      // 更新遥控开关指示灯
      if (data.ch6 >= 550 && data.ch6 <= 650) {
        remote_mode_enabled = 1;
      }
      HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin,
                        remote_mode_enabled ? GPIO_PIN_RESET : GPIO_PIN_SET);

      if (remote_mode_enabled && data.connected) {
        /* 使用舵机映射逻辑转换为 mm/s */
        float ly_speed = ((float)((int32_t)data.ly - REMOTE_CENTER)) *
                         REMOTE_MAX_SPEED / REMOTE_RANGE;
        float lx_speed = ((float)((int32_t)data.lx - REMOTE_CENTER)) *
                         REMOTE_MAX_SPEED / REMOTE_RANGE;
        float rx_speed = ((float)((int32_t)data.rx - REMOTE_CENTER)) *
                         REMOTE_MAX_SPEED / REMOTE_RANGE;

        /* 应用死区 */
        if (fabsf(ly_speed) > 20.0f)
          vy = ly_speed;
        if (fabsf(lx_speed) > 20.0f)
          vx = lx_speed;
        if (fabsf(rx_speed) > 20.0f)
          w = rx_speed;
      }
    }

    /* 3. 任务执行与锁头下发 */
    if (MoveControl_GetState() == MOVE_EXECUTING) {
      MoveControl_Update();
      HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin,
                        GPIO_PIN_RESET); // 移动中 LED 常亮
    } else {
      /* Motion_HandleManual 内部包含锁头逻辑 */
      Motion_HandleManual(vx, vy, w, imu_angles.yaw);
    }

    /* 更新里程计 */
    Odometry_Update(imu_angles.yaw, 0.01f);
  } else {
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
		int len=0;
		char buf[128];
		len = snprintf(buf, sizeof(buf),
                       "ODO: x:%.1f y:%.1f yaw:%.1f vx:%.1f vy:%.1f\n",
                       odo_data.x, odo_data.y, odo_data.yaw,
                       odo_data.vx, odo_data.vy);
        if (len > 0) {
            HAL_UART_Transmit(&huart4, (uint8_t*)buf, (uint16_t)len, 10);
        }
    //printf("ODO: x:%.1f y:%.1f yaw:%.1f vx:%.1f vy:%.1f\n", odo_data.x,
           //odo_data.y, odo_data.yaw, odo_data.vx, odo_data.vy);
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
