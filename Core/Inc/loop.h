/**
 ******************************************************************************
 * @file    loop.h
 * @brief   主循环任务分频调度模块
 *
 *  分频结构:
 *    LOOP_1000HZ  (1ms)  — IMU 读取 + Mahony 姿态解算
 *    LOOP_100HZ   (10ms) — Yaw PID + 运动解耦 + 遥控模式判断
 *    LOOP_50HZ    (20ms) — UART4 调试输出 (VOFA+)
 *
 *  电机速度 PID (100Hz) 由 TIM6 硬件中断独立驱动，不在此调度。
 ******************************************************************************
 */
#ifndef __LOOP_H__
#define __LOOP_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  主循环调度入口，在 while(1) 中无限调用
 */
void Loop_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* __LOOP_H__ */
