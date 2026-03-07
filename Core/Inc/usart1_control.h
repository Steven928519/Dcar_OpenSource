/**
 ******************************************************************************
 * @file    usart1_control.h
 * @brief   上位机串口控制接口 (USART1 + DMA+IDLE)
 *
 * 负责：
 *   - 通过 USART1 DMA+IDLE 接收上位机数据帧
 *   - 解析出统一的车体运动指令 (Vx/Vy/w，单位 mm/s)
 *   - 提供查询最新命令的接口，供主循环/调度层调用
 *
 * 具体协议格式稍后与上位机约定后再补充实现。
 ******************************************************************************
 */
#ifndef __USART1_CONTROL_H__
#define __USART1_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * @brief  串口指令类型
 */
typedef enum {
  UART1_CMD_NONE = 0,
  UART1_CMD_VELOCITY,    /* 0x67 持续速度控制 */
  UART1_CMD_DISPLACEMENT /* 0x64 定点匀速位移 */
} Uart1_CmdType_t;

/**
 * @brief  串口控制命令结构体
 *         由 usart1_control.c 在收到完整一帧数据后填充
 */
typedef struct {
  Uart1_CmdType_t type; /* 指令类型 */

  /* 速度控制参数 (type == UART1_CMD_VELOCITY) */
  float vx_mmps; /* 车体 X 方向线速度 (mm/s)，正=右，负=左 */
  float vy_mmps; /* 车体 Y 方向线速度 (mm/s)，正=前，负=后 */
  float w_mmps;  /* 车体旋转等效线速度 (mm/s)，正=顺时针 */

  /* 位移控制参数 (type == UART1_CMD_DISPLACEMENT) */
  float target_x_mm;       /* 相对 X 位移 (mm) */
  float target_y_mm;       /* 相对 Y 位移 (mm) */
  float target_z_mm;       /* 相对 Z 位移 (mm)，暂存为 0 */
  float target_speed_mmps; /* 移动速度上限 (mm/s) */

  uint8_t valid; /* 当前命令是否有效 (1=有效，0=无效或已过期) */
} Uart1_ControlCmd_t;

/**
 * @brief  标记命令已处理，清除有效位
 */
void Uart1_Control_ClearCmd(void);

/**
 * @brief  USART1 控制模块初始化
 * @note   需在 MX_USART1_UART_Init 之后调用
 *         内部会启动 DMA+IDLE 接收
 */
void Uart1_Control_Init(void);

/**
 * @brief  获取最近一次解析成功的控制命令
 * @param  out: 用户提供的结构体指针，用于接收数据
 */
void Uart1_Control_GetLatestCmd(Uart1_ControlCmd_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __USART1_CONTROL_H__ */
