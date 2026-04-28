/**
 ******************************************************************************
 * @file    usart1_control.h
 * @brief   上位机串口控制接口 (USART1 + DMA+IDLE)
 *
 * 支持三种控制帧:
 *   - 0x67 持续速度控制 (Vx/Vy/w, mm/s)
 *   - 0x64 定点匀速位移 (X/Y/Z cm, 速度 cm/s)
 *   - 0x65 定点自适应位移 (自动加减速, 同0x64格式)
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
  UART1_CMD_VELOCITY,     /* 0x67 持续速度控制 */
  UART1_CMD_DISPLACEMENT, /* 0x64 定点匀速位移 */
  UART1_CMD_ADAPTIVE_DISP /* 0x65 定点自适应位移 (自动加减速) */
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

/**
 * @brief  发送串口应答（在主循环中调用）
 * @note   内部检查应答标志，有应答时才发送
 */
void Uart1_Control_SendAck(void);

#ifdef __cplusplus
}
#endif

#endif /* __USART1_CONTROL_H__ */
