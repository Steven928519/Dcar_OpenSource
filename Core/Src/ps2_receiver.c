/**
  ******************************************************************************
  * @file    ps2_receiver.c
  * @brief   PS2 遥控器信号接收模块实现
  *          通过 PA3 接收 PS2 遥控器数据
  ******************************************************************************
  */
#include "ps2_receiver.h"
#include "gpio.h"

/* 当前接收到的 PS2 数据 */
static PS2_Data_TypeDef ps2_data;

void PS2_Receiver_Init(void)
{
  /* 初始化数据变量 */
  ps2_data.lx        = 128;
  ps2_data.ly        = 128;
  ps2_data.rx        = 128;
  ps2_data.ry        = 128;
  ps2_data.buttons   = 0;
  ps2_data.connected = 0;

  /* PA3 已在 gpio.c 中配置为输入，此处可添加其他初始化 */
  /* 具体接收逻辑后续实现 */
}

void PS2_Receiver_GetData(PS2_Data_TypeDef *data)
{
  if (data == NULL) return;

  /* 具体解析逻辑后续实现，目前仅拷贝当前缓存数据 */
  data->lx        = ps2_data.lx;
  data->ly        = ps2_data.ly;
  data->rx        = ps2_data.rx;
  data->ry        = ps2_data.ry;
  data->buttons   = ps2_data.buttons;
  data->connected = ps2_data.connected;
}
