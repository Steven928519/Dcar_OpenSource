#include "uart4_debug.h"
#include "usart.h"
#include <stdio.h>

void UART4_Debug_SendIMUData(ICM20602_Attitude_TypeDef *angle) {
  char buf[64];
  // 采用 CSV 格式: Roll,Pitch,Yaw\n
  int len = snprintf(buf, sizeof(buf), "%.2f,%.2f,%.2f\n", angle->roll,
                     angle->pitch, angle->yaw);
  if (len > 0) {
    HAL_UART_Transmit(&huart4, (uint8_t *)buf, (uint16_t)len, 10);
  }
}
