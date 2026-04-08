#ifndef __UART4_DEBUG_H__
#define __UART4_DEBUG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "imu.h"
#include "main.h"

/**
 * @brief  通过 UART4 发送 IMU 数据字符串
 * @param  data: IMU 数据结构体
 */
void UART4_Debug_SendIMUData(ICM20602_Attitude_TypeDef *angle);

#ifdef __cplusplus
}
#endif

#endif /* __UART4_DEBUG_H__ */
