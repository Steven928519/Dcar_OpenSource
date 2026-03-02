#ifndef __IMU_H
#define __IMU_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/
typedef enum {
  IMU_STATE_RESET = 0,   // 发送复位指令
  IMU_STATE_WAIT_RESET,  // 等待复位完成 (100ms)
  IMU_STATE_WAKEUP,      // 发送唤醒指令
  IMU_STATE_WAIT_WAKEUP, // 等待唤醒稳定 (50ms)
  IMU_STATE_CONFIG,      // 配置量程等参数
  IMU_STATE_READY,       // 已就绪
  IMU_STATE_ERROR        // 出错
} IMU_InitState_t;

typedef struct {
  float roll;
  float pitch;
  float yaw;
} ICM20602_Attitude_TypeDef;

typedef struct {
  int16_t acc_x;
  int16_t acc_y;
  int16_t acc_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t temp;
} ICM20602_Data_TypeDef;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
uint8_t ICM20602_Init(void);
IMU_InitState_t ICM20602_Init_NonBlocking(void);
uint8_t ICM20602_Check(void);
void ICM20602_ReadData(ICM20602_Data_TypeDef *data);
void ICM20602_UpdateAttitude(ICM20602_Data_TypeDef *raw,
                             ICM20602_Attitude_TypeDef *angle);
uint8_t ICM20602_ReadReg(uint8_t reg);
void ICM20602_WriteReg(uint8_t reg, uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_H */
