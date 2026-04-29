/* Includes ------------------------------------------------------------------*/
#include "imu.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
#include "main.h"
#include "spi.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define ICM20602_WHO_AM_I_REG 0x75
#define ICM20602_WHO_AM_I_VAL 0x12 /* ICM20602 的默认 ID */

/* Private macro -------------------------------------------------------------*/
#define ICM_CS_LOW()                                                           \
  HAL_GPIO_WritePin(CS_ICM_GPIO_Port, CS_ICM_Pin, GPIO_PIN_RESET)
#define ICM_CS_HIGH()                                                          \
  HAL_GPIO_WritePin(CS_ICM_GPIO_Port, CS_ICM_Pin, GPIO_PIN_SET)

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static uint8_t SPI2_ReadWriteByte(uint8_t byte);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief  向 ICM20602 寄存器写入一个字节
 * @param  reg: 寄存器地址
 * @param  data: 要写入的数据
 */
void ICM20602_WriteReg(uint8_t reg, uint8_t data) {
  ICM_CS_LOW();
  SPI2_ReadWriteByte(reg & 0x7F); /* 最高位置0表示写入 */
  SPI2_ReadWriteByte(data);
  ICM_CS_HIGH();
}

/**
 * @brief  向 ICM20602 寄存器读取一个字节
 * @param  reg: 寄存器地址
 * @retval 读取到的数据
 */
uint8_t ICM20602_ReadReg(uint8_t reg) {
  uint8_t res;
  ICM_CS_LOW();
  SPI2_ReadWriteByte(reg | 0x80); /* 最高位置1表示读取 */
  res = SPI2_ReadWriteByte(0xFF); /* 发送 dummy byte 获取数据 */
  ICM_CS_HIGH();
  return res;
}

/**
 * @brief  SPI2 读写一个字节 (基础阻塞方式)
 */
static uint8_t SPI2_ReadWriteByte(uint8_t byte) {
  uint8_t res;
  HAL_SPI_TransmitReceive(&hspi2, &byte, &res, 1, 1);
  return res;
}

/**
 * @brief  检查 ICM20602 是否正常连接
 * @retval 1: 成功, 0: 失败
 */
uint8_t ICM20602_Check(void) {
  uint8_t id = ICM20602_ReadReg(ICM20602_WHO_AM_I_REG);
  if (id == ICM20602_WHO_AM_I_VAL) {
    return 1;
  }
  return 0;
}

static IMU_InitState_t current_state = IMU_STATE_RESET;
static uint32_t state_tick = 0;

/**
 * @brief  非阻塞式初始化状态机
 * @retval 返回当前状态
 */
IMU_InitState_t ICM20602_Init_NonBlocking(void) {
  uint32_t now = HAL_GetTick();

  switch (current_state) {
  case IMU_STATE_RESET:
    ICM20602_WriteReg(0x6B, 0x80); // 发送复位
    state_tick = now;
    current_state = IMU_STATE_WAIT_RESET;
    break;

  case IMU_STATE_WAIT_RESET:
    if (now - state_tick >= 100) { // 等待复位完成
      current_state = IMU_STATE_WAKEUP;
    }
    break;

  case IMU_STATE_WAKEUP:
    ICM20602_WriteReg(0x6B, 0x01); // 唤醒，选择自动时钟源
    state_tick = now;
    current_state = IMU_STATE_WAIT_WAKEUP;
    break;

  case IMU_STATE_WAIT_WAKEUP:
    if (now - state_tick >= 50) { // 等待唤醒稳定
      if (ICM20602_Check()) {
        current_state = IMU_STATE_CONFIG;
      } else {
        current_state = IMU_STATE_ERROR;
      }
    }
    break;

  case IMU_STATE_CONFIG:
    // 基础量程配置
    ICM20602_WriteReg(0x1B, 0x18); // 陀螺仪：+-2000dps
    ICM20602_WriteReg(0x1C, 0x18); // 加速度计：+-16g
    ICM20602_WriteReg(0x1D, 0x06); // 加速度计低通滤波
    current_state = IMU_STATE_READY;
    break;

  case IMU_STATE_READY:
    /* 正常运行状态，保持 */
    break;

  case IMU_STATE_ERROR:
    /* 错误状态：等待 500ms 后自动重试复位 */
    if (now - state_tick >= 500) {
      current_state = IMU_STATE_RESET;
      state_tick = now;
    }
    break;
  }
  return current_state;
}

/**
 * @brief  初始化 ICM20602 (传统阻塞方式，保留备用)
 * @retval 1: 成功, 0: 失败
 */
uint8_t ICM20602_Init(void) {
  // 1. 复位设备
  ICM20602_WriteReg(0x6B, 0x80);
  HAL_Delay(100);

  // 2. 解除睡眠，选择自动选择时钟源
  ICM20602_WriteReg(0x6B, 0x01);
  HAL_Delay(50);

  // 3. 检查 ID
  if (!ICM20602_Check()) {
    return 0;
  }

  // 4. 配置陀螺仪/加速度计量程 (基础设置)
  ICM20602_WriteReg(0x1B, 0x18); // 陀螺仪: +-2000dps
  ICM20602_WriteReg(0x1C, 0x18); // 加速度计: +-16g
  ICM20602_WriteReg(0x1D, 0x06); // 加速度计低通滤波

  return 1;
}

#include <math.h>

/* Mahony 互补滤波参数 */
#define Kp 2.0f       /* 比例增益控制加速度计补偿陀螺仪漂移的速度 */
#define Ki 0.005f     /* 积分增益控制漂移补偿的精确度 */
#define halfT 0.0005f /* 采样周期的一半 (假设主循环中 IMU 更新频率为 1kHz) */

/* 四元数初始化 */
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float integral_ex = 0.0f, integral_ey = 0.0f, integral_ez = 0.0f;

/**
 * @brief  Mahony 互补滤波及四元数更新
 */
void ICM20602_UpdateAttitude(ICM20602_Data_TypeDef *raw,
                             ICM20602_Attitude_TypeDef *angle) {
  float ax = raw->acc_x;
  float ay = raw->acc_y;
  float az = raw->acc_z;
  float gx = raw->gyro_x * 0.0010653f; /* 转化为弧度/秒 */
  float gy = raw->gyro_y * 0.0010653f;
  float gz = raw->gyro_z * 0.0010653f;

  float norm;
  float vx, vy, vz;
  float ex, ey, ez;

  /* 1. 加速度归一化 (重力方向) */
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm > 0) {
    ax /= norm;
    ay /= norm;
    az /= norm;

    /* 2. 提取当前四元数得到的重力分量 (在地理坐标系下的投影) */
    vx = 2 * (q1 * q3 - q0 * q2);
    vy = 2 * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    /* 3. 叉乘得到误差 (测量重力 vs 预测重力) */
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    /* 4. 误差积分 */
    integral_ex += ex * Ki;
    integral_ey += ey * Ki;
    integral_ez += ez * Ki;

    /* 5. 使用 Kp 和 Ki 修正角速度 */
    gx = gx + Kp * ex + integral_ex;
    gy = gy + Kp * ey + integral_ey;
    gz = gz + Kp * ez + integral_ez;
  }

  /* 6. 四元数微分方程更新 */
  q0 += (-q1 * gx - q2 * gy - q3 * gz) * halfT;
  q1 += (q0 * gx + q2 * gz - q3 * gy) * halfT;
  q2 += (q0 * gy - q1 * gz + q3 * gx) * halfT;
  q3 += (q0 * gz + q1 * gy - q2 * gx) * halfT;

  /* 7. 四元数归一化 */
  norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 /= norm;
  q1 /= norm;
  q2 /= norm;
  q3 /= norm;

  /* 8. 转换为欧拉角 (角度制) */
  angle->roll =
      atan2(2 * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) *
      57.29578f;
  angle->pitch = -asin(2 * (q1 * q3 - q0 * q2)) * 57.29578f;

  /* 9. Yaw 轴特殊处理: 条件积分 (Deadzone) */
  /* 说明: 这种写法通常用于滤除零飘，认为当转速大于 3 或小于 -3 时才开始积分更新
   * Yaw */
  float gz_deg = raw->gyro_z * 0.061035f; // 转化到 度/秒 (2000/32768)
  if (gz_deg > 3.0f || gz_deg < -3.0f) {
    // 0.001f 为时钟周期 (1ms)，这里假设本函数被 1kHz 周期调用
    angle->yaw -= (gz_deg * 0.001f);
  }
}

/**
 * @brief  读取 IMU 原始数据 (加速度、温度、陀螺仪)
 * @param  data: 存储读到数据的结构体指针
 */
void ICM20602_ReadData(ICM20602_Data_TypeDef *data) {
  uint8_t buf[14];

  ICM_CS_LOW();
  SPI2_ReadWriteByte(0x3B | 0x80); // 从加速度 X 轴高位地址开始读

  // 连续读取 14 个字节 (Burst Read)
  for (int i = 0; i < 14; i++) {
    buf[i] = SPI2_ReadWriteByte(0xFF);
  }
  ICM_CS_HIGH();

  // 数据拼凑 (高位在前)
  data->acc_x = (int16_t)((buf[0] << 8) | buf[1]);
  data->acc_y = (int16_t)((buf[2] << 8) | buf[3]);
  data->acc_z = (int16_t)((buf[4] << 8) | buf[5]);
  data->temp = (int16_t)((buf[6] << 8) | buf[7]);
  data->gyro_x = (int16_t)((buf[8] << 8) | buf[9]);
  data->gyro_y = (int16_t)((buf[10] << 8) | buf[11]);
  data->gyro_z = (int16_t)((buf[12] << 8) | buf[13]);
}
