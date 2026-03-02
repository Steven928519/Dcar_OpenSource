/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#if defined(__GNUC__) && !defined(__CC_ARM)
/* Compatibility macros for GCC (Mac/Linux) to support Keil-style keywords */
#ifndef __weak
#define __weak __attribute__((weak))
#endif
#ifndef __packed
#define __packed __attribute__((packed))
#endif
#endif
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED0_Pin GPIO_PIN_5
#define LED0_GPIO_Port GPIOE
#define CS_ICM_Pin GPIO_PIN_8
#define CS_ICM_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
/* 电机方向控制引脚 - 放在此处 CubeMX 重新生成不会删除 */
#define AIN1_Pin GPIO_PIN_0
#define AIN1_GPIO_Port GPIOD
#define AIN2_Pin GPIO_PIN_1
#define AIN2_GPIO_Port GPIOD
#define BIN1_Pin GPIO_PIN_2
#define BIN1_GPIO_Port GPIOD
#define BIN2_Pin GPIO_PIN_3
#define BIN2_GPIO_Port GPIOD
#define CIN1_Pin GPIO_PIN_4
#define CIN1_GPIO_Port GPIOD
#define CIN2_Pin GPIO_PIN_5
#define CIN2_GPIO_Port GPIOD
#define DIN1_Pin GPIO_PIN_6
#define DIN1_GPIO_Port GPIOD
#define DIN2_Pin GPIO_PIN_7
#define DIN2_GPIO_Port GPIOD
#define PS2_DATA_Pin GPIO_PIN_3
#define PS2_DATA_GPIO_Port GPIOA
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
