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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* ========== 引脚定义 - 请根据实际接线修改 ========== */

/* 扫描头超声波 (HC-SR04 #1) */
#define HCSR04_SCAN_TRIG_PORT    GPIOA
#define HCSR04_SCAN_TRIG_PIN     GPIO_PIN_2
#define HCSR04_SCAN_ECHO_PORT    GPIOA
#define HCSR04_SCAN_ECHO_PIN     GPIO_PIN_1

/* 面包板超声波 (HC-SR04 #2) */
#define HCSR04_BREAD_TRIG_PORT   GPIOA
#define HCSR04_BREAD_TRIG_PIN    GPIO_PIN_3
#define HCSR04_BREAD_ECHO_PORT   GPIOB
#define HCSR04_BREAD_ECHO_PIN    GPIO_PIN_0

/* I2C 设备分配 */
#define MPU6050_SCAN_I2C         &hi2c1    /* 扫描头MPU6050用I2C1 */
#define MPU6050_BREAD_I2C         &hi2c2    /* 面包板MPU6050用I2C2 */
#define OLED_I2C                 &hi2c1    /* OLED屏幕用I2C1 */

/* 调试模式选择 */
#define DEBUG_MODE_SCAN_MPU      1        /* 调试扫描头MPU6050 */
#define DEBUG_MODE_SCAN_HCSR04   2        /* 调试扫描头超声波 */
#define DEBUG_MODE_BREAD_HCSR04  3        /* 调试面包板超声波 */
#define DEBUG_MODE_ALL           4        /* 全部一起调试 */

/* 当前调试模式 - 修改这里切换调试对象 */
#define CURRENT_DEBUG_MODE       DEBUG_MODE_ALL

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
