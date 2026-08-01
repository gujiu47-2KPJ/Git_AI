/**
  ******************************************************************************
  * @file    AHT20.h
  * @brief   AHT20 温湿度传感器驱动头文件 (HAL 硬件 I2C 版)
  *          基于参考工程移植，使用硬件 I2C1 与 OLED 共总线
  *          特性: 温度 (-50~200℃) + 湿度 (0~100%RH)
  ******************************************************************************
  */

#ifndef __AHT20_H__
#define __AHT20_H__

#include "main.h"
#include <stdint.h>

/* AHT20 I2C 从机地址 (7 位) */
#define AHT20_ADDR          0x38

/* AHT20 命令 */
#define AHT20_INIT_CMD      0xBE    /* 初始化 */
#define AHT20_SOFTRESET     0xBA    /* 软复位 */
#define AHT20_TRIGGER       0xAC    /* 触发测量 */

/* 函数声明 */
uint8_t AHT20_Init(I2C_HandleTypeDef* hi2c);                 /* 初始化，返回 1=成功 */
uint8_t AHT20_Read_Data(I2C_HandleTypeDef* hi2c,
                        float* temperature, float* humidity); /* 读取温度(℃)/湿度(%RH)，返回 1=成功 */

#endif /* __AHT20_H__ */
