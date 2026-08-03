/**
  ******************************************************************************
  * @file    BMP280.h
  * @brief   BMP280 气压/温度传感器驱动头文件 (HAL 硬件 I2C 版)
  *          基于参考工程移植，使用硬件 I2C1 与 OLED/AHT20 共总线
  *          特性: 气压 (300~1100 hPa) + 温度 (-40~85℃) + 海拔估算
  ******************************************************************************
  */

#ifndef __BMP280_H__
#define __BMP280_H__

#include "main.h"
#include <stdint.h>

/* BMP280 I2C 从机地址 (7 位): SDO=1(接VCC/悬空) → 0x77, SDO=0(接地) → 0x76 */
#define BMP280_ADDR_DEF     0x77
#define BMP280_ADDR_ALT     0x76

/* 寄存器 */
#define BMP280_CHIPID_REG   0xD0    /* Chip ID, 应为 0x58 */
#define BMP280_CTRLMEAS_REG 0xF4
#define BMP280_CONFIG_REG   0xF5
#define BMP280_DATA_REG     0xF7    /* 压力/温度 原始数据 (6 字节) */

/* 校准数据结构 */
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    int32_t  t_fine;
} BMP280_Calib_t;

/* 函数声明 */
uint8_t  BMP280_Init(I2C_HandleTypeDef* hi2c);   /* 初始化，返回实际地址 (0x76/0x77) 或 0=失败 */
uint8_t  BMP280_GetData(I2C_HandleTypeDef* hi2c,
                        float* pressure_hpa,    /* 气压 (hPa) */
                        float* temperature_c,   /* 温度 (℃) */
                        float* altitude_m);     /* 海拔 (m) */

#endif /* __BMP280_H__ */
