/**
  ******************************************************************************
  * @file    MPU6050.h
  * @brief   MPU6050 六轴传感器驱动头文件
  *          来源: mpu-flash-template 项目优化
  *          优化: 支持双 I2C 总线 (hi2c1/hi2c2), 适配 ServoRadar 项目
  ******************************************************************************
  */

#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "main.h"
#include <math.h>

/* MPU6050 I2C 地址 (7-bit: 0x68, 8-bit: 0xD0) */
#define MPU6050_ADDR_7BIT     0x68
#define MPU6050_ADDR_8BIT     0xD0

/* MPU6050 寄存器地址 */
#define MPU6050_SMPLRT_DIV    0x19
#define MPU6050_CONFIG        0x1A
#define MPU6050_GYRO_CONFIG   0x1B
#define MPU6050_ACCEL_CONFIG  0x1C
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_WHO_AM_I      0x75
#define MPU6050_ACCEL_XOUT_H  0x3B
#define MPU6050_TEMP_OUT_H    0x41
#define MPU6050_GYRO_XOUT_H   0x43

/* MPU6050 数据结构体 */
typedef struct {
    float ax, ay, az;         /* 加速度计 (g) */
    float gx, gy, gz;         /* 陀螺仪 (°/s) */
    float roll, pitch, yaw;   /* 姿态角 (°) */
    float temperature;        /* 温度 (°C) */
} MPU6050_Data;

/* I2C 总线选择 */
typedef enum {
    MPU6050_I2C1 = 0,         /* 使用 hi2c1 */
    MPU6050_I2C2 = 1          /* 使用 hi2c2 */
} MPU6050_I2C_Bus;

/* 函数声明 */
void        MPU6050_Init(I2C_HandleTypeDef* hi2c);
HAL_StatusTypeDef MPU6050_ReadRaw(I2C_HandleTypeDef* hi2c,
                                   int16_t* ax, int16_t* ay, int16_t* az,
                                   int16_t* gx, int16_t* gy, int16_t* gz);
void        MPU6050_Calculate(MPU6050_Data *data,
                              int16_t ax, int16_t ay, int16_t az,
                              int16_t gx, int16_t gy, int16_t gz);
void        MPU6050_Update(I2C_HandleTypeDef* hi2c, MPU6050_Data *data);
uint8_t     MPU6050_ReadID(I2C_HandleTypeDef* hi2c);

#endif /* __MPU6050_H__ */