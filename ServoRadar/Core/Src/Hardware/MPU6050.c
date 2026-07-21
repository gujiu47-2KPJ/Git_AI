/**
  ******************************************************************************
  * @file    MPU6050.c
  * @brief   MPU6050 六轴传感器驱动源文件
  *          来源: mpu-flash-template 项目优化
  *          优化: 支持双 I2C 总线, 增加错误处理, 适配 ServoRadar 项目
  ******************************************************************************
  */

#include "Hardware/MPU6050.h"

/* 外部 I2C 句柄声明 */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

/**
  * @brief  MPU6050 初始化
  * @param  hi2c: I2C 句柄指针 (hi2c1 或 hi2c2)
  * @retval 无
  */
void MPU6050_Init(I2C_HandleTypeDef* hi2c)
{
    uint8_t ret;

    /* 1. 唤醒 MPU6050 (退出睡眠模式) */
    ret = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR_8BIT, MPU6050_PWR_MGMT_1,
                            1, (uint8_t[]){0x00}, 1, 100);
    if (ret != HAL_OK) return;
    HAL_Delay(100);

    /* 2. 配置采样率分频器 (1kHz 采样率) */
    ret = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR_8BIT, MPU6050_SMPLRT_DIV,
                            1, (uint8_t[]){0x07}, 1, 100);
    if (ret != HAL_OK) return;

    /* 3. 配置 DLPF (数字低通滤波器, 带宽 42Hz) */
    ret = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR_8BIT, MPU6050_CONFIG,
                            1, (uint8_t[]){0x03}, 1, 100);
    if (ret != HAL_OK) return;

    /* 4. 配置陀螺仪量程 (±250°/s) */
    ret = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR_8BIT, MPU6050_GYRO_CONFIG,
                            1, (uint8_t[]){0x00}, 1, 100);
    if (ret != HAL_OK) return;

    /* 5. 配置加速度计量程 (±2g) */
    ret = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR_8BIT, MPU6050_ACCEL_CONFIG,
                            1, (uint8_t[]){0x00}, 1, 100);
}

/**
  * @brief  读取 MPU6050 WHO_AM_I 寄存器 (用于验证设备)
  * @param  hi2c: I2C 句柄指针
  * @retval 设备 ID (正常应返回 0x68)
  */
uint8_t MPU6050_ReadID(I2C_HandleTypeDef* hi2c)
{
    uint8_t id = 0;
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR_8BIT, MPU6050_WHO_AM_I,
                     1, &id, 1, 100);
    return id;
}

/**
  * @brief  读取 MPU6050 原始数据 (加速度计 + 陀螺仪)
  * @param  hi2c: I2C 句柄指针
  * @param  ax, ay, az: 加速度计原始值
  * @param  gx, gy, gz: 陀螺仪原始值
  * @retval HAL 状态
  */
HAL_StatusTypeDef MPU6050_ReadRaw(I2C_HandleTypeDef* hi2c,
                                   int16_t* ax, int16_t* ay, int16_t* az,
                                   int16_t* gx, int16_t* gy, int16_t* gz)
{
    uint8_t data[14];
    HAL_StatusTypeDef status;

    /* 一次性读取 14 字节 (0x3B ~ 0x48) */
    status = HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR_8BIT, MPU6050_ACCEL_XOUT_H,
                              1, data, 14, 100);
    if (status != HAL_OK) return status;

    /* 解析数据 (大端序) */
    *ax = (int16_t)((data[0] << 8) | data[1]);
    *ay = (int16_t)((data[2] << 8) | data[3]);
    *az = (int16_t)((data[4] << 8) | data[5]);
    /* data[6~7] 是温度, 跳过 */
    *gx = (int16_t)((data[8] << 8) | data[9]);
    *gy = (int16_t)((data[10] << 8) | data[11]);
    *gz = (int16_t)((data[12] << 8) | data[13]);

    return HAL_OK;
}

/**
  * @brief  计算姿态角 (Roll/Pitch) 和物理量
  * @param  data: MPU6050_Data 结构体指针
  * @param  ax, ay, az: 加速度计原始值
  * @param  gx, gy, gz: 陀螺仪原始值
  * @retval 无
  */
void MPU6050_Calculate(MPU6050_Data *data,
                       int16_t ax, int16_t ay, int16_t az,
                       int16_t gx, int16_t gy, int16_t gz)
{
    /* 1. 转换加速度计为 g 单位 (±2g 量程, 灵敏度 16384 LSB/g) */
    data->ax = ax / 16384.0f;
    data->ay = ay / 16384.0f;
    data->az = az / 16384.0f;

    /* 2. 转换陀螺仪为 °/s (±250°/s 量程, 灵敏度 131 LSB/°/s) */
    data->gx = gx / 131.0f;
    data->gy = gy / 131.0f;
    data->gz = gz / 131.0f;

    /* 3. 计算 Roll 和 Pitch (使用加速度计)
     *    Roll: 绕 X 轴旋转角度
     *    Pitch: 绕 Y 轴旋转角度
     *    注意: 静态时准确, 动态时需配合陀螺仪积分 (互补滤波/卡尔曼滤波) */
    data->pitch = atan2f(data->ax,
                         sqrtf(data->ay * data->ay + data->az * data->az))
                  * 180.0f / (float)M_PI;
    data->roll = atan2f(data->ay, data->az) * 180.0f / (float)M_PI;
    data->yaw = 0.0f;  /* MPU6050 无磁力计, 无法直接获取 Yaw */
}

/**
  * @brief  更新 MPU6050 数据 (读取 + 计算)
  * @param  hi2c: I2C 句柄指针
  * @param  data: MPU6050_Data 结构体指针
  * @retval 无
  */
void MPU6050_Update(I2C_HandleTypeDef* hi2c, MPU6050_Data *data)
{
    int16_t ax, ay, az, gx, gy, gz;

    if (MPU6050_ReadRaw(hi2c, &ax, &ay, &az, &gx, &gy, &gz) == HAL_OK)
    {
        MPU6050_Calculate(data, ax, ay, az, gx, gy, gz);
    }
}