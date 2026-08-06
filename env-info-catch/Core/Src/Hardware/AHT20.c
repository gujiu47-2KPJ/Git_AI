/**
  ******************************************************************************
  * @file    AHT20.c
  * @brief   AHT20 温湿度传感器驱动实现 (HAL 硬件 I2C 版)
  *          协议: 命令 0xBE 初始化 / 0xAC 触发测量 / 0xBA 软复位
  *          测量结果: 湿度 20bit + 温度 20bit (各占 2.5 字节)
  ******************************************************************************
  */

#include "Hardware/AHT20.h"

#define AHT20_I2C_TIMEOUT   100   /* I2C 超时 (ms) */

/**
  * @brief  读取 AHT20 状态寄存器
  * @param  hi2c: I2C 句柄
  * @param  status: 输出状态字节
  * @retval 1=成功 0=失败
  */
static uint8_t AHT20_ReadStatus(I2C_HandleTypeDef* hi2c, uint8_t* status)
{
    return (HAL_I2C_Master_Receive(hi2c, AHT20_ADDR << 1, status, 1, AHT20_I2C_TIMEOUT) == HAL_OK);
}

/**
  * @brief  初始化 AHT20
  * @param  hi2c: I2C 句柄
  * @retval 1=成功 0=失败
  * @note   发送 0xBE+0x08 0x00 初始化，等待校准完成位 (Bit3)；
  *         失败则软复位 0xBA 重试最多 10 次
  */
uint8_t AHT20_Init(I2C_HandleTypeDef* hi2c)
{
    uint8_t cmd[3];
    uint8_t status;
    uint8_t retry;

    HAL_Delay(40);

    /* 初始化命令：0xBE + 0x08 0x00 (标准时序, 三字节直发) */
    cmd[0] = AHT20_INIT_CMD;
    cmd[1] = 0x08;
    cmd[2] = 0x00;
    if (HAL_I2C_Master_Transmit(hi2c, AHT20_ADDR << 1, cmd, 3, AHT20_I2C_TIMEOUT) != HAL_OK)
    {
        return 0;
    }

    HAL_Delay(500);

    /* 等待校准完成（状态寄存器 Bit3=1），否则软复位重试 */
    for (retry = 0; retry < 10; retry++)
    {
        if (AHT20_ReadStatus(hi2c, &status) && (status & 0x08))
        {
            return 1;   /* 校准完成 */
        }

        /* 软复位后重新初始化 */
        {
          uint8_t sr = AHT20_SOFTRESET;
          HAL_I2C_Master_Transmit(hi2c, AHT20_ADDR << 1, &sr, 1, AHT20_I2C_TIMEOUT);
          HAL_Delay(200);
          HAL_I2C_Master_Transmit(hi2c, AHT20_ADDR << 1, cmd, 3, AHT20_I2C_TIMEOUT);
          HAL_Delay(500);
        }
    }

    return 0;
}

/**
  * @brief  触发测量并读取温度/湿度
  * @param  hi2c: I2C 句柄
  * @param  temperature: 输出温度 (℃)
  * @param  humidity: 输出湿度 (%RH)
  * @retval 1=成功 0=失败
  */
uint8_t AHT20_Read_Data(I2C_HandleTypeDef* hi2c, float* temperature, float* humidity)
{
    uint8_t cmd[3];
    uint8_t data[7];
    uint8_t status;
    uint16_t cnt = 0;
    uint32_t hum_raw;
    uint32_t temp_raw;

    /* 触发测量：0xAC + 0x33 0x00 (标准时序, 三字节直发) */
    cmd[0] = AHT20_TRIGGER;
    cmd[1] = 0x33;
    cmd[2] = 0x00;
    if (HAL_I2C_Master_Transmit(hi2c, AHT20_ADDR << 1, cmd, 3, AHT20_I2C_TIMEOUT) != HAL_OK)
    {
        return 0;
    }
    HAL_Delay(80);

    /* 等待忙状态结束（Bit7=0），超时 100ms */
    do
    {
        if (!AHT20_ReadStatus(hi2c, &status))
        {
            return 0;
        }
        HAL_Delay(1);
    } while ((status & 0x80) && (++cnt < 100));

    /* 读取 7 字节数据：Data[0]=状态, Data[1..5]=温湿度, Data[6]=CRC */
    if (HAL_I2C_Master_Receive(hi2c, AHT20_ADDR << 1, data, 7, AHT20_I2C_TIMEOUT) != HAL_OK)
    {
        return 0;
    }

    /* 湿度 20bit: Data[1]<<12 | Data[2]<<4 | Data[3]>>4 */
    hum_raw = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | ((uint32_t)data[3] >> 4);

    /* 温度 20bit: Data[3]<<16 | Data[4]<<8 | Data[5] (低 4 位清零) */
    temp_raw = ((uint32_t)(data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | (uint32_t)data[5];

    /* 换算：RH% = raw/2^20*100，℃ = raw/2^20*200-50 */
    *humidity = (float)hum_raw * 100.0f / 1048576.0f;
    *temperature = (float)temp_raw * 200.0f / 1048576.0f - 50.0f;

    return 1;
}
