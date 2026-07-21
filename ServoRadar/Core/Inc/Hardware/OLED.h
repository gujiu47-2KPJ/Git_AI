/**
  ******************************************************************************
  * @file    OLED.h
  * @brief   SSD1306 OLED 驱动头文件 (128x64, I2C 接口)
  *          来源: 工作区 mpu-flash-template 项目优化
  *          优化: 支持任意 I2C 句柄, 增加显存缓冲, 适配 ServoRadar 项目
  ******************************************************************************
  */

#ifndef __OLED_H__
#define __OLED_H__

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* OLED I2C 地址 */
#define OLED_I2C_ADDR       0x78

/* OLED 尺寸 */
#define OLED_WIDTH          128
#define OLED_HEIGHT         64
#define OLED_PAGE_NUM       8

/* 函数声明 */
void        OLED_Init(I2C_HandleTypeDef* hi2c);
void        OLED_Clear(I2C_HandleTypeDef* hi2c);
void        OLED_Refresh(I2C_HandleTypeDef* hi2c);
void        OLED_DrawPoint(I2C_HandleTypeDef* hi2c, uint8_t x, uint8_t y, uint8_t color);
void        OLED_ShowChar(I2C_HandleTypeDef* hi2c, uint8_t x, uint8_t y, uint8_t chr, uint8_t size);
void        OLED_ShowString(I2C_HandleTypeDef* hi2c, uint8_t x, uint8_t y, char *str);
void        OLED_ShowNum(I2C_HandleTypeDef* hi2c, uint8_t x, uint8_t y, uint32_t num, uint8_t len);
void        OLED_ShowFloat(I2C_HandleTypeDef* hi2c, uint8_t x, uint8_t y, float num, uint8_t decimal);
void        OLED_ShowSignedNum(I2C_HandleTypeDef* hi2c, uint8_t x, uint8_t y, int32_t num, uint8_t len);

#endif /* __OLED_H__ */