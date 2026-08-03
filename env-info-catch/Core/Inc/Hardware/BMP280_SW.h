#ifndef __BMP280_SW_H
#define __BMP280_SW_H

#include "main.h"

/* 软件 I2C 引脚（BMP280 换线：SDA→PB8, SCL→PB9） */
#define SW_SCL_PORT  GPIOB
#define SW_SCL_PIN   GPIO_PIN_9
#define SW_SDA_PORT  GPIOB
#define SW_SDA_PIN   GPIO_PIN_8

#define BMP280_SW_ADDR  0x76   /* 默认地址，模块焊接 0x77 时改这里 */

uint8_t BMP280_SW_Init(void);
uint8_t BMP280_SW_GetData(float* pressure_hpa, float* temperature_c);

#endif
