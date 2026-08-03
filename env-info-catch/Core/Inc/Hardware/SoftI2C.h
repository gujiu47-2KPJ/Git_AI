#ifndef __SOFTI2C_H
#define __SOFTI2C_H

#include "main.h"

/* 软 I2C 引脚（二合一模块 AHT20+BMP280 共用，接这里） */
#define SW_SCL_PORT  GPIOB
#define SW_SCL_PIN   GPIO_PIN_9
#define SW_SDA_PORT  GPIOB
#define SW_SDA_PIN   GPIO_PIN_8

void    SW_I2C_Init(void);
void    SW_I2C_Unlock(void);
uint8_t SW_I2C_WriteBytes(uint8_t dev_addr, uint8_t reg, const uint8_t* buf, uint8_t len);
uint8_t SW_I2C_ReadBytes (uint8_t dev_addr, uint8_t reg, uint8_t* buf, uint8_t len);
uint8_t SW_I2C_WriteRaw  (uint8_t dev_addr, const uint8_t* buf, uint8_t len);  /* 无寄存器地址（AHT20 用） */
uint8_t SW_I2C_ReadRaw   (uint8_t dev_addr, uint8_t* buf, uint8_t len);

#endif
