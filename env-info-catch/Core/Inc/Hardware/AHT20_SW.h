#ifndef __AHT20_SW_H
#define __AHT20_SW_H

#include "main.h"

#define AHT20_SW_ADDR  0x38

uint8_t AHT20_SW_Init(void);
uint8_t AHT20_SW_Read(float* temperature_c, float* humidity);

#endif
