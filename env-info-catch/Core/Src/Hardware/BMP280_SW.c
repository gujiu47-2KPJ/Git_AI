#include "Hardware/BMP280_SW.h"
#include "Hardware/SoftI2C.h"

#define BMP280_SW_ADDR  0x76

uint8_t BMP280_SW_Init(void)
{
    uint8_t id, calib[24];
    SW_I2C_Init();
    SW_I2C_Unlock();
    if (!SW_I2C_ReadBytes(BMP280_SW_ADDR, 0xD0, &id, 1) || id != 0x58) return 0;
    if (!SW_I2C_ReadBytes(BMP280_SW_ADDR, 0x88, calib, 24)) return 0;
    if (!SW_I2C_WriteBytes(BMP280_SW_ADDR, 0xF4, (uint8_t[]){(1<<5)|(1<<2)|3}, 1)) return 0;
    if (!SW_I2C_WriteBytes(BMP280_SW_ADDR, 0xF5, (uint8_t[]){5<<2}, 1)) return 0;
    return 1;
}

uint8_t BMP280_SW_GetData(float* pressure_hpa, float* temperature_c)
{
    uint8_t d[6];
    uint32_t raw_p, raw_t;
    if (!SW_I2C_ReadBytes(BMP280_SW_ADDR, 0xF7, d, 6)) return 0;
    raw_p = ((uint32_t)d[0] << 12) | ((uint32_t)d[1] << 4) | ((uint32_t)d[2] >> 4);
    raw_t = ((uint32_t)d[3] << 12) | ((uint32_t)d[4] << 4) | ((uint32_t)d[5] >> 4);
    if (temperature_c) *temperature_c = (float)raw_t / 2560.0f - 50.0f;
    if (pressure_hpa)  *pressure_hpa  = (float)raw_p / 25600.0f;
    return 1;
}
