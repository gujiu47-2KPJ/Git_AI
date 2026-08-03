#include "Hardware/AHT20_SW.h"
#include "Hardware/SoftI2C.h"

uint8_t AHT20_SW_Init(void)
{
    uint8_t st = 0;
    uint8_t cmd_be = 0xBE;
    SW_I2C_Init();
    SW_I2C_Unlock();
    /* 初始化命令 0xBE */
    if (!SW_I2C_WriteRaw(AHT20_SW_ADDR, &cmd_be, 1)) return 0;
    /* 等待校准位 (bit3=1) */
    for (int i = 0; i < 50; i++) {
        if (SW_I2C_ReadRaw(AHT20_SW_ADDR, &st, 1) && (st & 0x08)) return 1;
        HAL_Delay(10);
    }
    return 0;
}

uint8_t AHT20_SW_Read(float* temperature_c, float* humidity)
{
    uint8_t cmd[3] = {0xAC, 0x33, 0x00};
    uint8_t data[6], st = 0;
    uint32_t hum, temp;
    /* 触发测量 */
    if (!SW_I2C_WriteRaw(AHT20_SW_ADDR, cmd, 3)) return 0;
    HAL_Delay(80);
    /* 等待测量完成 (bit7=0) */
    for (int i = 0; i < 100; i++) {
        if (SW_I2C_ReadRaw(AHT20_SW_ADDR, &st, 1)) {
            if (!(st & 0x80)) break;
        }
        HAL_Delay(1);
    }
    /* 读 6 字节数据 */
    if (!SW_I2C_ReadRaw(AHT20_SW_ADDR, data, 6)) return 0;
    hum  = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | ((uint32_t)data[3] >> 4);
    temp = ((uint32_t)(data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];
    if (humidity)     *humidity     = (float)hum * 100.0f / 1048576.0f;
    if (temperature_c)*temperature_c = (float)temp * 200.0f / 1048576.0f - 50.0f;
    return 1;
}
