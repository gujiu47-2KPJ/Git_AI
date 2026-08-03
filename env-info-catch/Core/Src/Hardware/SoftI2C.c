#include "Hardware/SoftI2C.h"

static void SCL_H(void) { HAL_GPIO_WritePin(SW_SCL_PORT, SW_SCL_PIN, GPIO_PIN_SET); }
static void SCL_L(void) { HAL_GPIO_WritePin(SW_SCL_PORT, SW_SCL_PIN, GPIO_PIN_RESET); }
static void SDA_H(void) { HAL_GPIO_WritePin(SW_SDA_PORT, SW_SDA_PIN, GPIO_PIN_SET); }
static void SDA_L(void) { HAL_GPIO_WritePin(SW_SDA_PORT, SW_SDA_PIN, GPIO_PIN_RESET); }
static uint8_t SDA_READ(void) { return (SW_SDA_PORT->IDR & SW_SDA_PIN) ? 1 : 0; }

static void delay_us(void) { volatile uint32_t i = 200; while (i--); }
static void start(void) { SDA_L(); delay_us(); SCL_L(); delay_us(); }
static void stop(void)  { SCL_L(); delay_us(); SDA_L(); delay_us();
                          SCL_H(); delay_us(); SDA_H(); delay_us(); }
static void ack(void)   { SDA_L(); delay_us(); SCL_H(); delay_us(); SCL_L(); delay_us(); }
static void nack(void)  { SDA_H(); delay_us(); SCL_H(); delay_us(); SCL_L(); delay_us(); }

static uint8_t write_byte(uint8_t byte)
{
    uint8_t i, a;
    for (i = 0; i < 8; i++) {
        (byte & 0x80) ? SDA_H() : SDA_L();
        byte <<= 1; delay_us();
        SCL_H(); delay_us(); SCL_L(); delay_us();
    }
    SDA_H(); delay_us(); SCL_H(); delay_us();
    a = SDA_READ(); SCL_L(); delay_us();
    return a;  /* 0=ACK, 1=NACK */
}

static uint8_t read_byte(uint8_t ackflag)
{
    uint8_t i, byte = 0;
    SDA_H();
    for (i = 0; i < 8; i++) {
        byte <<= 1; SCL_H(); delay_us();
        if (SDA_READ()) byte |= 1;
        SCL_L(); delay_us();
    }
    ackflag ? ack() : nack();
    return byte;
}

void SW_I2C_Init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    g.Pin = SW_SCL_PIN | SW_SDA_PIN;
    g.Mode = GPIO_MODE_OUTPUT_OD;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);
    SCL_H(); SDA_H();
}

void SW_I2C_Unlock(void)   /* 总线被拉死时发 9 个时钟脉冲解锁 */
{
    uint8_t i;
    SDA_H();
    for (i = 0; i < 9; i++) { SCL_H(); delay_us(); SCL_L(); delay_us(); }
    stop();
}

uint8_t SW_I2C_WriteBytes(uint8_t addr, uint8_t reg, const uint8_t* buf, uint8_t len)
{
    uint8_t i;
    SW_I2C_Unlock();
    start();
    if (write_byte(addr << 1)) { stop(); return 0; }
    if (write_byte(reg))       { stop(); return 0; }
    for (i = 0; i < len; i++)  if (write_byte(buf[i])) { stop(); return 0; }
    stop();
    return 1;
}

uint8_t SW_I2C_ReadBytes(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len)
{
    uint8_t i;
    SW_I2C_Unlock();
    start();
    if (write_byte(addr << 1))      { stop(); return 0; }
    if (write_byte(reg))            { stop(); return 0; }
    start();                        /* repeated start */
    if (write_byte((addr << 1) | 1)){ stop(); return 0; }
    for (i = 0; i < len; i++) buf[i] = read_byte(i < len - 1);
    stop();
    return 1;
}

uint8_t SW_I2C_WriteRaw(uint8_t addr, const uint8_t* buf, uint8_t len)
{
    uint8_t i;
    SW_I2C_Unlock();
    start();
    if (write_byte(addr << 1)) { stop(); return 0; }
    for (i = 0; i < len; i++)  if (write_byte(buf[i])) { stop(); return 0; }
    stop();
    return 1;
}

uint8_t SW_I2C_ReadRaw(uint8_t addr, uint8_t* buf, uint8_t len)
{
    uint8_t i;
    SW_I2C_Unlock();
    start();
    if (write_byte((addr << 1) | 1)) { stop(); return 0; }
    for (i = 0; i < len; i++) buf[i] = read_byte(i < len - 1);
    stop();
    return 1;
}
