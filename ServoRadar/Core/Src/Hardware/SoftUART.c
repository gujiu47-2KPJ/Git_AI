#include "Hardware/SoftUART.h"
#include <string.h>

#define DWT_CR      *(volatile uint32_t*)0xE0001000
#define DWT_CYCCNT  *(volatile uint32_t*)0xE0001004
#define DEM_CR      *(volatile uint32_t*)0xE000EDFC
#define DEM_CR_TRCENA       (1 << 24)
#define DWT_CR_CYCCNTENA    (1 << 0)

#define SYS_CLK_FREQ        72000000
#define CYCLES_PER_BIT      (SYS_CLK_FREQ / SOFTUART_BAUDRATE)

static volatile uint8_t rx_buffer[SOFTUART_RX_BUF_SIZE];
static volatile uint8_t rx_head = 0;
static volatile uint8_t rx_tail = 0;

static void DWT_Init(void)
{
    DEM_CR |= DEM_CR_TRCENA;
    DWT_CR |= DWT_CR_CYCCNTENA;
    DWT_CYCCNT = 0;
}

static inline void Delay_Cycles(uint32_t cycles)
{
    uint32_t start = DWT_CYCCNT;
    while ((DWT_CYCCNT - start) < cycles);
}

static inline void TX_HIGH(void) { GPIOB->BSRR = SOFTUART_TX_PIN; }
static inline void TX_LOW(void)  { GPIOB->BSRR = (uint32_t)SOFTUART_TX_PIN << 16; }
static inline uint8_t RX_READ(void) { return (SOFTUART_RX_PORT->IDR & SOFTUART_RX_PIN) ? 1 : 0; }

void SoftUART_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    DWT_Init();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = SOFTUART_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SOFTUART_TX_PORT, &GPIO_InitStruct);
    TX_HIGH();
    GPIO_InitStruct.Pin = SOFTUART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(SOFTUART_RX_PORT, &GPIO_InitStruct);
    memset((void*)rx_buffer, 0, sizeof(rx_buffer));
    rx_head = 0; rx_tail = 0;
}

void SoftUART_SendByte(uint8_t byte)
{
    uint8_t i;
    __disable_irq();
    TX_LOW(); Delay_Cycles(CYCLES_PER_BIT - 2);
    for (i = 0; i < 8; i++) {
        (byte & (1 << i)) ? TX_HIGH() : TX_LOW();
        Delay_Cycles(CYCLES_PER_BIT - 2);
    }
    TX_HIGH(); Delay_Cycles(CYCLES_PER_BIT - 2);
    __enable_irq();
}

void SoftUART_SendString(const char *str) { while (*str) SoftUART_SendByte(*str++); }

/**
  * @brief  接收一个字节（非阻塞轮询方式）
  * @retval 接收到的字节(0~255)，如果当前没有起始位返回 0xFF
  * @note   必须在起始位刚开始时调用才能正确接收
  *         如果RX引脚为高（空闲态），立即返回0xFF，绝不阻塞
  *         如果检测到起始位（低电平），关中断接收约1ms后返回
  */
uint8_t SoftUART_ReceiveByte(void)
{
    uint8_t byte = 0, i;
    
    /* 【非阻塞】先检查RX引脚：如果是高电平（空闲），直接返回，不等待 */
    if (GPIOB->IDR & SOFTUART_RX_PIN) {
        return 0xFF;
    }
    
    /* 检测到低电平：可能是起始位，关中断开始精确时序接收 */
    __disable_irq();
    
    /* 再确认一次，防止是毛刺干扰 */
    if (GPIOB->IDR & SOFTUART_RX_PIN) {
        __enable_irq();
        return 0xFF;
    }
    
    /* 延时1.5位时间：跳过起始位，直接到第一个数据位的中心采样点 */
    Delay_Cycles(CYCLES_PER_BIT + CYCLES_PER_BIT / 2);
    
    /* 依次读取8个数据位（低位在前） */
    for (i = 0; i < 8; i++) {
        byte >>= 1;
        if (GPIOB->IDR & SOFTUART_RX_PIN) {
            byte |= 0x80;
        }
        Delay_Cycles(CYCLES_PER_BIT);
    }
    
    /* 跳过停止位（不需要采样，直接开中断返回） */
    __enable_irq();
    return byte;
}

uint8_t SoftUART_Available(void) { return (RX_READ() == 0) ? 1 : 0; }
uint8_t SoftUART_ReadBuffer(uint8_t *buf, uint8_t max_len)
{
    uint8_t count = 0;
    while (count < max_len && SoftUART_Available()) { uint8_t b = SoftUART_ReceiveByte(); if (b != 0xFF) buf[count++] = b; }
    return count;
}