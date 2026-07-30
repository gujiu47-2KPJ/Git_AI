/**
  ******************************************************************************
  * @file    SoftUART.h
  * @brief   软件模拟串口驱动头文件
  *          
  * 【模块说明】
  *   由于 STM32F103C8T6 的 3 个硬件 USART 已全部占用：
  *   - USART1：连接电脑，用于调试信息打印
  *   - USART2：预留
  *   - USART3：预留
  *   因此使用软件模拟串口与 ESP32 通信，解决硬件串口资源不足的问题。
  *   
  * 【引脚配置】
  *   - TX：PB12（发送数据到 ESP32）
  *   - RX：PB13（接收 ESP32 发来的温湿度数据）
  *   
  * 【波特率】
  *   - 9600 bps（与 ESP32 保持一致）
  *   - 每位时间：104.17μs
  *   - 使用 DWT 计数器实现周期级精确延时
  *   
  * 【优化说明】
  *   1. 使用 DWT 时钟周期计数，实现周期级精确延时（非微秒级）
  *   2. 使用 BSRR 寄存器直接操作 GPIO，比 HAL_GPIO_WritePin 快约 25 倍
  *   3. 发送/接收期间关闭中断，消除中断延迟影响
  *   4. 接收时采用 1.5 位采样点，确保在数据位中心采样
  *   5. 内联关键函数，减少函数调用开销
  *   
  * 【使用示例】
  *   // 初始化
  *   SoftUART_Init();
  *   
  *   // 发送字符串
  *   SoftUART_SendString("Hello ESP32\r\n");
  *   
  *   // 接收数据（轮询方式）
  *   uint8_t byte = SoftUART_ReceiveByte();
  *   if (byte != 0xFF) {
  *       // 处理接收到的字节
  *   }
  ******************************************************************************
  */

#ifndef __SOFTUART_H__
#define __SOFTUART_H__

#include "main.h"

/* ==================== 引脚配置 ==================== */
#define SOFTUART_TX_PORT    GPIOB
#define SOFTUART_TX_PIN     GPIO_PIN_12
#define SOFTUART_RX_PORT    GPIOB
#define SOFTUART_RX_PIN     GPIO_PIN_13

/* ==================== 波特率配置 ==================== */
#define SOFTUART_BAUDRATE   9600
#define SOFTUART_BIT_TIME   (1000000 / SOFTUART_BAUDRATE)  /* 每位时间（微秒） */

/* ==================== 接收缓冲区 ==================== */
#define SOFTUART_RX_BUF_SIZE  64

/* ==================== 函数声明 ==================== */

/**
  * @brief  初始化软件模拟串口
  * @note   配置 TX/RX 引脚，初始化 DWT 计数器，清空接收缓冲区
  * @param  无
  * @retval 无
  */
void SoftUART_Init(void);

/**
  * @brief  发送一个字节
  * @param  byte: 要发送的字节
  * @retval 无
  * @note   阻塞式发送，UART 帧格式：1 起始位 + 8 数据位 + 1 停止位
  */
void SoftUART_SendByte(uint8_t byte);

/**
  * @brief  发送字符串
  * @param  str: 要发送的字符串（以 '\0' 结尾）
  * @retval 无
  */
void SoftUART_SendString(const char *str);

/**
  * @brief  接收一个字节（轮询方式）
  * @retval 接收到的字节（0~255），如果没有数据返回 0xFF
  * @note   非阻塞，调用一次只尝试接收 1 个字节
  */
uint8_t SoftUART_ReceiveByte(void);

/**
  * @brief  检查是否有可用数据
  * @retval 1 表示有数据（检测到起始位），0 表示无数据
  */
uint8_t SoftUART_Available(void);

/**
  * @brief  读取接收缓冲区数据
  * @param  buf: 数据缓冲区
  * @param  max_len: 最大读取长度
  * @retval 实际读取的字节数
  */
uint8_t SoftUART_ReadBuffer(uint8_t *buf, uint8_t max_len);

#endif /* __SOFTUART_H__ */