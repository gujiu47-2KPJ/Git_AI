/**
  ******************************************************************************
  * @file    HCSR04.h
  * @brief   HC-SR04 超声波测距模块驱动头文件
  *          参考: CSDN "STM32 HAL库开发HC-SR04超声波测距模块(终极版)"
  *          原理: Trig 触发 → Echo 高电平宽度 = 声波往返时间
  *          距离(mm) = 时间(us) × 0.343 / 2
  ******************************************************************************
  */

#ifndef __HCSR04_H
#define __HCSR04_H

#include "main.h"

/* HC-SR04 参数 */
#define HCSR04_MAX_DIST_MM      4000    /* 最大测量距离 400cm */
#define HCSR04_MIN_DIST_MM      20      /* 最小测量距离 2cm */
#define HCSR04_TIMEOUT_US       30000   /* Echo 超时时间 30ms (约5m) */

/* 无效距离标记 */
#define HCSR04_DIST_INVALID     0xFFFF

/* 函数声明 */
void    HCSR04_Init(void);
uint16_t HCSR04_Measure(GPIO_TypeDef* TrigPort, uint16_t TrigPin,
                        GPIO_TypeDef* EchoPort, uint16_t EchoPin);
uint16_t HCSR04_MeasureMedian(GPIO_TypeDef* TrigPort, uint16_t TrigPin,
                              GPIO_TypeDef* EchoPort, uint16_t EchoPin,
                              uint8_t times);

#endif /* __HCSR04_H */