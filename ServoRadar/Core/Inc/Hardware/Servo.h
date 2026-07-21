/**
  ******************************************************************************
  * @file    Servo.h
  * @brief   SG90 舵机驱动头文件
  *          参考: CSDN 高赞博客 + GitHub 开源项目优化
  *          TIM2 Channel1 PWM 输出, 50Hz (20ms周期)
  ******************************************************************************
  */

#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"

/* 舵机角度范围 */
#define SERVO_MIN_ANGLE       0
#define SERVO_MAX_ANGLE       180
#define SERVO_DEFAULT_ANGLE   90

/* PWM 参数 (TIM2: PSC=719, ARR=1999 → 72MHz/720/2000=50Hz)
 * 脉冲宽度: 0.5ms~2.5ms 对应 0°~180°
 * TIM2 计数频率: 100kHz (1 tick = 10us)
 * CCR 值: 50 (0.5ms) ~ 250 (2.5ms) */
#define SERVO_CCR_MIN         50
#define SERVO_CCR_MAX         250

/* 舵机稳定等待时间 (ms) */
#define SERVO_STABLE_TIME_MS  300

/* 函数声明 */
void        Servo_Init(void);
void        Servo_SetAngle(uint16_t angle);
uint16_t    Servo_GetAngle(void);
void        Servo_Sweep(uint16_t start_angle, uint16_t end_angle, uint16_t step, uint16_t delay_ms);

#endif /* __SERVO_H */