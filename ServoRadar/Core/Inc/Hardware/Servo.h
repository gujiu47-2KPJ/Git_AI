/**
  ******************************************************************************
  * @file    Servo.h
  * @brief   SG90 舵机驱动头文件
  *          
  * 【模块说明】
  *   SG90 是一款常用的微型舵机，工作电压 4.8~6V，扭矩 1.8kg·cm。
  *   控制方式：PWM 信号，50Hz（20ms 周期），脉冲宽度 0.5~2.5ms 对应 0~180°。
  *   
  * 【本项目应用】
  *   - 搭载扫描头超声波和 MPU6050 模块
  *   - 通过旋转实现雷达扫描功能
  *   - 使用 TIM2 Channel1 输出 PWM（PA0 引脚）
  *   
  * 【PWM 参数】
  *   - TIM2 配置：PSC=719, ARR=1999 → 72MHz/(719+1)/(1999+1) = 50Hz
  *   - 计数频率：100kHz（1 tick = 10μs）
  *   - CCR 值范围：50（0.5ms）~ 250（2.5ms）
  *   
  * 【注意事项】
  *   - 舵机供电需充足（建议外部 5V 电源）
  *   - 频繁转动可能引起电流波动，建议设置合理的扫描间隔
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