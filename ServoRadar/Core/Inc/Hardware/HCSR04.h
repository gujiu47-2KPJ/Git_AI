/**
  ******************************************************************************
  * @file    HCSR04.h
  * @brief   HC-SR04 超声波测距模块驱动头文件
  *          参考：CSDN "STM32 HAL 库开发 HC-SR04 超声波测距模块 (终极版)"
  *          原理：Trig 触发 → Echo 高电平宽度 = 声波往返时间
  *          距离 (mm) = 时间 (us) × 0.343 / 2
  *
  * 【优化说明】
  * 1. 添加了距离有效性校验宏，避免返回异常值
  * 2. 增加了测量状态枚举，方便判断测量结果
  * 3. 优化了超时参数，适应不同测量场景
  ******************************************************************************
  */

#ifndef __HCSR04_H
#define __HCSR04_H

#include "main.h"

/* ==================== HC-SR04 参数定义 ==================== */

/* 测量范围限制 */
#define HCSR04_MAX_DIST_MM      4000    /* 最大测量距离 400cm（模块规格上限） */
#define HCSR04_MIN_DIST_MM      20      /* 最小测量距离 2cm（模块规格下限） */

/* 超时参数 */
#define HCSR04_TIMEOUT_US       30000   /* Echo 超时时间 30ms（约 5m，安全余量） */
#define HCSR04_MEASURE_INTERVAL_MS  60  /* 两次测量最小间隔（防止回波干扰） */

/* 无效距离标记 */
#define HCSR04_DIST_INVALID     0xFFFF  /* 测量失败或超时时的返回值 */

/* ==================== 测量状态枚举 ==================== */
/**
  * @brief  超声波测量状态
  * @note   用于判断测量结果是否有效
  */
typedef enum {
    HCSR04_OK = 0,              /* 测量成功 */
    HCSR04_TIMEOUT_ECHO_HIGH,   /* 等待 Echo 变高超时 */
    HCSR04_TIMEOUT_ECHO_LOW,    /* 等待 Echo 变低超时 */
    HCSR04_DIST_OUT_OF_RANGE    /* 距离超出有效范围 */
} HCSR04_Status_t;

/* ==================== 函数声明 ==================== */

/**
  * @brief  HC-SR04 初始化
  * @note   确保 TIM3 处于停止状态，计数器清零
  * @param  无
  * @retval 无
  */
void HCSR04_Init(void);

/**
  * @brief  单次超声波测距
  * @param  TrigPort: Trig 引脚所在端口（如 GPIOA）
  * @param  TrigPin: Trig 引脚号（如 GPIO_PIN_1）
  * @param  EchoPort: Echo 引脚所在端口（如 GPIOA）
  * @param  EchoPin: Echo 引脚号（如 GPIO_PIN_2）
  * @retval 距离值（单位：mm），失败返回 HCSR04_DIST_INVALID
  * @note   阻塞式测量，会等待 Echo 信号完成
  */
uint16_t HCSR04_Measure(GPIO_TypeDef* TrigPort, uint16_t TrigPin,
                        GPIO_TypeDef* EchoPort, uint16_t EchoPin);

/**
  * @brief  多次测量取中值（去极值滤波）
  * @param  TrigPort: Trig 引脚所在端口
  * @param  TrigPin: Trig 引脚号
  * @param  EchoPort: Echo 引脚所在端口
  * @param  EchoPin: Echo 引脚号
  * @param  times: 测量次数（建议 3~5 次，最多 10 次）
  * @retval 中值距离（单位：mm），全部失败返回 HCSR04_DIST_INVALID
  * @note   算法：去掉一个最大值和一个最小值，剩余值求平均
  *         优点：能有效滤除偶然的异常测量值
  */
uint16_t HCSR04_MeasureMedian(GPIO_TypeDef* TrigPort, uint16_t TrigPin,
                              GPIO_TypeDef* EchoPort, uint16_t EchoPin,
                              uint8_t times);

#endif /* __HCSR04_H */