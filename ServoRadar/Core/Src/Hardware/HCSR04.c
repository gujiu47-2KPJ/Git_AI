/**
  ******************************************************************************
  * @file    HCSR04.c
  * @brief   HC-SR04 超声波测距模块驱动源文件
  *          参考: CSDN "STM32 HAL库开发HC-SR04超声波测距模块(终极版)"
  *          原理: Trig 触发 → Echo 高电平宽度 = 声波往返时间
  *          距离(mm) = 时间(us) × 0.343 / 2
  ******************************************************************************
  */

#include "Hardware/HCSR04.h"

extern TIM_HandleTypeDef htim3;

/**
  * @brief  HC-SR04 初始化
  *         确保 TIM3 处于停止状态
  */
void HCSR04_Init(void)
{
    __HAL_TIM_DISABLE(&htim3);
    TIM3->CNT = 0;
}

/**
  * @brief  单次超声波测距
  * @param  TrigPort/TrigPin: Trig 引脚
  * @param  EchoPort/EchoPin: Echo 引脚
  * @retval 距离 (mm), 超时返回 HCSR04_DIST_INVALID
  *
  * 原理:
  *   1. Trig 发 ≥10us 高电平脉冲
  *   2. 模块发 8 个 40kHz 超声波
  *   3. Echo 高电平宽度 = 声波往返时间
  *   4. 距离(mm) = 时间(us) × 0.343 / 2
  */
uint16_t HCSR04_Measure(GPIO_TypeDef* TrigPort, uint16_t TrigPin,
                        GPIO_TypeDef* EchoPort, uint16_t EchoPin)
{
    uint32_t timeout;
    uint32_t echo_us;

    /* 1. 发 Trig 脉冲 (≥10us) */
    HAL_GPIO_WritePin(TrigPort, TrigPin, GPIO_PIN_SET);
    /* 延时约 15us (72MHz 下约 1080 个时钟周期) */
    for (volatile int i = 0; i < 50; i++);
    HAL_GPIO_WritePin(TrigPort, TrigPin, GPIO_PIN_RESET);

    /* 2. 等待 Echo 变高 (超时 100ms) */
    timeout = 100000;
    while (HAL_GPIO_ReadPin(EchoPort, EchoPin) == GPIO_PIN_RESET)
    {
        if (--timeout == 0)
            return HCSR04_DIST_INVALID;
    }

    /* 3. 启动 TIM3 计时 (1 tick = 1us) */
    TIM3->CNT = 0;
    __HAL_TIM_ENABLE(&htim3);

    /* 4. 等待 Echo 变低 (超时 40ms ≈ 最大 7m) */
    timeout = 40000;
    while (HAL_GPIO_ReadPin(EchoPort, EchoPin) == GPIO_PIN_SET)
    {
        if (--timeout == 0)
        {
            __HAL_TIM_DISABLE(&htim3);
            return HCSR04_DIST_INVALID;
        }
    }

    /* 5. 停止计时, 读取计数值 */
    echo_us = TIM3->CNT;
    __HAL_TIM_DISABLE(&htim3);

    /* 6. 计算距离: 距离(mm) = echo_us × 343 / 2000
     *    声速 343m/s = 0.343mm/us, 除以 2 是往返 */
    return (uint16_t)((echo_us * 343) / 2000);
}

/**
  * @brief  多次测量取中值 (去极值)
  * @param  times: 测量次数 (建议 3)
  * @retval 中值距离 (mm), 全部超时无效返回 HCSR04_DIST_INVALID
  */
uint16_t HCSR04_MeasureMedian(GPIO_TypeDef* TrigPort, uint16_t TrigPin,
                              GPIO_TypeDef* EchoPort, uint16_t EchoPin,
                              uint8_t times)
{
    if (times < 1) times = 1;
    if (times > 10) times = 10;

    uint16_t dist[10];
    uint8_t valid_count = 0;

    for (uint8_t i = 0; i < times; i++)
    {
        dist[i] = HCSR04_Measure(TrigPort, TrigPin, EchoPort, EchoPin);
        if (dist[i] != HCSR04_DIST_INVALID)
            valid_count++;

        /* 每次测量间隔 60ms, 防止回波干扰 */
        if (i < times - 1)
            HAL_Delay(60);
    }

    /* 全部无效 */
    if (valid_count == 0)
        return HCSR04_DIST_INVALID;

    /* 只有 1 次有效, 直接返回 */
    if (valid_count == 1)
    {
        for (uint8_t i = 0; i < times; i++)
            if (dist[i] != HCSR04_DIST_INVALID)
                return dist[i];
    }

    /* 去极值: 找最大和最小, 返回中间值之和的平均 */
    uint16_t min_d = 0xFFFF, max_d = 0;
    uint32_t sum = 0;
    uint8_t count = 0;

    for (uint8_t i = 0; i < times; i++)
    {
        if (dist[i] == HCSR04_DIST_INVALID)
            continue;
        if (dist[i] < min_d) min_d = dist[i];
        if (dist[i] > max_d) max_d = dist[i];
        sum += dist[i];
        count++;
    }

    /* 去掉一个最大和一个最小 */
    if (count >= 3)
    {
        sum = sum - min_d - max_d;
        count -= 2;
    }

    return (uint16_t)(sum / count);
}