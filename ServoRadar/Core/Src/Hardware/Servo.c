/**
  ******************************************************************************
  * @file    Servo.c
  * @brief   SG90 舵机驱动源文件
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

#include "Hardware/Servo.h"

/* 外部定时器句柄（TIM2 用于 PWM 输出） */
extern TIM_HandleTypeDef htim2;

/* 当前舵机角度（静态变量，仅在文件内可见） */
static uint16_t current_angle = SERVO_DEFAULT_ANGLE;

/**
  * @brief  舵机初始化
  * @note   设置初始角度为 90°（中位），启动 PWM 输出
  * @param  无
  * @retval 无
  * 
  * 【初始化流程】
  *   1. 设置初始角度为 90°
  *   2. 计算对应的 CCR 值
  *   3. 设置 TIM2 Channel1 的比较值
  *   4. 启动 PWM 输出
  */
void Servo_Init(void)
{
    current_angle = SERVO_DEFAULT_ANGLE;  /* 初始角度 90° */
    
    /* 计算 CCR 值：线性映射 0~180° → 50~250 */
    uint16_t ccr = SERVO_CCR_MIN + (uint32_t)current_angle * (SERVO_CCR_MAX - SERVO_CCR_MIN) / SERVO_MAX_ANGLE;
    
    /* 设置 TIM2 Channel1 的比较值 */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr);
    
    /* 启动 PWM 输出 */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

/**
  * @brief  设置舵机角度
  * @param  angle: 目标角度（0~180）
  * @retval 无
  * 
  * 【原理】
  *   将角度线性映射到 CCR 值：
  *   CCR = 50 + angle × (250 - 50) / 180
  *   例：90° → CCR = 50 + 90 × 200 / 180 = 150（对应 1.5ms 脉冲）
  * 
  * 【注意事项】
  *   - 角度超出 0~180 范围时会自动限制
  *   - 设置后舵机会立即开始转动，无需额外操作
  */
void Servo_SetAngle(uint16_t angle)
{
    /* 限制角度范围 */
    if (angle > SERVO_MAX_ANGLE)
        angle = SERVO_MAX_ANGLE;

    current_angle = angle;
    
    /* 计算 CCR 值并设置 */
    uint16_t ccr = SERVO_CCR_MIN + (uint32_t)angle * (SERVO_CCR_MAX - SERVO_CCR_MIN) / SERVO_MAX_ANGLE;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr);
}

/**
  * @brief  获取当前舵机角度
  * @retval 当前角度（0~180）
  */
uint16_t Servo_GetAngle(void)
{
    return current_angle;
}

/**
  * @brief  舵机平滑扫描（从 start_angle 到 end_angle）
  * @param  start_angle: 起始角度
  * @param  end_angle: 结束角度
  * @param  step: 每步角度（建议 1~5）
  * @param  delay_ms: 每步延时（ms）
  * @retval 无
  * 
  * 【原理】
  *   以小步长逐步改变舵机角度，实现平滑转动。
  *   步长越小，转动越平滑，但耗时越长。
  *   延时越短，转动速度越快。
  * 
  * 【使用示例】
  *   // 从 0° 扫描到 180°，每步 5°，每步延时 20ms
  *   Servo_Sweep(0, 180, 5, 20);
  */
void Servo_Sweep(uint16_t start_angle, uint16_t end_angle, uint16_t step, uint16_t delay_ms)
{
    /* 参数合法性检查 */
    if (step == 0) step = 1;
    if (delay_ms == 0) delay_ms = 10;

    if (start_angle <= end_angle)
    {
        /* 正向扫描（从小到大） */
        for (uint16_t angle = start_angle; angle <= end_angle; angle += step)
        {
            Servo_SetAngle(angle);
            HAL_Delay(delay_ms);
        }
    }
    else
    {
        /* 反向扫描（从大到小） */
        for (uint16_t angle = start_angle; angle >= end_angle; angle -= step)
        {
            Servo_SetAngle(angle);
            HAL_Delay(delay_ms);
        }
    }
    current_angle = end_angle;  /* 更新当前角度 */
}