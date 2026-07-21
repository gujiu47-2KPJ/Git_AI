/**
  ******************************************************************************
  * @file    Servo.c
  * @brief   SG90 舵机驱动源文件
  *          参考: CSDN 高赞博客 + GitHub 开源项目优化
  *          TIM2 Channel1 PWM 输出, 50Hz (20ms周期)
  ******************************************************************************
  */

#include "Hardware/Servo.h"

extern TIM_HandleTypeDef htim2;

static uint16_t current_angle = SERVO_DEFAULT_ANGLE;

/**
  * @brief  舵机初始化
  *         设置初始角度为 90° (中位), 启动 PWM 输出
  */
void Servo_Init(void)
{
    current_angle = SERVO_DEFAULT_ANGLE;
    uint16_t ccr = SERVO_CCR_MIN + (uint32_t)current_angle * (SERVO_CCR_MAX - SERVO_CCR_MIN) / SERVO_MAX_ANGLE;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

/**
  * @brief  设置舵机角度
  * @param  angle: 目标角度 (0~180)
  */
void Servo_SetAngle(uint16_t angle)
{
    if (angle > SERVO_MAX_ANGLE)
        angle = SERVO_MAX_ANGLE;

    current_angle = angle;
    uint16_t ccr = SERVO_CCR_MIN + (uint32_t)angle * (SERVO_CCR_MAX - SERVO_CCR_MIN) / SERVO_MAX_ANGLE;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr);
}

/**
  * @brief  获取当前舵机角度
  * @retval 当前角度 (0~180)
  */
uint16_t Servo_GetAngle(void)
{
    return current_angle;
}

/**
  * @brief  舵机平滑扫描 (从 start_angle 到 end_angle)
  * @param  start_angle: 起始角度
  * @param  end_angle: 结束角度
  * @param  step: 每步角度 (建议 1~5)
  * @param  delay_ms: 每步延时 (ms)
  */
void Servo_Sweep(uint16_t start_angle, uint16_t end_angle, uint16_t step, uint16_t delay_ms)
{
    if (step == 0) step = 1;
    if (delay_ms == 0) delay_ms = 10;

    if (start_angle <= end_angle)
    {
        for (uint16_t angle = start_angle; angle <= end_angle; angle += step)
        {
            Servo_SetAngle(angle);
            HAL_Delay(delay_ms);
        }
    }
    else
    {
        for (uint16_t angle = start_angle; angle >= end_angle; angle -= step)
        {
            Servo_SetAngle(angle);
            HAL_Delay(delay_ms);
        }
    }
    current_angle = end_angle;
}