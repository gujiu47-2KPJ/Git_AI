/**
  ******************************************************************************
  * @file    MQ135.c
  * @brief   MQ-135 空气质量传感器驱动实现
  *          来源: 综合 GitHub/CSDN 成熟方案优化
  *          特性: 支持 ADC 采集、PPM 计算、空气质量等级评估
  ******************************************************************************
  */

#include "Hardware/MQ135.h"

/* 外部 ADC 句柄声明 */
extern ADC_HandleTypeDef hadc1;

/* 静态变量 */
static ADC_HandleTypeDef* s_hadc = NULL;
static float s_rzero = MQ135_RZERO;  /* R0 值，可通过校准更新 */

/**
  * @brief  MQ-135 初始化
  * @param  hadc: ADC 句柄指针
  * @retval 无
  */
void MQ135_Init(ADC_HandleTypeDef* hadc)
{
    s_hadc = hadc;
    s_rzero = MQ135_RZERO;  /* 使用默认 R0 值 */
}

/**
  * @brief  读取 ADC 原始值
  * @retval ADC 值 (0-4095)
  */
uint16_t MQ135_ReadADC(void)
{
    uint16_t adc_value = 0;
    
    /* 启动 ADC 转换 */
    HAL_ADC_Start(s_hadc);
    
    /* 等待转换完成 */
    if (HAL_ADC_PollForConversion(s_hadc, 100) == HAL_OK)
    {
        /* 读取转换结果 */
        adc_value = HAL_ADC_GetValue(s_hadc);
    }
    
    /* 停止 ADC */
    HAL_ADC_Stop(s_hadc);
    
    return adc_value;
}

/**
  * @brief  计算电压值
  * @param  adc_value: ADC 原始值
  * @retval 电压值 (V)
  */
float MQ135_GetVoltage(uint16_t adc_value)
{
    /* STM32F103 参考电压为 3.3V，ADC 分辨率为 12 位 (0-4095) */
    return (float)adc_value * 3.3f / 4095.0f;
}

/**
  * @brief  计算传感器电阻 Rs
  * @param  adc_value: ADC 原始值
  * @retval 传感器电阻 (kOhm)
  * @note   使用分压公式: Rs = (Vc * Rload / Vs) - Rload
  *         其中 Vc = 5V (传感器供电), Vs = 输出电压
  */
float MQ135_CalculateRS(uint16_t adc_value)
{
    float voltage = MQ135_GetVoltage(adc_value);
    
    /* 防止除零 */
    if (voltage <= 0.001f)
    {
        return 999.0f;  /* 返回极大值表示异常 */
    }
    
    /* 计算传感器电阻: Rs = ((Vc / Vs) - 1) * Rload */
    /* Vc = 5.0V (传感器供电电压), Rload = 10kOhm */
    float rs = ((5.0f / voltage) - 1.0f) * MQ135_RLOAD;
    
    return rs;
}

/**
  * @brief  计算 Rs/R0 比值
  * @param  rs: 传感器电阻
  * @param  rzero: R0 值 (清洁空气中的传感器电阻)
  * @retval Rs/R0 比值
  */
float MQ135_CalculateRSRatio(float rs, float rzero)
{
    if (rzero <= 0.001f)
    {
        return 1.0f;  /* 防止除零 */
    }
    return rs / rzero;
}

/**
  * @brief  计算 CO2 浓度 (PPM)
  * @param  rs_ratio: Rs/R0 比值
  * @retval CO2 浓度 (PPM)
  * @note   基于 MQ-135 数据手册 CO2 曲线拟合公式
  *         PPM = a * (RS/R0)^b
  */
float MQ135_CalculateCO2PPM(float rs_ratio)
{
    /* CO2 曲线参数 (基于数据手册拟合) */
    const float a = MQ135_SCONE;
    const float b = -2.862f;  /* 典型指数值 */
    
    /* 防止无效值 */
    if (rs_ratio <= 0.0f)
    {
        return 0.0f;
    }
    
    /* 计算 PPM: PPM = a * (RS/R0)^b */
    float ppm = a * powf(rs_ratio, b);
    
    /* 限制合理范围 */
    if (ppm < 0.0f) ppm = 0.0f;
    if (ppm > 10000.0f) ppm = 10000.0f;  /* MQ-135 最大测量范围 */
    
    return ppm;
}

/**
  * @brief  计算 CO 浓度 (PPM)
  * @param  rs_ratio: Rs/R0 比值
  * @retval CO 浓度 (PPM)
  */
float MQ135_CalculateCOPPM(float rs_ratio)
{
    const float a = MQ135_SCO;
    const float b = -3.109f;
    
    if (rs_ratio <= 0.0f)
    {
        return 0.0f;
    }
    
    float ppm = a * powf(rs_ratio, b);
    
    if (ppm < 0.0f) ppm = 0.0f;
    if (ppm > 1000.0f) ppm = 1000.0f;
    
    return ppm;
}

/**
  * @brief  评估空气质量等级
  * @param  co2_ppm: CO2 浓度 (PPM)
  * @retval 空气质量等级
  * @note   基于 CO2 浓度评估室内空气质量
  */
MQ135_AirQuality_t MQ135_AssessAirQuality(float co2_ppm)
{
    if (co2_ppm < 50.0f)
    {
        return MQ135_AIR_QUALITY_EXCELLENT;
    }
    else if (co2_ppm < 100.0f)
    {
        return MQ135_AIR_QUALITY_GOOD;
    }
    else if (co2_ppm < 150.0f)
    {
        return MQ135_AIR_QUALITY_MODERATE;
    }
    else if (co2_ppm < 200.0f)
    {
        return MQ135_AIR_QUALITY_POOR;
    }
    else if (co2_ppm < 300.0f)
    {
        return MQ135_AIR_QUALITY_BAD;
    }
    else
    {
        return MQ135_AIR_QUALITY_HAZARDOUS;
    }
}

/**
  * @brief  获取完整的传感器数据
  * @param  data: 数据结构指针
  * @retval 0=成功, 1=失败
  */
uint8_t MQ135_GetData(MQ135_Data_t* data)
{
    if (data == NULL || s_hadc == NULL)
    {
        return 1;  /* 参数错误 */
    }
    
    /* 读取 ADC 值 */
    data->adc_value = MQ135_ReadADC();
    
    /* 计算电压 */
    data->voltage = MQ135_GetVoltage(data->adc_value);
    
    /* 计算传感器电阻 */
    data->rs_ratio = MQ135_CalculateRS(data->adc_value);
    
    /* 计算 Rs/R0 比值 */
    data->rs_ratio = MQ135_CalculateRSRatio(data->rs_ratio, s_rzero);
    
    /* 计算 CO2 浓度 */
    data->co2_ppm = MQ135_CalculateCO2PPM(data->rs_ratio);
    
    /* 计算 CO 浓度 */
    data->co_ppm = MQ135_CalculateCOPPM(data->rs_ratio);
    
    /* 评估空气质量 */
    data->air_quality = MQ135_AssessAirQuality(data->co2_ppm);
    
    /* 标记已校准 */
    data->is_calibrated = (s_rzero > 0.0f) ? 1 : 0;
    
    return 0;  /* 成功 */
}

/**
  * @brief  校准 R0 值 (在清洁空气中执行)
  * @retval 无
  * @note   此函数应在清洁空气中调用，以获取准确的 R0 值
  *         建议在通风良好的室外或新鲜空气环境中执行
  */
void MQ135_CalibrateRZero(void)
{
    uint16_t adc_value = MQ135_ReadADC();
    float rs = MQ135_CalculateRS(adc_value);
    
    /* 在清洁空气中，Rs/R0 比值约为 1 (根据数据手册) */
    /* 因此 R0 = Rs */
    s_rzero = rs;
}

/**
  * @brief  获取当前 R0 值
  * @retval R0 值 (kOhm)
  */
float MQ135_GetRZero(void)
{
    return s_rzero;
}

/**
  * @brief  设置 R0 值 (用于加载已保存的校准值)
  * @param  rzero: R0 值 (kOhm)
  * @retval 无
  */
void MQ135_SetRZero(float rzero)
{
    if (rzero > 0.0f)
    {
        s_rzero = rzero;
    }
}