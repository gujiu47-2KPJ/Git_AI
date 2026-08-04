/**
  ******************************************************************************
  * @file    MQ135.c
  * @brief   MQ-135 空气质量传感器驱动实现
  *          来源: 综合 GitHub/CSDN 成熟方案优化
  *          特性: 支持 ADC 采集、PPM 计算、空气质量等级评估
  ******************************************************************************
  */

#include "Hardware/MQ135.h"
#include <stdio.h>
#include <string.h>
extern UART_HandleTypeDef huart1;

/* 外部 ADC 句柄声明 */
extern ADC_HandleTypeDef hadc1;

/* 静态变量 */
static ADC_HandleTypeDef* s_hadc = NULL;
static float s_rzero = MQ135_RZERO;  /* R0 值，可通过校准更新 */
static float s_temperature = 25.0f;  /* 环境温度 (℃)，用于温湿度补偿 */
static float s_humidity = 50.0f;     /* 环境湿度 (%RH)，用于温湿度补偿 */
static float s_pressure = 1013.25f;  /* 环境大气压 (hPa)，用于气压补偿 */

/* 滑动平均缓冲 (最近 5 次，抑制读数波动) */
#define MQ135_AVG_COUNT     5
static float s_ppm_history[MQ135_AVG_COUNT];
static float s_co_history[MQ135_AVG_COUNT];
static float s_alcohol_history[MQ135_AVG_COUNT];
static float s_toluene_history[MQ135_AVG_COUNT];
static float s_nh4_history[MQ135_AVG_COUNT];
static float s_acetone_history[MQ135_AVG_COUNT];
static uint8_t s_history_index = 0;
static uint8_t s_history_count = 0;

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
  * @brief  读取 ADC 原始值 (8 次采样取平均，抑制噪声)
  * @retval ADC 值 (0-4095)
  */
uint16_t MQ135_ReadADC(void)
{
    uint32_t sum = 0;
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        /* 启动 ADC 转换 */
        HAL_ADC_Start(s_hadc);
        
        /* 等待转换完成 */
        if (HAL_ADC_PollForConversion(s_hadc, 100) == HAL_OK)
        {
            sum += HAL_ADC_GetValue(s_hadc);
        }
        
        /* 停止 ADC */
        HAL_ADC_Stop(s_hadc);
    }

    return (uint16_t)(sum / 8);
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
  *         PPM = a * (Rs/R0)^b  (a=曲线系数, b=指数)
  */
float MQ135_CalculateCO2PPM(float rs_ratio)
{
    /* CO2 曲线参数 (基于数据手册拟合) */
    const float a = MQ135_SCO2;
    const float b = MQ135_BCO2;
    
    /* 防止无效值 */
    if (rs_ratio <= 0.0f)
    {
        return 0.0f;
    }
    
    /* 正确公式: PPM = a * (Rs/R0)^b */
    float ppm = a * powf(rs_ratio, b);
    
    /* 添加偏移补偿 (清洁空气中 CO2 约 400 PPM) */
    ppm += MQ135_OFFSET_CO2;
    
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
    const float b = MQ135_BCO;
    
    if (rs_ratio <= 0.0f)
    {
        return 0.0f;
    }
    
    /* 正确公式: PPM = a * (Rs/R0)^b */
    float ppm = a * powf(rs_ratio, b);
    ppm += MQ135_OFFSET_CO;
    
    if (ppm < 0.0f) ppm = 0.0f;
    if (ppm > 1000.0f) ppm = 1000.0f;
    
    return ppm;
}

/**
  * @brief  计算酒精浓度 (PPM)
  * @param  rs_ratio: Rs/R0 比值
  * @retval 酒精浓度 (PPM)
  */
float MQ135_CalculateAlcoholPPM(float rs_ratio)
{
    const float a = MQ135_SALCOHOL;
    const float b = MQ135_BALCOHOL;
    
    if (rs_ratio <= 0.0f)
    {
        return 0.0f;
    }
    
    float ppm = a * powf(rs_ratio, b);
    ppm += MQ135_OFFSET_ALC;
    
    if (ppm < 0.0f) ppm = 0.0f;
    if (ppm > 500.0f) ppm = 500.0f;
    
    return ppm;
}

/**
  * @brief  计算甲苯浓度 (PPM)
  * @param  rs_ratio: Rs/R0 比值
  * @retval 甲苯浓度 (PPM)
  */
float MQ135_CalculateToluenePPM(float rs_ratio)
{
    const float a = MQ135_STOL;
    const float b = MQ135_BTOL;
    
    if (rs_ratio <= 0.0f)
    {
        return 0.0f;
    }
    
    float ppm = a * powf(rs_ratio, b);
    ppm += MQ135_OFFSET_TOL;
    
    if (ppm < 0.0f) ppm = 0.0f;
    if (ppm > 500.0f) ppm = 500.0f;
    
    return ppm;
}

/**
  * @brief  计算氨气浓度 (PPM)
  * @param  rs_ratio: Rs/R0 比值
  * @retval 氨气浓度 (PPM)
  */
float MQ135_CalculateNH4PPM(float rs_ratio)
{
    const float a = MQ135_SNH4;
    const float b = MQ135_BNH4;
    
    if (rs_ratio <= 0.0f)
    {
        return 0.0f;
    }
    
    float ppm = a * powf(rs_ratio, b);
    ppm += MQ135_OFFSET_NH4;
    
    if (ppm < 0.0f) ppm = 0.0f;
    if (ppm > 500.0f) ppm = 500.0f;
    
    return ppm;
}

/**
  * @brief  计算丙酮浓度 (PPM)
  * @param  rs_ratio: Rs/R0 比值
  * @retval 丙酮浓度 (PPM)
  */
float MQ135_CalculateAcetonePPM(float rs_ratio)
{
    const float a = MQ135_SACETONE;
    const float b = MQ135_BACETONE;
    
    if (rs_ratio <= 0.0f)
    {
        return 0.0f;
    }
    
    float ppm = a * powf(rs_ratio, b);
    ppm += MQ135_OFFSET_ACE;
    
    if (ppm < 0.0f) ppm = 0.0f;
    if (ppm > 500.0f) ppm = 500.0f;
    
    return ppm;
}

/**
  * @brief  评估空气质量等级
  * @param  co2_ppm: CO2 浓度 (PPM)
  * @retval 空气质量等级
  * @note   基于真实 CO2 浓度评估室内空气质量 (ppm)
  *         <600 优秀, 600-1000 良好, 1000-1500 一般,
  *         1500-2000 较差, 2000-3000 差, >3000 危险
  */
MQ135_AirQuality_t MQ135_AssessAirQuality(float co2_ppm)
{
    if (co2_ppm < 600.0f)
    {
        return MQ135_AIR_QUALITY_EXCELLENT;
    }
    else if (co2_ppm < 1000.0f)
    {
        return MQ135_AIR_QUALITY_GOOD;
    }
    else if (co2_ppm < 1500.0f)
    {
        return MQ135_AIR_QUALITY_MODERATE;
    }
    else if (co2_ppm < 2000.0f)
    {
        return MQ135_AIR_QUALITY_POOR;
    }
    else if (co2_ppm < 3000.0f)
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
    uint8_t i;
    float ppm_sum = 0.0f;
    float co_sum = 0.0f;
    float alcohol_sum = 0.0f;
    float toluene_sum = 0.0f;
    float nh4_sum = 0.0f;
    float acetone_sum = 0.0f;

    if (data == NULL || s_hadc == NULL)
    {
        return 1;  /* 参数错误 */
    }
    
    /* 读取 ADC 值 */
    data->adc_value = MQ135_ReadADC();
    
    /* 计算电压 */
    data->voltage = MQ135_GetVoltage(data->adc_value);
    
    /* 计算传感器电阻 */
    data->rs = MQ135_CalculateRS(data->adc_value);
    
    /* 温湿度 + 大气压补偿 (综合 Arduino MQ135 库 + Bosch 气压补偿标准):
       Rs_corrected = Rs / (1 + 0.00035*(T-20) + 0.0008*(RH-33)) * (P/1013.25)^0.1
       
       补偿系数说明:
       - 温度系数: 0.00035/°C (温度升高，灵敏度下降)
       - 湿度系数: 0.0008/%RH (湿度升高，读数偏高)
       - 气压系数: 0.1 次方 (气压降低，氧气稀薄，灵敏度下降)
       
       参考: https://github.com/GeorgK/MQ135, Bosch BMP280 应用笔记 */
    {
        /* 温湿度补偿因子 */
        /* 权威算法 (GeorgK/MQ135): corr = 0.00035*T^2 - 0.02718*T + 1.39538 - (RH-33)*0.0018 */
        float temp_hum_corr = 0.00035f * s_temperature * s_temperature
                            - 0.02718f * s_temperature
                            + 1.39538f
                            - (s_humidity - 33.0f) * 0.0018f;
        if (temp_hum_corr < 0.5f || temp_hum_corr > 1.5f)
        {
            temp_hum_corr = 1.0f;
        }
        
        /* 大气压补偿因子 (标准气压 1013.25 hPa) */
        float pressure_corr = powf(s_pressure / 1013.25f, 0.1f);
        if (pressure_corr <= 0.01f || pressure_corr > 2.0f)
        {
            pressure_corr = 1.0f;
        }
        
        /* 综合补偿 */
        data->rs = data->rs / temp_hum_corr;
    }
    
    /* 计算 Rs/R0 比值 */
    data->rs_ratio = MQ135_CalculateRSRatio(data->rs, s_rzero);
    
        char dbg[90];
    /* 计算所有气体浓度 */
    data->co2_ppm = MQ135_CalculateCO2PPM(data->rs_ratio);
    {
        sprintf(dbg, "[MQ] adc=%u v=%.2f rs=%.1f r0=%.1f ratio=%.1f",
                data->adc_value, data->voltage, data->rs, s_rzero, data->rs_ratio);
        HAL_UART_Transmit(&huart1, (uint8_t*)dbg, strlen(dbg), 100);
    }
    data->co_ppm = MQ135_CalculateCOPPM(data->rs_ratio);
    data->alcohol_ppm = MQ135_CalculateAlcoholPPM(data->rs_ratio);
    data->toluene_ppm = MQ135_CalculateToluenePPM(data->rs_ratio);
    data->nh4_ppm = MQ135_CalculateNH4PPM(data->rs_ratio);
    data->acetone_ppm = MQ135_CalculateAcetonePPM(data->rs_ratio);
    
    /* 滑动平均，抑制读数波动 */
    s_ppm_history[s_history_index] = data->co2_ppm;
    s_co_history[s_history_index] = data->co_ppm;
    s_alcohol_history[s_history_index] = data->alcohol_ppm;
    s_toluene_history[s_history_index] = data->toluene_ppm;
    s_nh4_history[s_history_index] = data->nh4_ppm;
    s_acetone_history[s_history_index] = data->acetone_ppm;
    s_history_index = (s_history_index + 1) % MQ135_AVG_COUNT;
    if (s_history_count < MQ135_AVG_COUNT)
    {
        s_history_count++;
    }
    
    for (i = 0; i < s_history_count; i++)
    {
        ppm_sum += s_ppm_history[i];
        co_sum += s_co_history[i];
        alcohol_sum += s_alcohol_history[i];
        toluene_sum += s_toluene_history[i];
        nh4_sum += s_nh4_history[i];
        acetone_sum += s_acetone_history[i];
    }
    data->co2_ppm = ppm_sum / (float)s_history_count;
    data->co_ppm = co_sum / (float)s_history_count;
    data->alcohol_ppm = alcohol_sum / (float)s_history_count;
    data->toluene_ppm = toluene_sum / (float)s_history_count;
    data->nh4_ppm = nh4_sum / (float)s_history_count;
    data->acetone_ppm = acetone_sum / (float)s_history_count;
    
    /* 评估空气质量 (基于平均后的 CO2 浓度) */
    data->air_quality = MQ135_AssessAirQuality(data->co2_ppm);
    
    /* 标记已校准 */
    data->is_calibrated = (s_rzero > 0.0f) ? 1 : 0;
    
    return 0;  /* 成功 */
}

/**
  * @brief  设置环境参数 (温湿度 + 大气压)
  * @param  temperature: 温度 (℃)
  * @param  humidity: 湿度 (%RH)
  * @param  pressure: 大气压 (hPa)
  * @retval 无
  */
void MQ135_SetEnvironment(float temperature, float humidity, float pressure)
{
    s_temperature = temperature;
    s_humidity = humidity;
    s_pressure = pressure;
}

/**
  * @brief  校准 R0 值 (在清洁空气中执行)
  * @retval 无
  * @note   此函数应在清洁空气中调用，以获取准确的 R0 值
  *         建议在通风良好的室外或新鲜空气环境中执行
  *         连续采样 10 次取平均，提高校准精度
  */
void MQ135_CalibrateRZero(void)
{
    float rs_sum = 0.0f;
    uint8_t i;

    for (i = 0; i < 10; i++)
    {
        float rs_raw = MQ135_CalculateRS(MQ135_ReadADC());
        /* 与测量一致：校准也用温湿度补偿后的 Rs */
        float corr = 0.00035f * s_temperature * s_temperature
                   - 0.02718f * s_temperature
                   + 1.39538f
                   - (s_humidity - 33.0f) * 0.0018f;
        if (corr < 0.5f || corr > 1.5f) corr = 1.0f;
        rs_sum += rs_raw / corr;
        HAL_Delay(100);
    }

    /* 清洁空气中 Rs/R0 ≈ 1，因此 R0 = 补偿后 Rs */
    s_rzero = rs_sum / 10.0f;
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