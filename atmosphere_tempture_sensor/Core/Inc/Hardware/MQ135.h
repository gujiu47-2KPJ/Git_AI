/**
  ******************************************************************************
  * @file    MQ135.h
  * @brief   MQ-135 空气质量传感器驱动头文件
  *          来源: 综合 GitHub/CSDN 成熟方案优化
  *          特性: 支持 ADC 采集、PPM 计算、空气质量等级评估
  ******************************************************************************
  */

#ifndef __MQ135_H__
#define __MQ135_H__

#include "main.h"
#include <math.h>
#include <stdint.h>

/* MQ-135 ADC 采集引脚 (使用 ADC1 Channel 0, PA0) */
#define MQ135_ADC_CHANNEL   ADC_CHANNEL_0

/* MQ-135 传感器参数 */
#define MQ135_RLOAD         4.4f        /* 负载电阻 (kOhm), 实测板子 A0-GND ≈ 4.3-4.5kΩ */
#define MQ135_RZERO         76.63f      /* 清洁空气中传感器电阻 (kOhm), 需校准 */
#define MQ135_SCONE         400.0f      /* CO2 曲线在 Rs/R0=1 处的浓度 (清洁空气约 400ppm, 数据手册值) */
#define MQ135_SCO           605.1822459f /* CO 曲线斜率因子 */
#define MQ135_SALCOHOL      77.255f     /* 酒精曲线斜率因子 */
#define MQ135_STOL          10.0f       /* 甲苯曲线斜率因子 */
#define MQ135_SNH4          102.2f      /* 氨气曲线斜率因子 */
#define MQ135_SACETONE      34.668f     /* 丙酮曲线斜率因子 */

/* 空气质量等级定义 (基于 CO2 ppm) */
typedef enum {
    MQ135_AIR_QUALITY_EXCELLENT = 0,  /* 优秀: <600 ppm */
    MQ135_AIR_QUALITY_GOOD,          /* 良好: 600-1000 ppm */
    MQ135_AIR_QUALITY_MODERATE,      /* 一般: 1000-1500 ppm */
    MQ135_AIR_QUALITY_POOR,          /* 较差: 1500-2000 ppm */
    MQ135_AIR_QUALITY_BAD,           /* 差: 2000-3000 ppm */
    MQ135_AIR_QUALITY_HAZARDOUS      /* 危险: >3000 ppm */
} MQ135_AirQuality_t;

/* MQ-135 数据结构 */
typedef struct {
    uint16_t adc_value;              /* ADC 原始值 (0-4095) */
    float voltage;                   /* 电压值 (V) */
    float rs_ratio;                  /* Rs/R0 比值 */
    float co2_ppm;                   /* CO2 浓度 (PPM) */
    float co_ppm;                    /* CO 浓度 (PPM) */
    MQ135_AirQuality_t air_quality;  /* 空气质量等级 */
    uint8_t is_calibrated;           /* 是否已校准 */
} MQ135_Data_t;

/* 函数声明 */
void        MQ135_Init(ADC_HandleTypeDef* hadc);
uint16_t    MQ135_ReadADC(void);
float       MQ135_GetVoltage(uint16_t adc_value);
float       MQ135_CalculateRS(uint16_t adc_value);
float       MQ135_CalculateRSRatio(float rs, float rzero);
float       MQ135_CalculateCO2PPM(float rs_ratio);
float       MQ135_CalculateCOPPM(float rs_ratio);
MQ135_AirQuality_t MQ135_AssessAirQuality(float co2_ppm);
uint8_t     MQ135_GetData(MQ135_Data_t* data);
void        MQ135_CalibrateRZero(void);
float       MQ135_GetRZero(void);
void        MQ135_SetRZero(float rzero);

#endif /* __MQ135_H__ */