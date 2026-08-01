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

/* MQ-135 传感器参数 (基于 GitHub 成熟开源库优化) */
#define MQ135_RLOAD         4.4f        /* 负载电阻 (kOhm), 实测板子 A0-GND ≈ 4.3-4.5kΩ */
#define MQ135_RZERO         4.8f        /* 清洁空气中传感器电阻 (kOhm), 20K 档位实测校准值 */

/* 气体灵敏度参数 (PPM = a * (Rs/R0)^b, 来自数据手册温和拟合) */
/* 说明: MQ135 为单传感元件，CO2 为主指标（相对可信），
   CO/酒精/甲苯/氨气/丙酮为估算参考值（量级参考，非精确测量） */
#define MQ135_OFFSET_CO2    0.0f      /* CO2 基线已由 a=400 提供 */
#define MQ135_OFFSET_CO     0.0f      /* CO 偏移补偿 */
#define MQ135_OFFSET_ALC    0.0f      /* 酒精偏移补偿 */
#define MQ135_OFFSET_TOL    0.0f      /* 甲苯偏移补偿 */
#define MQ135_OFFSET_NH4    0.0f      /* 氨气偏移补偿 */
#define MQ135_OFFSET_ACE    0.0f      /* 丙酮偏移补偿 */

/* CO2 曲线 (数据手册): ratio=1→400ppm, 0.5→1000ppm, 0.3→1930ppm */
#define MQ135_SCO2          400.0f    /* CO2 曲线系数 */
#define MQ135_BCO2          -1.323f   /* CO2 指数 */

/* 保守估算曲线: ratio=1→几~几十量级, 0.3→上限附近 */
#define MQ135_SCO           8.0f      /* CO 曲线系数 */
#define MQ135_BCO           -2.0f     /* CO 指数 */
#define MQ135_SALCOHOL      6.0f      /* 酒精曲线系数 */
#define MQ135_BALCOHOL      -1.8f     /* 酒精指数 */
#define MQ135_STOL          5.0f      /* 甲苯曲线系数 */
#define MQ135_BTOL          -1.6f     /* 甲苯指数 */
#define MQ135_SNH4          7.0f      /* 氨气曲线系数 */
#define MQ135_BNH4          -1.9f     /* 氨气指数 */
#define MQ135_SACETONE      4.0f      /* 丙酮曲线系数 */
#define MQ135_BACETONE      -1.5f     /* 丙酮指数 */

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
    float rs;                        /* 传感器电阻 (kOhm) */
    float rs_ratio;                  /* Rs/R0 比值 */
    float co2_ppm;                   /* CO2 浓度 (PPM) */
    float co_ppm;                    /* CO 浓度 (PPM) */
    float alcohol_ppm;               /* 酒精浓度 (PPM) */
    float toluene_ppm;               /* 甲苯浓度 (PPM) */
    float nh4_ppm;                   /* 氨气浓度 (PPM) */
    float acetone_ppm;               /* 丙酮浓度 (PPM) */
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
float       MQ135_CalculateAlcoholPPM(float rs_ratio);
float       MQ135_CalculateToluenePPM(float rs_ratio);
float       MQ135_CalculateNH4PPM(float rs_ratio);
float       MQ135_CalculateAcetonePPM(float rs_ratio);
MQ135_AirQuality_t MQ135_AssessAirQuality(float co2_ppm);
uint8_t     MQ135_GetData(MQ135_Data_t* data);
void        MQ135_CalibrateRZero(void);
float       MQ135_GetRZero(void);
void        MQ135_SetRZero(float rzero);
void        MQ135_SetEnvironment(float temperature, float humidity);  /* 设置环境温湿度用于补偿 */

#endif /* __MQ135_H__ */