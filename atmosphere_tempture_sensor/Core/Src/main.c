/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Hardware/MQ135.h"
#include "Hardware/OLED.h"
#include "Hardware/W25QXX.h"
#include "Hardware/AHT20.h"
#include "Hardware/BMP280.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* MQ135 校准数据 Flash 存储 (W25QXX 起始扇区) */
#define MQ135_CALIB_ADDR   0x00000000UL
#define MQ135_CALIB_MAGIC  0x4D513136UL  /* "MQ16" - v2: RLOAD 实测 4.4kΩ 后校准格式 */

typedef struct {
    uint32_t magic;
    float    rzero;
    uint32_t checksum;
} MQ135_Calib_t;

/* OLED 显示模式 (建议界面与传感器多页界面交替) */
#define DISPLAY_MODE_SUGGESTION  0
#define DISPLAY_MODE_SENSOR      1
#define DISPLAY_SWITCH_MS        10000  /* 每个界面停留 10 秒 */

/* 调试开关: 1=将 USART1 收到的原始行转发到 USART3 (VOFA+)，验证接收；验证后改 0 */
#define DEBUG_RX_FWD  0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t uart_tx_buffer[256];
uint8_t uart_rx_buffer[256];
uint8_t system_ready = 0;
uint32_t last_sample_time = 0;
uint32_t last_page_switch_time = 0;
uint8_t current_display_page = 0;
MQ135_Data_t mq135_data;
volatile uint8_t force_calib = 0;  /* 强制校准标志（串口接收'1'触发） */

/* 室外空气质量数据（从 ESP32 获取） */
typedef struct {
    float co2;          /* 室外 CO2 (PPM) */
    float co;           /* 室外 CO (PPM) */
    float alcohol;      /* 室外酒精 (PPM) */
    float toluene;      /* 室外甲苯 (PPM) */
    float nh3;          /* 室外氨气 (PPM) */
    float acetone;      /* 室外丙酮 (PPM) */
    uint8_t valid;      /* 数据是否有效 */
} OutdoorData_t;

OutdoorData_t outdoor_data = {0};

/* AHT20 温湿度 + BMP280 气压数据 */
float env_temperature = 25.0f;   /* 室内温度 (℃) */
float env_humidity = 50.0f;      /* 室内湿度 (%RH) */
float env_pressure = 1013.25f;   /* 气压 (hPa) */
float env_altitude = 0.0f;       /* 海拔 (m) */
uint8_t aht20_ok = 0;            /* AHT20 初始化成功标志 */
uint8_t bmp280_ok = 0;           /* BMP280 初始化成功标志 */

/* 接收 ESP32 建议数据的缓冲区 */
char esp_suggestion_buffer[512];
volatile uint8_t rx_index = 0;
volatile uint8_t suggestion_ready = 0;   /* 收到完整数据，待主循环解析 */
uint32_t suggestion_hold_until = 0;       /* 建议显示保持到该时刻 */
uint8_t display_mode = DISPLAY_MODE_SENSOR;    /* 当前 OLED 显示模式 */
uint8_t suggestion_pending = 0;                /* 传感器界面期间收到建议，待切换显示 */
uint32_t display_until = 0;                    /* 当前显示模式截止时刻 */

/* ESP32 融合数据回显 (调试对比用) */
float fusion_co2 = 0.0f;
float fusion_co = 0.0f;
uint8_t fusion_valid = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
/* 函数声明 */
void SendDataToESP32(MQ135_Data_t* data);
void SendDataToVOFA(MQ135_Data_t* data);
void DisplayDataOnOLED(MQ135_Data_t* data);
void DisplaySuggestionOnOLED(const char* suggestion);
void ParseESP32FusionData(const char* data);
void GenerateAirQualitySuggestion(MQ135_Data_t* indoor, OutdoorData_t* outdoor, char* buffer, int len);
const char* GetAirQualityString(MQ135_AirQuality_t quality);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  R0 校准函数
  * @note   在清洁空气中运行，获取准确的 R0 值
  *         校准过程：采集 10 次 Rs 值取平均，作为 R0
  * @retval 校准后的 R0 值 (kOhm)
  */
float CalibrateR0(void)
{
    float rs_sum = 0.0f;
    uint8_t i;
    
    /* 显示校准提示 */
    OLED_Clear(&hi2c1);
    OLED_ShowString(&hi2c1, 0, 0, "Calibrating...");
    OLED_ShowString(&hi2c1, 0, 2, "Please wait 10s");
    
    /* 预热 2 秒 */
    HAL_Delay(2000);
    
    /* 采集 10 次 Rs 值 */
    for (i = 0; i < 10; i++)
    {
        rs_sum += MQ135_CalculateRS(MQ135_ReadADC());
        HAL_Delay(1000);  /* 每秒采集一次 */
        
        /* 显示进度 */
        char progress[16];
        sprintf(progress, "Progress: %d/10", i + 1);
        OLED_ShowString(&hi2c1, 0, 4, progress);
    }
    
    /* 计算平均 R0 */
    float r0 = rs_sum / 10.0f;
    
    /* 显示校准结果 */
    OLED_Clear(&hi2c1);
    OLED_ShowString(&hi2c1, 0, 0, "Calibration Done");
    char result[32];
    sprintf(result, "R0 = %.2f kOhm", r0);
    OLED_ShowString(&hi2c1, 0, 2, result);
    OLED_ShowString(&hi2c1, 0, 4, "Save to Flash?");
    OLED_ShowString(&hi2c1, 0, 6, "Auto saving...");
    
    /* 自动保存，无需等待 */
    HAL_Delay(1000);
    
    /* 保存到 Flash */
    W25QXX_Init();
    W25QXX_Write(0x000000, (uint8_t*)&r0, sizeof(float));
    
    OLED_ShowString(&hi2c1, 0, 7, "Saved!");
    HAL_Delay(2000);
    
    return r0;
}

/**
  * @brief  发送数据到 VOFA+ 上位机（USART3）
  * @note   使用 ASCII 可视化格式，直接在串口终端显示形象的仪表盘
  *         包含：室内数据 + 室外数据 + 对比分析 + 建议
  */
void SendDataToVOFA(MQ135_Data_t* data)
{
    int len;
    
    /* 空气质量等级字符串 */
    const char* aq_str[] = {"Excellent", "Good", "Moderate", "Poor", "Bad", "Hazardous"};
    
    /* 第一行：标题栏 */
    len = sprintf((char*)uart_tx_buffer, 
                  "\r\n========== Air Quality Monitor (Debug) ==========\r\n");
    HAL_UART_Transmit(&huart3, uart_tx_buffer, len, 100);
    
    /* STM32 原始计算数据 */
    len = sprintf((char*)uart_tx_buffer, 
                  ">>> STM32 RAW (ADC/V/Rs/RSR/R0):\r\n");
    HAL_UART_Transmit(&huart3, uart_tx_buffer, len, 100);
    len = sprintf((char*)uart_tx_buffer, 
                  "  ADC=%d | V=%.3fV | Rs=%.1fk | RSR=%.3f | R0=%.1fk\r\n",
                  data->adc_value, data->voltage, data->rs, data->rs_ratio, MQ135_GetRZero());
    HAL_UART_Transmit(&huart3, uart_tx_buffer, len, 100);
    
    /* 第二行：室内数据标题 */
    len = sprintf((char*)uart_tx_buffer, 
                  "\r\n>>> INDOOR (STM32 calc):\r\n");
    HAL_UART_Transmit(&huart3, uart_tx_buffer, len, 100);
    
    /* 第三行：室内 CO2 + CO */
    len = sprintf((char*)uart_tx_buffer, 
                  "  [CO2] %.1f PPM  |  [CO] %.1f PPM\r\n",
                  data->co2_ppm, data->co_ppm);
    HAL_UART_Transmit(&huart3, uart_tx_buffer, len, 100);
    
    /* 第四行：室内酒精 + 甲苯 */
    len = sprintf((char*)uart_tx_buffer, 
                  "  [Alcohol] %.1f PPM  |  [Toluene] %.1f PPM\r\n",
                  data->alcohol_ppm, data->toluene_ppm);
    HAL_UART_Transmit(&huart3, uart_tx_buffer, len, 100);
    
    /* 第五行：室内氨气 + 丙酮 */
    len = sprintf((char*)uart_tx_buffer, 
                  "  [NH3] %.1f PPM  |  [Acetone] %.1f PPM\r\n",
                  data->nh4_ppm, data->acetone_ppm);
    HAL_UART_Transmit(&huart3, uart_tx_buffer, len, 100);
    
    /* 第六行：环境温湿度/气压 (AHT20+BMP280) */
    len = sprintf((char*)uart_tx_buffer, 
                  "  [Temp] %.1f C  |  [Humi] %.1f %%  |  [Pres] %.1f hPa  |  [Alt] %.0f m\r\n",
                  env_temperature, env_humidity, env_pressure, env_altitude);
    HAL_UART_Transmit(&huart3, uart_tx_buffer, len, 100);
    
    /* ESP32 融合数据回显对比 */
    len = sprintf((char*)uart_tx_buffer, 
                  "\r\n>>> FUSION (ESP32 echo):\r\n");
    HAL_UART_Transmit(&huart3, uart_tx_buffer, len, 100);
    if (fusion_valid)
    {
        len = sprintf((char*)uart_tx_buffer, 
                      "  STM32 CO2: %.1f  |  ESP32 CO2: %.1f  |  Diff: %+.1f\r\n",
                      data->co2_ppm, fusion_co2, data->co2_ppm - fusion_co2);
        HAL_UART_Transmit(&huart3, uart_tx_buffer, len, 100);
        len = sprintf((char*)uart_tx_buffer, 
                      "  STM32 CO: %.1f  |  ESP32 CO: %.1f  |  Diff: %+.1f\r\n",
                      data->co_ppm, fusion_co, data->co_ppm - fusion_co);
        HAL_UART_Transmit(&huart3, uart_tx_buffer, len, 100);
    }
    else
    {
        len = sprintf((char*)uart_tx_buffer, 
                      "  No fusion data yet\r\n");
        HAL_UART_Transmit(&huart3, uart_tx_buffer, len, 100);
    }
    
    /* 第六行：室外数据标题 */
    len = sprintf((char*)uart_tx_buffer, 
                  "\r\n>>> OUTDOOR (ESP32):\r\n");
    HAL_UART_Transmit(&huart3, uart_tx_buffer, len, 100);
    
    if (outdoor_data.valid)
    {
        /* 第七行：室外 CO2 + CO */
        len = sprintf((char*)uart_tx_buffer, 
                      "  [CO2] %.1f PPM  |  [CO] %.1f PPM\r\n",
                      outdoor_data.co2, outdoor_data.co);
        HAL_UART_Transmit(&huart3, uart_tx_buffer, len, 100);
        
        /* 第八行：室外酒精 + 甲苯 */
        len = sprintf((char*)uart_tx_buffer, 
                      "  [Alcohol] %.1f PPM  |  [Toluene] %.1f PPM\r\n",
                      outdoor_data.alcohol, outdoor_data.toluene);
        HAL_UART_Transmit(&huart3, uart_tx_buffer, len, 100);
        
        /* 第九行：室外氨气 + 丙酮 */
        len = sprintf((char*)uart_tx_buffer, 
                      "  [NH3] %.1f PPM  |  [Acetone] %.1f PPM\r\n",
                      outdoor_data.nh3, outdoor_data.acetone);
        HAL_UART_Transmit(&huart3, uart_tx_buffer, len, 100);
    }
    else
    {
        len = sprintf((char*)uart_tx_buffer, 
                      "  Waiting for ESP32 data...\r\n");
        HAL_UART_Transmit(&huart3, uart_tx_buffer, len, 100);
    }
    
    /* 第十行：对比分析 */
    len = sprintf((char*)uart_tx_buffer, 
                  "\r\n>>> COMPARISON:\r\n");
    HAL_UART_Transmit(&huart3, uart_tx_buffer, len, 100);
    
    if (outdoor_data.valid)
    {
        float co2_diff = data->co2_ppm - outdoor_data.co2;
        float co_diff = data->co_ppm - outdoor_data.co;
        
        len = sprintf((char*)uart_tx_buffer, 
                      "  CO2 Diff: %+.1f PPM  |  CO Diff: %+.1f PPM\r\n",
                      co2_diff, co_diff);
        HAL_UART_Transmit(&huart3, uart_tx_buffer, len, 100);
    }
    else
    {
        len = sprintf((char*)uart_tx_buffer, 
                      "  No outdoor data for comparison\r\n");
        HAL_UART_Transmit(&huart3, uart_tx_buffer, len, 100);
    }
    
    /* 第十一行：建议 */
    char suggestion[64];
    GenerateAirQualitySuggestion(data, &outdoor_data, suggestion, sizeof(suggestion));
    
    len = sprintf((char*)uart_tx_buffer, 
                  "\r\n>>> SUGGESTION:\r\n  %s\r\n",
                  suggestion);
    HAL_UART_Transmit(&huart3, uart_tx_buffer, len, 100);
    
    /* 第十二行：空气质量等级 */
    len = sprintf((char*)uart_tx_buffer, 
                  "\r\n[Quality] %s (%d)\r\n",
                  aq_str[data->air_quality], data->air_quality);
    HAL_UART_Transmit(&huart3, uart_tx_buffer, len, 100);
    
    /* 第十三行：传感器状态 */
    len = sprintf((char*)uart_tx_buffer, 
                  "[Sensor] V=%.2fV, Rs=%.1fkOhm, Ratio=%.3f\r\n",
                  data->voltage, data->rs, data->rs_ratio);
    HAL_UART_Transmit(&huart3, uart_tx_buffer, len, 100);
    
    /* 第十四行：分隔线 */
    len = sprintf((char*)uart_tx_buffer, 
                  "===========================================\r\n");
    HAL_UART_Transmit(&huart3, uart_tx_buffer, len, 100);
}

/**
  * @brief  发送数据到 ESP32（包含原始数据供 ESP32 独立计算）
  * @note   数据格式：MQ135:ADC=%d,V=%.3f,RS=%.2f,RSR=%.3f,CO2=%.2f,CO=%.2f,ALC=%.2f,TOL=%.2f,NH4=%.2f,ACE=%.2f,AQ=%d\r\n
  *         ESP32 接收后使用相同算法独立计算，然后进行数据融合提高精度
  */
void SendDataToESP32(MQ135_Data_t* data)
{
    /* 格式化数据：STM32 计算结果 + 原始数据 + 环境温湿度/气压（供 ESP32 分析） */
    int len = sprintf((char*)uart_tx_buffer, 
                     "MQ135:ADC=%d,V=%.3f,RS=%.2f,RSR=%.3f,CO2=%.2f,CO=%.2f,ALC=%.2f,TOL=%.2f,NH4=%.2f,ACE=%.2f,AQ=%d,TEMP=%.2f,HUMI=%.2f,PRES=%.2f\r\n",
                     data->adc_value,
                     data->voltage,
                     data->rs,
                     data->rs_ratio,
                     data->co2_ppm,
                     data->co_ppm,
                     data->alcohol_ppm,
                     data->toluene_ppm,
                     data->nh4_ppm,
                     data->acetone_ppm,
                     data->air_quality,
                     env_temperature,
                     env_humidity,
                     env_pressure);
    
    /* 通过 USART1 发送 */
    HAL_UART_Transmit(&huart1, uart_tx_buffer, len, 1000);
}

/**
  * @brief  解析 ESP32 返回的融合数据
  * @note   数据格式：FUSION:CO2=%.2f,CO=%.2f,ALC=%.2f,TOL=%.2f,NH4=%.2f,ACE=%.2f,AQ=%d\r\n
  */
void ParseESP32FusionData(const char* data)
{
    if (strstr(data, "FUSION:") != NULL)
    {
        /* 解析融合后的数据 */
        float fused_co2, fused_co, fused_alc, fused_tol, fused_nh4, fused_ace;
        int fused_aq;
        
        if (sscanf(data, "FUSION:CO2=%f,CO=%f,ALC=%f,TOL=%f,NH4=%f,ACE=%f,AQ=%d",
                   &fused_co2, &fused_co, &fused_alc, &fused_tol, &fused_nh4, &fused_ace, &fused_aq) == 7)
        {
            /* 保存 ESP32 融合回显值 (调试对比用) */
            fusion_co2 = fused_co2;
            fusion_co = fused_co;
            fusion_valid = 1;
            
            /* 更新显示数据为融合后的值 */
            mq135_data.co2_ppm = fused_co2;
            mq135_data.co_ppm = fused_co;
            mq135_data.alcohol_ppm = fused_alc;
            mq135_data.toluene_ppm = fused_tol;
            mq135_data.nh4_ppm = fused_nh4;
            mq135_data.acetone_ppm = fused_ace;
            mq135_data.air_quality = (MQ135_AirQuality_t)fused_aq;
            
            /* 标记融合数据已更新（主循环负责显示） */
            display_until = HAL_GetTick() + DISPLAY_SWITCH_MS;  /* 保持显示 10 秒 */
        }
    }
    else if (strstr(data, "OUTDOOR:") != NULL)
    {
        /* 解析室外数据 */
        if (sscanf(data, "OUTDOOR:CO2=%f,CO=%f,ALC=%f,TOL=%f,NH4=%f,ACE=%f",
                   &outdoor_data.co2, &outdoor_data.co, &outdoor_data.alcohol,
                   &outdoor_data.toluene, &outdoor_data.nh3, &outdoor_data.acetone) == 6)
        {
            outdoor_data.valid = 1;
        }
    }
}

/**
  * @brief  生成空气质量建议
  * @param  indoor: 室内数据
  * @param  outdoor: 室外数据
  * @param  buffer: 输出缓冲区
  * @param  len: 缓冲区长度
  */
void GenerateAirQualitySuggestion(MQ135_Data_t* indoor, OutdoorData_t* outdoor, char* buffer, int len)
{
    if (!outdoor->valid)
    {
        snprintf(buffer, len, "No outdoor data");
        return;
    }
    
    float co2_diff = indoor->co2_ppm - outdoor->co2;
    float co_diff = indoor->co_ppm - outdoor->co;
    
    /* 综合室内外差异 + 环境温湿度给出建议 */
    if (co2_diff > 200.0f)
    {
        snprintf(buffer, len, "High CO2! Open window");
    }
    else if (co_diff > 20.0f)
    {
        snprintf(buffer, len, "High CO! Check gas");
    }
    else if (indoor->co2_ppm > 1000.0f)
    {
        snprintf(buffer, len, "Ventilate room");
    }
    else if (env_humidity > 75.0f)
    {
        snprintf(buffer, len, "Humid %.0f%%! Ventilate", env_humidity);
    }
    else if (env_temperature > 32.0f)
    {
        snprintf(buffer, len, "Hot %.1fC! Ventilate", env_temperature);
    }
    else if (co2_diff < -100.0f)
    {
        snprintf(buffer, len, "Good ventilation");
    }
    else
    {
        snprintf(buffer, len, "Air quality OK");
    }
}

/**
  * @brief  在 OLED 上显示数据（多页循环显示所有参数）
  * @note   第1页：CO2 + CO 浓度（主要指标）
  *         第2页：酒精 + 甲苯（有机挥发物）
  *         第3页：氨气 + 丙酮（工业气体）
  *         第4页：传感器状态（电压 + Rs + R0）
  *         第5页：原始数据（ADC + Rs/R0 + 校准状态）
  *         第6页：室内外对比
  *         第7页：数据融合状态
  *         每3秒自动切换一页
  */
void DisplayDataOnOLED(MQ135_Data_t* data)
{
    char str_buf[32];
    uint32_t current_time = HAL_GetTick();
    
    /* 每 5 秒切换一页（3 页循环：气体/ VOC / 环境） */
    if ((current_time - last_page_switch_time) >= 5000)
    {
        last_page_switch_time = current_time;
        current_display_page = (current_display_page + 1) % 3;
    }
    
    /* 清屏 */
    OLED_Clear(&hi2c1);
    
    switch(current_display_page)
    {
        /* ===== 第 1 页：核心气体 ===== */
        case 0:
            OLED_ShowString(&hi2c1, 0, 0, "Air:");
            OLED_ShowString(&hi2c1, 30, 0, GetAirQualityString(data->air_quality));
            
            OLED_ShowString(&hi2c1, 0, 16, "CO2:");
            sprintf(str_buf, "%.0f PPM", data->co2_ppm);
            OLED_ShowString(&hi2c1, 30, 16, str_buf);
            
            OLED_ShowString(&hi2c1, 0, 32, "CO:");
            sprintf(str_buf, "%.1f PPM", data->co_ppm);
            OLED_ShowString(&hi2c1, 24, 32, str_buf);
            
            OLED_ShowString(&hi2c1, 0, 48, "[1/3] Gases");
            break;
            
        /* ===== 第 2 页：VOC 气体 ===== */
        case 1:
            OLED_ShowString(&hi2c1, 0, 0, "VOC Gases");
            
            OLED_ShowString(&hi2c1, 0, 16, "Alc:");
            sprintf(str_buf, "%.1f", data->alcohol_ppm);
            OLED_ShowString(&hi2c1, 30, 16, str_buf);
            
            OLED_ShowString(&hi2c1, 0, 24, "Tol:");
            sprintf(str_buf, "%.1f", data->toluene_ppm);
            OLED_ShowString(&hi2c1, 30, 24, str_buf);
            
            OLED_ShowString(&hi2c1, 0, 32, "NH3:");
            sprintf(str_buf, "%.1f", data->nh4_ppm);
            OLED_ShowString(&hi2c1, 30, 32, str_buf);
            
            OLED_ShowString(&hi2c1, 0, 40, "Ace:");
            sprintf(str_buf, "%.1f", data->acetone_ppm);
            OLED_ShowString(&hi2c1, 30, 40, str_buf);
            
            OLED_ShowString(&hi2c1, 0, 48, "[2/3] VOC");
            break;
            
        /* ===== 第 3 页：环境参数 ===== */
        case 2:
            OLED_ShowString(&hi2c1, 0, 0, "Environment");
            
            OLED_ShowString(&hi2c1, 0, 16, "Temp:");
            sprintf(str_buf, "%.1f C", env_temperature);
            OLED_ShowString(&hi2c1, 36, 16, str_buf);
            
            OLED_ShowString(&hi2c1, 0, 24, "Humi:");
            sprintf(str_buf, "%.1f %%", env_humidity);
            OLED_ShowString(&hi2c1, 36, 24, str_buf);
            
            OLED_ShowString(&hi2c1, 0, 32, "Pres:");
            sprintf(str_buf, "%.0f hPa", env_pressure);
            OLED_ShowString(&hi2c1, 36, 32, str_buf);
            
            OLED_ShowString(&hi2c1, 0, 48, "[3/3] Env");
            break;
            
        default:
            current_display_page = 0;
            break;
    }
    
    /* 刷新显示 */
    OLED_Refresh(&hi2c1);
}

/**
  * @brief  获取空气质量等级字符串
  */
const char* GetAirQualityString(MQ135_AirQuality_t quality)
{
    switch(quality)
    {
        case MQ135_AIR_QUALITY_EXCELLENT: return "Excellent";
        case MQ135_AIR_QUALITY_GOOD:      return "Good";
        case MQ135_AIR_QUALITY_MODERATE:  return "Moderate";
        case MQ135_AIR_QUALITY_POOR:      return "Poor";
        case MQ135_AIR_QUALITY_BAD:       return "Bad";
        case MQ135_AIR_QUALITY_HAZARDOUS: return "Hazardous";
        default: return "Unknown";
    }
}

/**
  * @brief  UART 接收完成回调函数
  * @note   接收来自 ESP32 的建议数据
  *         仅在中断中缓冲字节并置标志，不执行 I2C/OLED 等阻塞操作，
  *         实际显示由主循环调用 DisplaySuggestionOnOLED() 完成。
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uint8_t c = uart_rx_buffer[0];

        /* 忽略 \r，仅保留纯文本 */
        if (c != '\r')
        {
            /* 将接收到的字节存入缓冲区 */
            if (rx_index < sizeof(esp_suggestion_buffer) - 1)
            {
                esp_suggestion_buffer[rx_index++] = (char)c;
            }

            /* 检测换行符，表示一条完整消息 */
            if (c == '\n' || rx_index >= sizeof(esp_suggestion_buffer) - 1)
            {
                esp_suggestion_buffer[rx_index] = '\0';
                suggestion_ready = 1;   /* 通知主循环显示 */
                rx_index = 0;
            }
        }

        /* 继续接收下一个字节 */
        HAL_UART_Receive_IT(&huart1, &uart_rx_buffer[0], 1);
    }
}

/**
  * @brief  UART 错误回调
  * @note   发生溢出/帧错误后重新启动接收，防止通信停滞
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UART_Receive_IT(&huart1, &uart_rx_buffer[0], 1);
    }
}

/**
  * @brief  在 OLED 上显示建议文本 + 融合数据
  * @note   由主循环调用；suggestion 为生成的建议文本
  */
void DisplaySuggestionOnOLED(const char* suggestion)
{
    char local[128];
    char str_buf[32];
    
    if (suggestion == NULL)
    {
        return;
    }
    
    strncpy(local, suggestion, sizeof(local) - 1);
    local[sizeof(local) - 1] = '\0';

    /* 建议最多 2 行（42 字符），保持界面简洁 */
    if (strlen(local) > 42)
    {
        local[42] = '\0';
    }

    OLED_Clear(&hi2c1);
    OLED_ShowString(&hi2c1, 0, 0, "Suggestion:");
    OLED_ShowString(&hi2c1, 0, 8, local);

    /* 底部显示核心数据：CO2 + 温湿度 */
    OLED_ShowString(&hi2c1, 0, 40, "CO2:");
    sprintf(str_buf, "%.0f PPM", mq135_data.co2_ppm);
    OLED_ShowString(&hi2c1, 30, 40, str_buf);
    OLED_ShowString(&hi2c1, 0, 48, "T/H:");
    sprintf(str_buf, "%.1fC / %.0f%%", env_temperature, env_humidity);
    OLED_ShowString(&hi2c1, 30, 48, str_buf);

    OLED_Refresh(&hi2c1);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  MQ135_Calib_t calib;      /* MQ135 校准数据 (存 Flash) */
  uint8_t need_calib = 1;   /* 是否需要重新校准 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  /* 初始化 W25QXX Flash */
  if (W25QXX_Init() == W25QXX_OK)
  {
      /* Flash 初始化成功 */
  }
  
  /* 初始化 OLED 显示屏 */
  OLED_Init(&hi2c1);
  OLED_ShowString(&hi2c1, 0, 0, "System Init...");
  OLED_Refresh(&hi2c1);
  
  /* 初始化 MQ-135 传感器 */
  MQ135_Init(&hadc1);
  
  /* 初始化 AHT20 温湿度传感器 (I2C1, 与 OLED 共总线) */
  aht20_ok = AHT20_Init(&hi2c1);
  
  /* 初始化 BMP280 气压传感器 (I2C1, 自动探测地址) */
  bmp280_ok = BMP280_Init(&hi2c1);
  
  /* 从 Flash 读取上次校准的 R0，有效则免校准直接使用 */
  if (W25QXX_Read(MQ135_CALIB_ADDR, (uint8_t*)&calib, sizeof(calib)) == W25QXX_OK)
  {
      union { float f; uint32_t u; } conv;
      conv.f = calib.rzero;
      if (calib.magic == MQ135_CALIB_MAGIC
          && calib.checksum == (MQ135_CALIB_MAGIC ^ conv.u)
          && calib.rzero > 1.0f && calib.rzero < 1000.0f)
      {
          MQ135_SetRZero(calib.rzero);
          need_calib = 0;
      }
  }
  
  /* 显示当前 R0 值 */
  char r0_str[32];
  sprintf(r0_str, "Current R0=%.1f", MQ135_GetRZero());
  OLED_ShowString(&hi2c1, 0, 48, r0_str);
  OLED_Refresh(&hi2c1);
  HAL_Delay(2000);
  
  /* 检查串口命令：接收'1'则强制重新校准 */
  uint8_t rx_byte = 0;
  if (HAL_UART_Receive(&huart3, &rx_byte, 1, 0) == HAL_OK)
  {
      if (rx_byte == '1')
      {
          force_calib = 1;
          OLED_Clear(&hi2c1);
          OLED_ShowString(&hi2c1, 0, 0, "Command Received");
          OLED_ShowString(&hi2c1, 0, 16, "Force Calibrate");
          OLED_Refresh(&hi2c1);
          HAL_Delay(1000);
      }
  }
  
  if (need_calib)
  {
      /* 热机：MQ135 断电后需预热读数才稳定，先等待 10 秒 */
      OLED_Clear(&hi2c1);
      OLED_ShowString(&hi2c1, 0, 0, "Warming up...");
      OLED_ShowString(&hi2c1, 0, 16, "Keep fresh air");
      OLED_ShowString(&hi2c1, 0, 32, "10 seconds");
      OLED_Refresh(&hi2c1);
      HAL_Delay(10000);
      
      /* 上电自动校准 R0：需在清洁空气中进行（期间请勿对传感器吹气/遮挡） */
      OLED_Clear(&hi2c1);
      OLED_ShowString(&hi2c1, 0, 0, "Calibrating MQ135");
      OLED_ShowString(&hi2c1, 0, 16, "Fresh air only");
      OLED_ShowString(&hi2c1, 0, 32, "Please wait");
      OLED_Refresh(&hi2c1);
      MQ135_CalibrateRZero();
      
      /* 保存校准值到 Flash（先擦除扇区再写入） */
      union { float f; uint32_t u; } conv;
      conv.f = MQ135_GetRZero();
      calib.magic = MQ135_CALIB_MAGIC;
      calib.rzero = conv.f;
      calib.checksum = MQ135_CALIB_MAGIC ^ conv.u;
      W25QXX_SectorErase(MQ135_CALIB_ADDR);
      W25QXX_Write(MQ135_CALIB_ADDR, (uint8_t*)&calib, sizeof(calib));
  }
  
  /* 显示初始化完成 */
  OLED_Clear(&hi2c1);
  OLED_ShowString(&hi2c1, 0, 0, "System Ready");
  OLED_ShowString(&hi2c1, 0, 16, "STM32+ESP32");
  OLED_ShowString(&hi2c1, 0, 32, "Air Quality");
  OLED_Refresh(&hi2c1);
  HAL_Delay(1000);
  
  system_ready = 1;
  
  /* 使能 USART1 全局中断（CubeMX 工程未勾选，手动补上） */
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  
  /* 启动 UART 接收中断，用于接收 ESP32 发送的建议数据 */
  HAL_UART_Receive_IT(&huart1, &uart_rx_buffer[0], 1);
  
  /* 初始 OLED 显示模式：先显示 10 秒传感器多页界面 */
  display_mode = DISPLAY_MODE_SENSOR;
  display_until = HAL_GetTick() + DISPLAY_SWITCH_MS;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 检查串口命令：接收'1'则强制重新校准 */
    uint8_t rx_byte = 0;
    if (HAL_UART_Receive(&huart3, &rx_byte, 1, 0) == HAL_OK)
    {
        if (rx_byte == '1')
        {
            force_calib = 1;
        }
    }
    
    /* 执行强制校准 */
    if (force_calib)
    {
        force_calib = 0;
        OLED_Clear(&hi2c1);
        OLED_ShowString(&hi2c1, 0, 0, "Command Received");
        OLED_ShowString(&hi2c1, 0, 16, "Force Calibrate");
        OLED_Refresh(&hi2c1);
        HAL_Delay(1000);
        
        /* 热机 10 秒 */
        OLED_Clear(&hi2c1);
        OLED_ShowString(&hi2c1, 0, 0, "Warming up...");
        OLED_ShowString(&hi2c1, 0, 16, "Keep fresh air");
        OLED_ShowString(&hi2c1, 0, 32, "10 seconds");
        OLED_Refresh(&hi2c1);
        HAL_Delay(10000);
        
        /* 校准 R0 */
        OLED_Clear(&hi2c1);
        OLED_ShowString(&hi2c1, 0, 0, "Calibrating MQ135");
        OLED_ShowString(&hi2c1, 0, 16, "Fresh air only");
        OLED_ShowString(&hi2c1, 0, 32, "Please wait");
        OLED_Refresh(&hi2c1);
        MQ135_CalibrateRZero();
        
        /* 保存到 Flash */
        union { float f; uint32_t u; } conv;
        conv.f = MQ135_GetRZero();
        calib.magic = MQ135_CALIB_MAGIC;
        calib.rzero = conv.f;
        calib.checksum = MQ135_CALIB_MAGIC ^ conv.u;
        W25QXX_SectorErase(MQ135_CALIB_ADDR);
        W25QXX_Write(MQ135_CALIB_ADDR, (uint8_t*)&calib, sizeof(calib));
        
        /* 显示校准完成 */
        OLED_Clear(&hi2c1);
        char result[32];
        sprintf(result, "R0 = %.2f kOhm", MQ135_GetRZero());
        OLED_ShowString(&hi2c1, 0, 0, "Calibration Done");
        OLED_ShowString(&hi2c1, 0, 16, result);
        OLED_ShowString(&hi2c1, 0, 32, "Saved to Flash");
        OLED_Refresh(&hi2c1);
        HAL_Delay(3000);
    }
    
    /* 解析 ESP32 数据（FUSION:/OUTDOOR:） */
    if (suggestion_ready)
    {
        char rx_line[512];
        char suggestion[128];
        
        suggestion_ready = 0;
        
        /* 复制缓冲区（关中断避免竞争），解析 FUSION:/OUTDOOR: 数据 */
        __disable_irq();
        strncpy(rx_line, esp_suggestion_buffer, sizeof(rx_line) - 1);
        rx_line[sizeof(rx_line) - 1] = '\0';
        __enable_irq();
        ParseESP32FusionData(rx_line);
        
#if DEBUG_RX_FWD
        /* 调试：将收到的原始行转发到 VOFA+ 验证接收 */
        HAL_UART_Transmit(&huart3, (uint8_t*)"[RX] ", 5, 100);
        HAL_UART_Transmit(&huart3, (uint8_t*)rx_line, strlen(rx_line), 100);
        HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, 100);
#endif
        
        /* 建议界面模式下实时刷新建议；传感器界面模式下挂起待切换 */
        if (display_mode == DISPLAY_MODE_SUGGESTION)
        {
            GenerateAirQualitySuggestion(&mq135_data, &outdoor_data, suggestion, sizeof(suggestion));
            DisplaySuggestionOnOLED(suggestion);
        }
        else
        {
            suggestion_pending = 1;
        }
    }
    
    /* OLED 建议/传感器界面交替（各 10 秒） */
    if (HAL_GetTick() >= display_until)
    {
        if (display_mode == DISPLAY_MODE_SUGGESTION)
        {
            /* 建议界面 10 秒结束，切到传感器多页界面 */
            display_mode = DISPLAY_MODE_SENSOR;
            display_until = HAL_GetTick() + DISPLAY_SWITCH_MS;
        }
        else
        {
            /* 传感器界面 10 秒结束，切到建议界面（若期间有建议到达） */
            display_mode = DISPLAY_MODE_SUGGESTION;
            display_until = HAL_GetTick() + DISPLAY_SWITCH_MS;
            if (suggestion_pending)
            {
                suggestion_pending = 0;
                char suggestion[128];
                GenerateAirQualitySuggestion(&mq135_data, &outdoor_data, suggestion, sizeof(suggestion));
                DisplaySuggestionOnOLED(suggestion);
            }
        }
    }
    
        /* 每 2 秒采样一次并发送到 ESP32（不受显示模式影响，保持数据实时） */
    if ((HAL_GetTick() - last_sample_time) >= 2000)
    {
        last_sample_time = HAL_GetTick();
        
        /* 读取传感器数据 */
        if (MQ135_GetData(&mq135_data) == 0)
        {
            /* 读取 AHT20/BMP280 环境数据（失败保持上次值） */
            if (aht20_ok)
            {
                float t, h;
                if (AHT20_Read_Data(&hi2c1, &t, &h))
                {
                    env_temperature = t;
                    env_humidity = h;
                }
            }
            if (bmp280_ok)
            {
                float p, t, alt;
                if (BMP280_GetData(&hi2c1, &p, &t, &alt))
                {
                    env_pressure = p;
                    env_temperature = t;   /* BMP280 温度更精确，优先使用 */
                    env_altitude = alt;
                }
            }
            
            /* 设置 MQ135 温湿度补偿 */
            MQ135_SetEnvironment(env_temperature, env_humidity);
            
            /* 始终发送数据到 ESP32 */
            SendDataToESP32(&mq135_data);
            
            /* 发送数据到 VOFA+ 上位机 */
            SendDataToVOFA(&mq135_data);
            
            /* 传感器界面模式下刷新显示（内部每 3 秒翻页） */
            if (display_mode == DISPLAY_MODE_SENSOR)
            {
                DisplayDataOnOLED(&mq135_data);
            }
        }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @note  用于连接 VOFA+ 上位机，引脚 PB10(TX), PB11(RX)
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  
  /* 使能 USART3 时钟 */
  __HAL_RCC_USART3_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  /* 配置 PB10 为 USART3_TX */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /* 配置 PB11 为 USART3_RX */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */