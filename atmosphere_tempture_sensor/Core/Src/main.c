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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t uart_tx_buffer[256];
uint8_t uart_rx_buffer[256];
uint8_t system_ready = 0;
uint32_t last_sample_time = 0;
MQ135_Data_t mq135_data;

/* 接收 ESP32 建议数据的缓冲区 */
char esp_suggestion_buffer[512];
volatile uint8_t rx_index = 0;
volatile uint8_t suggestion_ready = 0;   /* 收到完整建议，待主循环显示 */
uint32_t suggestion_hold_until = 0;       /* 建议显示保持到该时刻 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
/* 函数声明 */
void SendDataToESP32(MQ135_Data_t* data);
void DisplayDataOnOLED(MQ135_Data_t* data);
void DisplaySuggestionOnOLED(const char* suggestion);
const char* GetAirQualityString(MQ135_AirQuality_t quality);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  发送数据到 ESP32
  * @note   数据格式：MQ135:PPM=%.2f,TEMP=%.2f,HUMI=%.2f,AQI=%.2f,LEVEL=%s\r\n
  */
void SendDataToESP32(MQ135_Data_t* data)
{
    /* 将空气质量等级转换为字符串 */
    const char* level_str = GetAirQualityString(data->air_quality);
    
    /* 格式化数据：包含 PPM 浓度、温湿度(预留)、AQI、等级 */
    /* 注意：温湿度暂时使用默认值，后续可添加 DHT11 传感器 */
    int len = sprintf((char*)uart_tx_buffer, 
                     "MQ135:PPM=%.2f,TEMP=25.00,HUMI=50.00,AQI=%.2f,LEVEL=%s\r\n",
                     data->co2_ppm,
                     data->co2_ppm,
                     level_str);
    
    /* 通过 USART1 发送 */
    HAL_UART_Transmit(&huart1, uart_tx_buffer, len, 1000);
}

/**
  * @brief  在 OLED 上显示数据
  */
void DisplayDataOnOLED(MQ135_Data_t* data)
{
    char str_buf[32];
    
    /* 清屏 */
    OLED_Clear(&hi2c1);
    
    /* 第 1 行：标题 */
    OLED_ShowString(&hi2c1, 0, 0, "Air Quality Monitor");
    
    /* 第 3 行：CO2 浓度 */
    OLED_ShowString(&hi2c1, 0, 16, "CO2:");
    sprintf(str_buf, "%.1f PPM", data->co2_ppm);
    OLED_ShowString(&hi2c1, 36, 16, str_buf);
    
    /* 第 4 行：CO 浓度 */
    OLED_ShowString(&hi2c1, 0, 24, "CO:");
    sprintf(str_buf, "%.1f PPM", data->co_ppm);
    OLED_ShowString(&hi2c1, 30, 24, str_buf);
    
    /* 第 5 行：空气质量等级 */
    OLED_ShowString(&hi2c1, 0, 32, "Quality:");
    OLED_ShowString(&hi2c1, 48, 32, GetAirQualityString(data->air_quality));
    
    /* 第 7 行：ADC 原始值 */
    OLED_ShowString(&hi2c1, 0, 48, "ADC:");
    sprintf(str_buf, "%d", data->adc_value);
    OLED_ShowString(&hi2c1, 30, 48, str_buf);
    
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
  * @brief  在 OLED 上显示 ESP32 发送的建议文本
  * @note   由主循环调用；使用 6x8 字库自动换行（每行约 21 字符）
  */
void DisplaySuggestionOnOLED(const char* suggestion)
{
    /* 先复制到本地，避免与中断缓冲产生竞争 */
    char local[512];
    char str_buf[32];
    __disable_irq();
    strncpy(local, esp_suggestion_buffer, sizeof(local) - 1);
    local[sizeof(local) - 1] = '\0';
    __enable_irq();

    /* 若传入参数非空则优先使用参数内容 */
    if (suggestion != NULL && strlen(suggestion) > 0)
    {
        strncpy(local, suggestion, sizeof(local) - 1);
        local[sizeof(local) - 1] = '\0';
    }

    /* 截断建议文本，最多显示 3 行（63 字符），避免盖住底部传感器数据 */
    if (strlen(local) > 63)
    {
        local[63] = '\0';
    }

    OLED_Clear(&hi2c1);
    OLED_ShowString(&hi2c1, 0, 0, "Suggestion:");
    OLED_ShowString(&hi2c1, 0, 8, local);

    /* 底部显示传感器实时数据，与建议同屏 */
    OLED_ShowString(&hi2c1, 0, 40, "CO2:");
    sprintf(str_buf, "%.1f PPM", mq135_data.co2_ppm);
    OLED_ShowString(&hi2c1, 30, 40, str_buf);
    OLED_ShowString(&hi2c1, 0, 48, "CO:");
    sprintf(str_buf, "%.1f PPM", mq135_data.co_ppm);
    OLED_ShowString(&hi2c1, 24, 48, str_buf);
    OLED_ShowString(&hi2c1, 0, 56, "Level:");
    OLED_ShowString(&hi2c1, 36, 56, GetAirQualityString(mq135_data.air_quality));

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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
        /* 收到 ESP32 建议：优先显示并保持 5 秒 */
    if (suggestion_ready)
    {
        suggestion_ready = 0;
        DisplaySuggestionOnOLED(NULL);
        suggestion_hold_until = HAL_GetTick() + 5000;
    }
    
        /* 每 2 秒采样一次并发送到 ESP32（不受建议显示影响，保持数据实时） */
    if ((HAL_GetTick() - last_sample_time) >= 2000)
    {
        last_sample_time = HAL_GetTick();
        
        /* 读取传感器数据 */
        if (MQ135_GetData(&mq135_data) == 0)
        {
            /* 始终发送数据到 ESP32 */
            SendDataToESP32(&mq135_data);
            
            /* 建议显示结束后，OLED 恢复传感器界面 */
            if (HAL_GetTick() >= suggestion_hold_until)
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