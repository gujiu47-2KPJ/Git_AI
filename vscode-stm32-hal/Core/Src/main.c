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
#include "stm32f103xb.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_i2c.h"
#include "stm32f1xx_hal_uart.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_tim_ex.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/_intsup.h>
#include "OLED.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* ===== 串口收数字用 ===== */
uint8_t num_buf[10];
uint8_t num_idx = 0;

/* ===== 系统工作模式 ===== */
typedef enum {
    mode_idle = 0,
    mode_running = 1,
    mode_load = 2,
    mode_shark = 3
} system_mode_t;

/* ===== 编码器工作模式 ===== */
typedef enum {
    led_default_mode = 0,
    led_brights_mode,
    led_turn_mode
} turn_mode_t;

/* ===== 编码器 / PWM 变量 ===== */
volatile turn_mode_t led_mode = led_default_mode;
volatile uint32_t t3_last_cnt = 0;
volatile uint8_t mode_press_flag = 0;
volatile uint8_t led_bright = 50;
volatile uint8_t encoder_flag = 0;

/* ===== LED模式变量 ===== */
volatile system_mode_t current_mode = mode_idle;
system_mode_t last_mode = (system_mode_t)99;   /* 初始设成"不等于任何模式"，让OLED首次刷新成立 */

/* ===== 按键中断变量 ===== */
volatile uint8_t key_press = 0;
volatile uint8_t system_state = 0;

/* ===== 串口通信变量 ===== */
volatile uint8_t receive_status = 0;
const char *mode_name[] = {"IDLE", "RUNNING", "LOAD", "SHARK"};
uint8_t rx_buffer[1];
uint32_t last_uart_tick = 0;

/* ===== OLED按需刷新 ===== */
uint8_t last_bright = 0;
turn_mode_t last_led_mode = (turn_mode_t)99;  /* 初始特殊值，确保首次刷新成立 */

/* ===== 板载LED非阻塞闪烁 ===== */
volatile uint16_t bright_speed = 500;
uint32_t last_led_tick = 0;
uint32_t blink_timer = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void Serial_ProcessChar(uint8_t ch);  // 串口命令处理函数声明
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  /* ========== 关键修复：强制把EXTI线5映射到GPIOB ==========
       STM32F1每条EXTI线(0-15)需要配置它连到哪个端口
       AFIO_EXTICR2寄存器的bit[10:8]控制EXTI5:
         000 = GPIOA (默认)
         001 = GPIOB  ← 我们需要
       PA0/PA1/PA2在EXTI0/1/2，它们默认映射GPIOA所以正常
       PB5在EXTI5，如果不改配置，按下PB5永远触发不了中断！ */
    __HAL_RCC_AFIO_CLK_ENABLE();            /* 先开AFIO时钟（CubeMX可能漏了） */
    AFIO->EXTICR[1] &= ~(0x7 << 8);          /* 清bit[10:8] */
    AFIO->EXTICR[1] |=  (0x1 << 8);          /* 设为001 = 映射到GPIOB */

    /* OLED初始化 */
    OLED_Init();
    OLED_ShowString(1, 1, "LED CONTROL");

    /* TIM2定时上报（每2秒中断一次） */
    HAL_TIM_Base_Start_IT(&htim2);

    /* TIM3编码器模式（PA6/PA7） */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    /* TIM1 PWM模式（PA8/PA9/PA10 = 3路灯珠） */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    /* 设置初始占空比 = led_bright(50) */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, led_bright);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, led_bright);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, led_bright);

    /* 开启串口中断接收 */
    HAL_UART_Receive_IT(&huart1, rx_buffer, 1);

    /* 记录编码器初始值 */
    t3_last_cnt = TIM3->CNT;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {

    uint32_t now_cnt = TIM3->CNT;
    uint32_t now = HAL_GetTick();
    
    

    /* ===== 板载LED非阻塞闪烁 ===== */
   if (now - last_led_tick >= bright_speed) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        last_led_tick = now;
    }

    // 快闪1秒后恢复慢闪
    if (bright_speed == 300 && (now - blink_timer) > 2000) {  // 快闪2秒后恢复，速度从100ms放宽到300ms 
        bright_speed = 500;
    }

    /* ===== 编码器处理：根据 led_mode 执行不同操作 ===== */
    switch (led_mode) {
        case led_default_mode:
            /* 不响应旋转 */
            break;

        case led_brights_mode:
            /* 旋转 → 调亮度(步进5) */
            if (now_cnt > t3_last_cnt) {
                if (led_bright < 99) {
                    led_bright += 5;
                    encoder_flag = 1;
                }
            } else if (now_cnt < t3_last_cnt) {
                if (led_bright > 0) {
                    led_bright -= 5;
                    encoder_flag = 1;
                }
            }
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, led_bright);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, led_bright);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, led_bright);
            break;

        case led_turn_mode:
            /* 旋转 → 切模式 */
            if (now_cnt > t3_last_cnt) {
                current_mode = (current_mode + 1) % 4;
                encoder_flag = 1;
            } else if (now_cnt < t3_last_cnt) {
                current_mode = (current_mode - 1 + 4) % 4;
                encoder_flag = 1;
            }
            break;

        default:
            break;
    }
    t3_last_cnt = now_cnt;

    /* ===== 编码器按键(PB5)：切 led_mode ===== */
    if (mode_press_flag != 0) {
        led_mode = (led_mode + 1) % 3;  /* 循环切换编码器模式：default → brights → turn → default */
        encoder_flag = 1;                /* 触发板载LED快闪反馈，让用户知道操作成功 */

        /* 新增：按下PB5后，串口同步通知电脑当前编码器模式，
           避免之前"切了模式但串口永远显示IDLE"的问题 */
        {
            char send_buf[60];
            switch (led_mode) {
                case led_default_mode:  sprintf(send_buf, "Encoder: IDLE (rotary does nothing)\r\n"); break;
                case led_brights_mode: sprintf(send_buf, "Encoder: BRIGHT (rotary adjusts brightness)\r\n"); break;
                case led_turn_mode:    sprintf(send_buf, "Encoder: MODE (rotary changes LED pattern)\r\n"); break;
            }
            HAL_UART_Transmit(&huart1, (uint8_t*)send_buf, strlen(send_buf), HAL_MAX_DELAY);
        }

        mode_press_flag = 0;
        system_state = 0;                /* 必须重置，否则下一次按键进不了中断 */
    }

    /* ===== 编码器操作成功反馈：板载LED快闪1秒 ===== */
    if (encoder_flag != 0) {
        bright_speed = 300;//编码器操作反馈：300ms翻转一次，比原来慢3倍，看得更清楚 
        blink_timer = HAL_GetTick();
        encoder_flag = 0;
    }

    /* ===== OLED模式刷新：只有 current_mode 变了才重画 ===== */
    if (current_mode != last_mode) {
        OLED_Clear();
        OLED_ShowString(1, 1, "LED CONTROL");
        switch (current_mode) {
            case mode_idle:    OLED_ShowString(2, 1, "Mode: IDLE   "); break;
            case mode_running: OLED_ShowString(2, 1, "Mode: RUNNING"); break;
            case mode_load:    OLED_ShowString(2, 1, "Mode: LOAD   "); break;
            case mode_shark:   OLED_ShowString(2, 1, "Mode: SHARK  "); break;
        }
        last_mode = current_mode;
    }

    /* ===== OLED亮度刷新：只有亮度变了才刷新 ===== */
    if (led_bright != last_bright) {
        char buf[20];
        sprintf(buf, "BRIGHT: %d   ", led_bright);
        OLED_ShowString(3, 1, buf);
        last_bright = led_bright;
    }

    /* ===== OLED编码器模式刷新：第4行显示当前编码器模式 ===== */
    if (led_mode != last_led_mode) {
        switch (led_mode) {
            case led_default_mode:
                OLED_ShowString(4, 1, "ENC: IDLE    ");
                break;
            case led_brights_mode:
                OLED_ShowString(4, 1, "ENC: BRIGHT  ");
                break;
            case led_turn_mode:
                OLED_ShowString(4, 1, "ENC: MODE    ");
                break;
        }
        last_led_mode = led_mode;
    }

    /* ===== 板载按键(PA0/PA1/PA2)：直接切模式 ===== */
    if (key_press != 0) {
        uint8_t key = key_press;
        HAL_Delay(20);
        GPIO_PinState level = GPIO_PIN_SET;
        if (key == 1)       level = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
        else if (key == 2)  level = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
        else if (key == 3)  level = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);

        if (level == GPIO_PIN_RESET) {
            switch (key) {
                case 1: current_mode = mode_running; break;
                case 2: current_mode = mode_load;    break;
                case 3: current_mode = mode_shark;   break;
            }
        }
        system_state = 0;
        key_press = 0;
    }

    /* ===== LED灯珠动画（各模式） ===== */
    switch (current_mode) {
        case mode_idle:
            /* 全灭 */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
            break;

        case mode_running:
            /* 流水灯：PA8 → PA9 → PA10 */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, led_bright);
            HAL_Delay(200);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, led_bright);
            HAL_Delay(200);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, led_bright);
            HAL_Delay(200);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

            HAL_Delay(300);
            break;

        case mode_shark:
            /* 两端 vs 中间：PA8+PA10 ↔ PA9 */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, led_bright);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, led_bright);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
            HAL_Delay(300);

            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, led_bright);
            HAL_Delay(300);
            break;

        case mode_load:
            /* 全亮 */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, led_bright);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, led_bright);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, led_bright);
            break;
    }

    /* ===== 串口命令处理 ===== */
    if (receive_status != 0) {
        Serial_ProcessChar(rx_buffer[0]);
        receive_status = 0;
    }

    /* ===== 串口2秒定时上报 ===== */
    if (HAL_GetTick() - last_uart_tick >= 2000) {
        char buf1[40];
        sprintf(buf1, "Current Brightness = %d\r\n", led_bright);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf1, strlen(buf1), HAL_MAX_DELAY);
        last_uart_tick = HAL_GetTick();
    }
}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7199;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ===== 串口单字符命令处理（主循环中调用） ===== */
void Serial_ProcessChar(uint8_t ch)
{
    switch (ch) {
        case '1': current_mode = mode_running; break;
        case '2': current_mode = mode_load;    break;
        case '3': current_mode = mode_shark;   break;
        case '0': current_mode = mode_idle;    break;
        case '+':
        case 'u':
            if (led_bright < 99) led_bright += 5;
            break;
        case '-':
        case 'd':
            if (led_bright > 0)  led_bright -= 5;
            break;
        default:
            return;  /* 非法命令直接返回 */
    }

    /* 命令处理成功后，回传当前状态到电脑 */
    char send_buf[64];
    sprintf(send_buf, "Mode=%s, Brightness=%d\r\n", mode_name[current_mode], led_bright);
    HAL_UART_Transmit(&huart1, (uint8_t*)send_buf, strlen(send_buf), HAL_MAX_DELAY);
}

/* ===== TIM2溢出中断：每2秒定时上报当前模式 ===== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        char send_buf[50];
         /* 把"只打印LED模式"升级为"同时打印LED模式/编码器模式/亮度值"，
           这样就能看到PB5切换后编码器模式的变化了 */
        {
            char enc_name[10];
            if (led_mode == led_default_mode)       strcpy(enc_name, "IDLE");
            else if (led_mode == led_brights_mode) strcpy(enc_name, "BRIGHT");
            else                                    strcpy(enc_name, "MODE");
            sprintf(send_buf, "LED=%s, Enc=%s, Bright=%d\r\n",
                    mode_name[current_mode], enc_name, led_bright);
        }
        HAL_UART_Transmit(&huart1, (uint8_t*)send_buf, strlen(send_buf), HAL_MAX_DELAY);
    }
}

/* ===== 串口接收完成中断（逐个字节接收） ===== */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
        return;

    uint8_t ch = rx_buffer[0];

    /* 1) 收到回车/换行 → 把缓冲区里累计的数字转成亮度值 */
    if (ch == '\r' || ch == '\n') {
        if (num_idx > 0) {
            num_buf[num_idx] = '\0';
            uint32_t val = 0;
            for (int i = 0; num_buf[i] != '\0'; i++) {
                if (num_buf[i] >= '0' && num_buf[i] <= '9') {
                    val = val * 10 + (num_buf[i] - '0');  /* 字符 → 数字 */
                }
            }
            if (val > 99) val = 99;
            led_bright = (uint8_t)val;

            /* 回传确认 */
            char ack[40];
            sprintf(ack, "OK, Brightness=%d\r\n", led_bright);
            HAL_UART_Transmit(&huart1, (uint8_t*)ack, strlen(ack), HAL_MAX_DELAY);

            num_idx = 0;
        }
    }
    /* 2) 收到数字字符 → 放进缓冲区 */
    else if (ch >= '0' && ch <= '9' && num_idx < 9) {
        num_buf[num_idx++] = ch;
    }
    /* 3) 其他字符 → 当作单字符命令处理 */
    else {
        Serial_ProcessChar(ch);
        num_idx = 0;
    }

    /* 重新开启下一次中断接收 */
    HAL_UART_Receive_IT(&huart1, rx_buffer, 1);
}

/* ===== EXTI外部中断回调（按键按下触发） ===== */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN)
{
  /* PB5编码器按键：最高优先级，直接处理，不受system_state限制 */
    if (GPIO_PIN == GPIO_PIN_5) {
        mode_press_flag = 1;
        return;
    }

    /* 板载按键(PA0/PA1/PA2)：仍然用system_state做软件防抖 */
    if (system_state != 0)
        return;

    if (GPIO_PIN == GPIO_PIN_0) {         /* 按键1(PA0) */
        key_press = 1;
        system_state = 1;
    } else if (GPIO_PIN == GPIO_PIN_1) {  /* 按键2(PA1) */
        key_press = 2;
        system_state = 1;
    } else if (GPIO_PIN == GPIO_PIN_2) {  /* 按键3(PA2) */
        key_press = 3;
        system_state = 1;
    }
}
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