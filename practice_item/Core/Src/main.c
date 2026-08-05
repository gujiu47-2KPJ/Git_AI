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
#include "OLED.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_MAX          999     // PWM 周期最大值
#define RANGE_SIZE       100    // 每段大小 = 999/3

// 每段边界
#define MODE0_MAX        RANGE_SIZE              // 0   ~ 333
#define MODE1_MIN        (RANGE_SIZE + 1)        // 334
#define MODE1_MAX        (RANGE_SIZE * 2)        // 334 ~ 666
#define MODE2_MIN        (RANGE_SIZE * 2 + 1)    // 667
#define MODE2_MAX        PWM_MAX                 // 667 ~ 999

#define SCALE_FACTOR     10  // 999/333 = 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
volatile uint16_t encoder_count = 0; 
volatile uint8_t encoder_changed = 0; 
uint8_t current_mode = 0;
uint8_t last_mode = 0;
uint16_t led_brightness = 0;    

uint8_t breath_dir = 0;
uint8_t flow_step = 0;
uint32_t last_anim_tick = 0 ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void OLED_UpdateDisplay(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{    
     /*硬件消抖*/
     static uint32_t last_tick = 0;//局部变量
     uint32_t now = HAL_GetTick();
     if (now - last_tick < 10) return ;
     last_tick = now;

    if (GPIO_Pin == GPIO_PIN_3) // 检查是否是我们关心的引脚
    {
        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET) // 读取另一个引脚的状态以确定旋转方向
        {
            if(encoder_count < PWM_MAX) encoder_count++; // 顺时针旋转
        }
        else
        {
            if(encoder_count > 0)  encoder_count--; // 逆时针旋转
        }
        encoder_changed = 1 ;
    }
    
}
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(500);   
  OLED_Init(&hi2c1);

  OLED_ShowString(&hi2c1, 0, 0, "OLED TEST");
  OLED_Refresh(&hi2c1);
  HAL_Delay(2000);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
     static uint32_t last_oled_tick = 0;
    uint8_t oled_need_refresh = 0;
    if (HAL_GetTick() - last_oled_tick >= 200)
    {
        last_oled_tick = HAL_GetTick();
        oled_need_refresh = 1;
    }

    if(encoder_changed){
      encoder_changed = 0;
      if(encoder_count <= MODE0_MAX){
        current_mode = 0;
      }else if(encoder_count <= MODE1_MAX && encoder_count>= MODE1_MIN ){
        current_mode = 1;
      } else {
          current_mode = 2;
	  }
          /*------------*/
    if(current_mode == 0){
      led_brightness = encoder_count;
    }else if(current_mode == 1){
      led_brightness = (encoder_count - MODE1_MIN) *SCALE_FACTOR;
    }else {
      led_brightness = (encoder_count - MODE2_MIN) *SCALE_FACTOR;
    }
  }

                  /*------------*/
    uint32_t now = HAL_GetTick();
    if(current_mode  == 0){
      __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,led_brightness);
      __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,led_brightness);
      __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,led_brightness);
    }
    else 
    if (current_mode == 1){
      uint16_t speed = led_brightness + 10;
      if(now - last_anim_tick >= speed ){
        last_anim_tick = now;
        if(breath_dir == 0 ){
          led_brightness += 5;
          if(led_brightness >= PWM_MAX) breath_dir = 1;
        }
        else
        {
        led_brightness-=5;
        if(led_brightness == 0) breath_dir = 0;
        }
      __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,led_brightness);
      __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,led_brightness);
      __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,led_brightness);
      }
    }
    else{
      uint16_t speed = led_brightness + 50;
      if(now - last_anim_tick >= speed)
      {
        last_anim_tick = now;
        flow_step = (flow_step + 1)% 3;
      __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,(flow_step == 0)? PWM_MAX : 0);
      __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,(flow_step == 0)? PWM_MAX : 0);
      __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,(flow_step == 0)? PWM_MAX : 0);
      }
    }
        if (oled_need_refresh)
        OLED_UpdateDisplay();
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_SET);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void OLED_UpdateDisplay(void)   // ← 注意函数名改成和声明一致
{
    char buf[22];
    uint8_t percentage, filled, i;

    OLED_Clear(&hi2c1);

    OLED_ShowString(&hi2c1, 0, 0, "==LED Controller==");

    if (current_mode == 0)
        sprintf(buf, "Mode:0 [Normal]");
    else if (current_mode == 1)
        sprintf(buf, "Mode:1 [Breath]");
    else
        sprintf(buf, "Mode:2 [Flow]");

    OLED_ShowString(&hi2c1, 0, 16, buf);

    sprintf(buf, "Bright: %d", led_brightness);
    OLED_ShowString(&hi2c1, 0, 24, buf);

    percentage = (led_brightness * 100) / PWM_MAX;
    filled = (led_brightness * 14) / PWM_MAX;

    OLED_ShowChar(&hi2c1, 0, 32, '[', 6);
    for (i = 0; i < 14; i++)
    {
        OLED_ShowChar(&hi2c1, 6 + i * 6, 32, (i < filled) ? '=' : ' ', 6);
    }
    OLED_ShowChar(&hi2c1, 6 + 14 * 6, 32, ']', 6);
    sprintf(buf, "%3d%%", percentage);
    OLED_ShowString(&hi2c1, 6 + 15 * 6, 32, buf);

    sprintf(buf, "Enc: %d", encoder_count);
    OLED_ShowString(&hi2c1, 0, 56, buf);

    OLED_Refresh(&hi2c1);
}
/*
void OLED_UpdateDisplay(void){
  char buf[22];
  uint8_t percentage,filled,i;

  OLED_Clear(&hi2c1);

  OLED_ShowString(&hi2c1,0,0,"==LED Controller==");
  if(current_mode == 0){
    sprintf(buf,"Mode:0 [%s]","常亮");
  }
  
    else if(current_mode == 1){
      sprintf(buf,"Mode:1 [%s]","呼吸灯");
    }else 
{sprintf(buf,"Mode:2 [%s]","流水灯");}

      OLED_ShowString(&hi2c1,0,16,buf);

      sprintf(buf,"bright: [%d]",led_brightness);

    OLED_ShowString(&hi2c1,0,24,buf);

    percentage = (led_brightness * 100)/PWM_MAX;
    filled = (led_brightness * 14)/PWM_MAX;
    OLED_ShowChar(&hi2c1,0,32,'[',6);
    for(i = 0;i<14;i++){
      OLED_ShowChar(&hi2c1,6+i*6,32,(i<filled)?'=':' ',6);
      sprintf(buf,"%3d%%",percentage);
      OLED_ShowString(&hi2c1,6+15*6,32,buf);

      sprintf(buf,"Enc:%d",encoder_count);
      OLED_ShowString(&hi2c1,0,56,buf);

      OLED_Refresh(&hi2c1);
    }
  }*/

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