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
#include "Hardware/AHT20.h"
#include "Hardware/BMP280.h"
#include "Hardware/W25QXX.h"
#include "Hardware/OLED.h"
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
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
RTC_HandleTypeDef hrtc;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;
/* USER CODE BEGIN PV */
volatile uint8_t calib_flag = 0;  /* button calib request, set in EXTI ISR */
static uint32_t last_collect_tick = 0;  /* 非阻塞采集间隔计时 */
typedef struct{
  uint32_t timestamp;
  float co2_ppm;
  float co_ppm;
  float nh4_ppm;
  float alcohol_ppm;
  float toluene_ppm;
  float acetone_ppm;
  uint8_t air_quality;
  float temperature_aht;
  float humidity;
  float pressure;
  uint16_t checksum;
}Data_t;
Data_t current_data = {0};
uint8_t pre_flag = 0;
/* Flash 存储管理：使用多扇区 */
#define MAX_FLASH_SECTORS   2047        /* 数据区 2047 扇区，最后 1 扇区留作管理区 */
#define RECORDS_PER_SECTOR  (W25QXX_SECTOR_SIZE / sizeof(Data_t))
#define MAX_TOTAL_RECORDS   (MAX_FLASH_SECTORS * RECORDS_PER_SECTOR)
/* 存储管理区：最后一个扇区末尾固定地址（存 current_sector/records_in_sector） */
#define FLASH_MGMT_ADDR      (MAX_FLASH_SECTORS * W25QXX_SECTOR_SIZE)  /* 专用管理扇区 */
uint32_t write_index = 0;               /* 全局写入索引（0 ~ MAX_TOTAL_RECORDS-1） */
uint32_t current_sector = 0;            /* 当前扇区号 */
uint32_t records_in_sector = 0;         /* 当前扇区已写入记录数 */
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void save_storage_state(void);
void restore_storage_state(void);
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <stdio.h>
#include <string.h>
uint8_t collect_data(){
  char msg[130];
  float pressure_hpa = 1013.25f, temp_bmp = 25.0f, alt_m = 0;
  
  sprintf(msg, "Start collect...\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
  
  current_data.timestamp = HAL_GetTick();
  
  /* 先读取 AHT20 温湿度 */
  sprintf(msg, "Read AHT20...\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
  if(AHT20_Read_Data(&hi2c1, &current_data.temperature_aht, &current_data.humidity) == 0){
    sprintf(msg, "AHT20 Read Failed!\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    current_data.temperature_aht = 25.0f;
    current_data.humidity = 50.0f;
  } else {
    sprintf(msg, "AHT20 Done! T=%.1f, H=%.1f\r\n", current_data.temperature_aht, current_data.humidity);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
  }
  
  /* 再读取 BMP280 气压 */
  sprintf(msg, "Read BMP280...\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
  if(BMP280_GetData(&hi2c1, &pressure_hpa, &temp_bmp, &alt_m) == 0){
    sprintf(msg, "BMP280 Read Failed!\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    current_data.pressure = 1013.25f;
  } else {
    current_data.pressure = pressure_hpa;
    sprintf(msg, "BMP280 Done! P=%.1f\r\n", pressure_hpa);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
  }
  
  /* 用实际环境参数更新 MQ135 补偿 */
  MQ135_SetEnvironment(current_data.temperature_aht, current_data.humidity, current_data.pressure);
  sprintf(msg, "MQ135 Env Updated: T=%.1f, H=%.1f, P=%.1f\r\n", 
          current_data.temperature_aht, current_data.humidity, current_data.pressure);
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
  
  /* 最后读取 MQ135 (此时已使用实际环境参数补偿) */
  MQ135_Data_t mq135_data;
  sprintf(msg, "Read MQ135...\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
  MQ135_GetData(&mq135_data);
  
  /* 打印 MQ135 详细调试信息 */
  sprintf(msg, "MQ135 DBG: ADC=%d, V=%.3fV, Rs=%.2fk, Rs/R0=%.3f\r\n",
          mq135_data.adc_value,
          mq135_data.voltage,
          mq135_data.rs,
          mq135_data.rs_ratio);
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
  
  current_data.co2_ppm = mq135_data.co2_ppm;
  current_data.co_ppm = mq135_data.co_ppm;
  current_data.nh4_ppm = mq135_data.nh4_ppm;
  current_data.alcohol_ppm = mq135_data.alcohol_ppm;
  current_data.toluene_ppm = mq135_data.toluene_ppm;
  current_data.acetone_ppm = mq135_data.acetone_ppm;
  current_data.air_quality = mq135_data.air_quality;
  
  /* 读取 RTC 实时时间 */
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
  
  sprintf(msg, "RTC: 20%02d-%02d-%02d %02d:%02d:%02d\r\n",
          sDate.Year, sDate.Month, sDate.Date,
          sTime.Hours, sTime.Minutes, sTime.Seconds);
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
  
  /* 时间合理性检查: RTC 未初始化或 LSI 掉电复位时, 用开机秒数兜底 */
  if(sDate.Date == 0 || sDate.Month == 0 || (sTime.Hours > 23) || (sTime.Minutes > 59) || (sTime.Seconds > 59)){
    uint32_t boot_sec = HAL_GetTick() / 1000;
    sDate.Year = 0; sDate.Month = 1; sDate.Date = 1;
    sTime.Hours = (boot_sec / 3600) % 24;
    sTime.Minutes = (boot_sec / 60) % 60;
    sTime.Seconds = boot_sec % 60;
  }
  /* 打包时间戳：年(6bit)+月(4bit)+日(5bit)+时(5bit)+分(6bit)+秒(6bit) = 32bit */
  current_data.timestamp = (uint32_t)(sDate.Year & 0x3F) << 26 | (uint32_t)(sDate.Month & 0x0F) << 22 |
                           (uint32_t)(sDate.Date & 0x1F) << 17 | (uint32_t)(sTime.Hours & 0x1F) << 12 |
                           (uint32_t)(sTime.Minutes & 0x3F) << 6 | (uint32_t)(sTime.Seconds & 0x3F);
  
  current_data.checksum = 0;
  sprintf(msg, "Collect Done! CO2=%.1f, CO=%.1f, NH4=%.1f, ALC=%.1f, TOL=%.1f, ACE=%.1f\r\n", 
          current_data.co2_ppm, current_data.co_ppm, current_data.nh4_ppm,
          current_data.alcohol_ppm, current_data.toluene_ppm, current_data.acetone_ppm);
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
  return 1;
}
void save_data_flash(void){
  uint8_t *write_data = (uint8_t*)&current_data;
  uint32_t data_add = current_sector * W25QXX_SECTOR_SIZE + records_in_sector * sizeof(current_data);
  
  /* 检查当前扇区是否满了 */
  if(records_in_sector >= RECORDS_PER_SECTOR){
    current_sector++;
    records_in_sector = 0;
    
    /* 检查是否超出最大扇区数 */
    if(current_sector >= MAX_FLASH_SECTORS){
      char msg[] = "Flash Full! Restarting...\r\n";
      HAL_UART_Transmit(&huart1, (uint8_t*)msg, sizeof(msg)-1, 100);
      current_sector = 0;
    }
    
    /* 擦除新扇区 */
    W25QXX_SectorErase(current_sector * W25QXX_SECTOR_SIZE);
    data_add = current_sector * W25QXX_SECTOR_SIZE;
    
    /* 扇区切换时持久化存储状态（断电恢复用） */
    save_storage_state();
  }
  
  W25QXX_Write(data_add, write_data, sizeof(current_data));
  records_in_sector++;
  write_index++;
}
void read_history_flash(uint32_t record_num){
  Data_t temp_data;
  
  /* 计算记录所在的扇区和扇区内索引 */
  uint32_t sector = record_num / RECORDS_PER_SECTOR;
  uint32_t index_in_sector = record_num % RECORDS_PER_SECTOR;
  
  /* 计算物理地址 */
  uint32_t addr = sector * W25QXX_SECTOR_SIZE + index_in_sector * sizeof(current_data);
  
  W25QXX_Read(addr, (uint8_t*)&temp_data, sizeof(current_data));
  
  char msg[200];
  uint32_t ts = temp_data.timestamp;
  uint8_t year = (ts >> 26) & 0x3F;
  uint8_t month = (ts >> 22) & 0x0F;
  uint8_t date = (ts >> 17) & 0x1F;
  uint8_t hours = (ts >> 12) & 0x1F;
  uint8_t minutes = (ts >> 6) & 0x3F;
  uint8_t seconds = ts & 0x3F;
  
  sprintf(msg, "[%lu] 20%02d-%02d-%02d %02d:%02d:%02d | CO2=%.1f CO=%.1f NH4=%.1f ALC=%.1f TOL=%.1f ACE=%.1f | T=%.1f H=%.1f P=%.1f\r\n",
           (unsigned long)record_num, year, month, date, hours, minutes, seconds,
           temp_data.co2_ppm,
           temp_data.co_ppm,
           temp_data.nh4_ppm,
           temp_data.alcohol_ppm,
           temp_data.toluene_ppm,
           temp_data.acetone_ppm,
           temp_data.temperature_aht,
           temp_data.humidity,
           temp_data.pressure);
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
}
/* 读取最近 N 条记录 */
void read_last_records(uint32_t num){
  char msg[50];
  
  if(num > write_index){
    num = write_index;
  }
  
  sprintf(msg, "=== 读取最近 %lu 条记录 ===\r\n", (unsigned long)num);
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
  
  /* 从最新的数据往前读 */
  uint32_t start = (write_index >= num) ? (write_index - num) : 0;
  for(uint32_t i = start; i < write_index; i++){
    read_history_flash(i);
  }
  
  sprintf(msg, "=== 读取完成 ===\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
}
/* 读取全部记录（谨慎使用，数据量大） */
void read_all_records(void){
  char msg[50];
  sprintf(msg, "=== 读取全部 %lu 条记录 ===\r\n", (unsigned long)write_index);
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
  
  for(uint32_t i = 0; i < write_index; i++){
    read_history_flash(i);
  }
  
  sprintf(msg, "=== 读取完成 ===\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
}
/* 保存存储状态到 Flash 管理区（扇区切换时调用） */
void save_storage_state(void){
    uint8_t buf[8];
    buf[0] = (uint8_t)(current_sector & 0xFF);
    buf[1] = (uint8_t)((current_sector >> 8) & 0xFF);
    buf[2] = (uint8_t)(records_in_sector & 0xFF);
    buf[3] = 0x5A;  /* 魔数，校验有效性 */
    buf[4] = 0xA5;
    buf[5] = 0x00; buf[6] = 0x00; buf[7] = 0x00;
    W25QXX_SectorErase(FLASH_MGMT_ADDR);
    W25QXX_Write(FLASH_MGMT_ADDR, buf, 8);
}
/* 上电恢复存储状态 */
void restore_storage_state(void){
    uint8_t buf[8] = {0};
    W25QXX_Read(FLASH_MGMT_ADDR, buf, 8);
    if(buf[3] == 0x5A && buf[4] == 0xA5){
        current_sector = buf[0] | (buf[1] << 8);
        records_in_sector = buf[2];
        write_index = current_sector * RECORDS_PER_SECTOR + records_in_sector;
    }
    /* 无有效魔数 → 保持 0，从头开始 */
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  char msg[50];
  sprintf(msg, "Init MQ135...\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
  MQ135_Init(&hadc1);
  
  sprintf(msg, "Init AHT20...\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
  if(AHT20_Init(&hi2c1)){
    sprintf(msg, "AHT20 Init OK!\r\n");
  } else {
    sprintf(msg, "AHT20 Init FAILED!\r\n");
  }
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
  
  sprintf(msg, "Init BMP280...\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
  BMP280_Init(&hi2c1);
  
  sprintf(msg, "Init W25QXX...\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
  uint8_t flash_ok = W25QXX_Init();
  uint16_t flash_id = W25QXX_ReadID();
  
  if(flash_ok == W25QXX_OK){
    sprintf(msg, "W25QXX OK! ID=0x%04X\r\n", flash_id);
    switch(flash_id){
      case 0xEF13: sprintf(msg, "W25Q80 (1MB)\r\n"); break;
      case 0xEF14: sprintf(msg, "W25Q16 (2MB)\r\n"); break;
      case 0xEF15: sprintf(msg, "W25Q32 (4MB)\r\n"); break;
      case 0xEF16: sprintf(msg, "W25Q64 (8MB)\r\n"); break;
      case 0xEF17: sprintf(msg, "W25Q128 (16MB)\r\n"); break;
      default: sprintf(msg, "Unknown Flash (0x%04X)\r\n", flash_id); break;
    }
  } else {
    sprintf(msg, "W25QXX FAILED! ID=0x%04X\r\n", flash_id);
  }
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
  
  sprintf(msg, "Init OLED...\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
  OLED_Init(&hi2c1);
  
  /* 恢复上次存储位置（断电续传） */
  restore_storage_state();
  pre_flag = 1;
  sprintf(msg, "All Init Done!\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 始终检查校准按钮标志 */
    if(calib_flag)
    {
      calib_flag = 0;
      OLED_DisplayOn();
      OLED_Clear(&hi2c1);
      OLED_ShowString(&hi2c1, 0, 0, "Calibrating...");
      OLED_Refresh(&hi2c1);
      MQ135_CalibrateRZero();
      OLED_Clear(&hi2c1);
      OLED_ShowString(&hi2c1, 0, 0, "Calib Done!");
      OLED_Refresh(&hi2c1);
      HAL_Delay(1000);
      {
        char cm[48];
        sprintf(cm, "Button Calib Done! R0=%.1f kOhm\r\n", MQ135_GetRZero());
        HAL_UART_Transmit(&huart1, (uint8_t*)cm, strlen(cm), 100);
      }
      OLED_DisplayOff();
    }
    /* 30 秒采集间隔（非阻塞，首次立即采集） */
    if (HAL_GetTick() - last_collect_tick >= 30000 || last_collect_tick == 0)
    {
      collect_data();
      save_data_flash();
    
      /* OLED 轮播显示：第1页-气体数据，第2页-环境数据+时间 */
      static uint8_t page = 0;
      OLED_DisplayOn();       /* 采集时点亮 */
      OLED_Clear(&hi2c1);
    
      /* 读取当前 RTC 时间用于显示 */
      RTC_TimeTypeDef dispTime;
      RTC_DateTypeDef dispDate;
      HAL_RTC_GetTime(&hrtc, &dispTime, RTC_FORMAT_BIN);
      HAL_RTC_GetDate(&hrtc, &dispDate, RTC_FORMAT_BIN);
    
      if(page == 0){
        /* 第1页：气体数据 + 时间 */
        char timeStr[20];
        sprintf(timeStr, "%02d:%02d:%02d", dispTime.Hours, dispTime.Minutes, dispTime.Seconds);
        OLED_ShowString(&hi2c1, 64, 0, timeStr);
      
        OLED_ShowString(&hi2c1, 0, 8, "CO2:");
        OLED_ShowFloat(&hi2c1, 32, 8, current_data.co2_ppm, 1);
      
        OLED_ShowString(&hi2c1, 0, 16, "CO:");
        OLED_ShowFloat(&hi2c1, 32, 16, current_data.co_ppm, 1);
      
        OLED_ShowString(&hi2c1, 0, 24, "NH4:");
        OLED_ShowFloat(&hi2c1, 32, 24, current_data.nh4_ppm, 1);
      
        OLED_ShowString(&hi2c1, 0, 32, "ALC:");
        OLED_ShowFloat(&hi2c1, 32, 32, current_data.alcohol_ppm, 1);
      
        OLED_ShowString(&hi2c1, 0, 40, "TOL:");
        OLED_ShowFloat(&hi2c1, 32, 40, current_data.toluene_ppm, 1);
      
        OLED_ShowString(&hi2c1, 0, 48, "ACE:");
        OLED_ShowFloat(&hi2c1, 32, 48, current_data.acetone_ppm, 1);
      } else {
        /* 第2页：环境数据 + 日期 */
        char dateStr[20];
        sprintf(dateStr, "20%02d-%02d-%02d", dispDate.Year, dispDate.Month, dispDate.Date);
        OLED_ShowString(&hi2c1, 32, 0, dateStr);
      
        OLED_ShowString(&hi2c1, 0, 8, "T:");
        OLED_ShowFloat(&hi2c1, 24, 8, current_data.temperature_aht, 1);
        OLED_ShowString(&hi2c1, 80, 8, "C");
      
        OLED_ShowString(&hi2c1, 0, 16, "H:");
        OLED_ShowFloat(&hi2c1, 24, 16, current_data.humidity, 1);
        OLED_ShowString(&hi2c1, 80, 16, "%");
      
        OLED_ShowString(&hi2c1, 0, 24, "P:");
        OLED_ShowFloat(&hi2c1, 24, 24, current_data.pressure, 1);
        OLED_ShowString(&hi2c1, 80, 24, "hPa");
      
        OLED_ShowString(&hi2c1, 0, 32, "AQ:");
        OLED_ShowInt(&hi2c1, 32, 32, current_data.air_quality);
      }
    
      OLED_Refresh(&hi2c1);
      HAL_Delay(2000);        /* 亮屏 2 秒，让用户看清 */
      OLED_DisplayOff();      /* 显示后关屏省电 */
      page = (page + 1) % 2;  /* 切换页面 */
    
      last_collect_tick = HAL_GetTick();
    }
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    uint8_t num = 0;
    if(HAL_UART_Receive(&huart1, &num, 1, 100) == HAL_OK){
      switch(num){
        case '1': {
          char msg1[] = "正在校准 MQ135...\r\n";
          HAL_UART_Transmit(&huart1, (uint8_t*)msg1, sizeof(msg1)-1, 100);
          MQ135_CalibrateRZero();
          char msg2[] = "校准完成！\r\n";
          HAL_UART_Transmit(&huart1, (uint8_t*)msg2, sizeof(msg2)-1, 100);
          break;
        }
        case '2': {
          char msg[] = "读取最近 10 条记录\r\n";
          HAL_UART_Transmit(&huart1, (uint8_t*)msg, sizeof(msg)-1, 100);
          read_last_records(10);
          break;
        }
        case '3': {
          char msg[] = "Dump ALL records (large!)\r\n";
          HAL_UART_Transmit(&huart1, (uint8_t*)msg, sizeof(msg)-1, 100);
          read_all_records();
          break;
        }
        case '4': {
          /* 清除近期数据: 擦除当前数据扇区, 回退到扇区开头 */
          char msg[] = "Erase recent sector...";
          HAL_UART_Transmit(&huart1, (uint8_t*)msg, sizeof(msg)-1, 100);
          W25QXX_SectorErase(current_sector * W25QXX_SECTOR_SIZE);
          records_in_sector = 0;
          write_index = current_sector * RECORDS_PER_SECTOR;
          save_storage_state();
          char msg2[] = "Recent sector cleared.";
          HAL_UART_Transmit(&huart1, (uint8_t*)msg2, sizeof(msg2)-1, 100);
          break;
        }
        case '5': {
          /* 清除所有数据: 整片擦除 + 重置索引 */
          char msg[] = "Erase ALL flash...";
          HAL_UART_Transmit(&huart1, (uint8_t*)msg, sizeof(msg)-1, 100);
          W25QXX_ChipErase();
          write_index = 0;
          current_sector = 0;
          records_in_sector = 0;
          save_storage_state();
          char msg2[] = "All data cleared.";
          HAL_UART_Transmit(&huart1, (uint8_t*)msg2, sizeof(msg2)-1, 100);
          break;
        }
        default: {
          char msg[] = "CMD: 1=cal 2=last10 3=dumpall 4=erase_recent 5=erase_all";
          HAL_UART_Transmit(&huart1, (uint8_t*)msg, sizeof(msg)-1, 100);
          break;
        }
      }
    }
    HAL_Delay(100);  /* 每 100ms 循环一次，及时响应按钮 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{
  /* USER CODE BEGIN RTC_Init 0 */
  /* USER CODE END RTC_Init 0 */
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};
  /* USER CODE BEGIN RTC_Init 1 */
  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Check_RTC_BKUP */
  /* 使能备份域访问（STM32F1 必须） */
  HAL_PWR_EnableBkUpAccess();
  
  /* 检查是否已经初始化过 RTC（通过备份寄存器 BKP_DR1 标记 + 时间合理性） */
  RTC_DateTypeDef chkDate;
  uint32_t bkup = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
  HAL_RTC_GetDate(&hrtc, &chkDate, RTC_FORMAT_BIN);
  if(bkup != 0x32F2 || chkDate.Date == 0 || chkDate.Year < 20){
    /* 第一次启动，设置初始时间 */
    sTime.Hours = 0x23;
    sTime.Minutes = 0x28;
    sTime.Seconds = 0x0;
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
    {
      Error_Handler();
    }
    DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
    DateToUpdate.Month = RTC_MONTH_AUGUST;
    DateToUpdate.Date = 0x3;
    DateToUpdate.Year = 0x26;
    if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
    {
      Error_Handler();
    }
    /* 写入标记，下次启动不再重新设置时间 */
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2);
  }
  /* USER CODE END Check_RTC_BKUP */
  /* USER CODE BEGIN RTC_Init 2 */
  /* USER CODE END RTC_Init 2 */
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
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
/* EXTI button callback: set flag, main loop does the work */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_1)
  {
    calib_flag = 1;  /* 只设标志位，不做其他操作 */
  }
}