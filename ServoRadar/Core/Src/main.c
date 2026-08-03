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
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f1xx_hal_gpio.h"
#include "Hardware/MPU6050.h"   /* MPU6050 driver */
#include "Hardware/OLED.h"      /* OLED driver */
#include "Hardware/HCSR04.h"    /* HC-SR04 ultrasonic driver */
#include "Hardware/Servo.h"     /* Servo motor driver */
#include "Hardware/OLED_UI.h"   /* V2 Vivid radar UI (explicit names+units) */
#include "Hardware/W25QXX.h"    /* 外扩Flash: 用它提升OLED显示效果!!! */
/* ---------------- W25QXX Flash 地址映射 (用于OLED显示效果增强) ----------------
 * W25Q80=1MB, 按扇区(4KB)划分, 各功能地址固定:
 *   0x000000 ~ 0x000FFF  [扇区0]   : MPU校准+显示配置(已预留)
 *   0x001000 ~ 0x001FFF  [扇区1]   : 最后1帧雷达障碍物图radar_history[181]掉电保存
 *   0x002000 ~ 0x002FFF  [扇区2]   : 开机画面+状态图标+大字体位图(可预烧录)
 *   0x003000 ~ 0x003FFF  [扇区3]   : 180个角度的cos/sin预计算表(float)
 *   0x004000 ~ 0x007FFF  [扇区4~7] : 3圈历史雷达扫描数据(时间衰减拖影用)
 * -------------------------------------------------------------------------- */
#define FLASH_ADDR_LAST_FRAME    0x001000
#define FLASH_ADDR_ICON_FONT     0x002000
#define FLASH_ADDR_ANGLE_TABLE   0x003000
#define FLASH_ADDR_HIST_RING0    0x004000  /* 第0圈 (最新) */
#define FLASH_ADDR_HIST_RING1    0x005000  /* 第1圈 (中) */
#define FLASH_ADDR_HIST_RING2    0x006000  /* 第2圈 (最旧) */

/* ---------- Fix: Keil ARMCC 兼容补丁 ----------
 * ARMCC 默认 math.h 不定义 M_PI (非 C 标准宏，需要手动定义
 * 同时 _USE_MATH_DEFINES 在部分编译器支持，但直接定义更稳妥 */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* strlen.h */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    WAIT_FRAME_H,    // 等待 0xAA
    WAIT_FRAME_L,    // 等待 0x55
    WAIT_DATA,       // 接收数据字节
    WAIT_TAIL        // 等待帧尾 0x0D 0x0A
} ParseState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* ========== ESP32 气象数据 ========== */
float env_temp_esp32 = 20.0f;   /* ESP32气象站气温(°C)，更新慢(90s)但准确 */
float env_hum_esp32  = 50.0f;   /* ESP32气象站湿度(%)，更新慢但准确 */
uint8_t env_data_valid = 0;     /* 标志：1=刚收到新ESP32数据，处理后清0 */

/* ========== MPU6050 芯片温度 ========== */
float mpu_chip_temp = 25.0f;    /* 扫描头MPU6050内部温度(°C)，实时性高但比环境高 */

/* ========== 温度融合物理模型（核心新增） ========== */
/**
  * 物理模型：T_chip = T_air_local + ΔT_self_heating
  *   ΔT_self_heating = 芯片自加热温升，稳定工作后恒定
  *   每次收到ESP32时校准：ΔT_calib = T_chip - T_esp
  *   两次ESP32更新之间，实时估算局部温度：T_local_est = T_chip - ΔT_calib
  */
float delta_T_selfheat = 10.0f;  /* 【校准后】芯片自加热温升 ΔT_calib，默认10°C */
float T_local_estimated = 20.0f;/* 【MPU实时估算】局部环境温度 = T_chip - ΔT_calib */
float T_fused_final   = 20.0f;  /* 【最终融合】用于声速计算的温度：ESP32(70%) + 估算(30%) */

/* 历史温升滑动平均：每次收到ESP32校准后加入4点平均，抑制偶然校准误差 */
#define HEAT_RISE_AVG_WIN 4
float heat_rise_hist[HEAT_RISE_AVG_WIN] = {10.0f,10.0f,10.0f,10.0f};
uint8_t heat_rise_idx = 0;
uint8_t heat_rise_calibrated = 0; /* 0=从未校准过，1=至少校准过1次 */

/* ========== Fix: 恢复 HCSR04.c 需要的全局变量（extern 引用） ========== */
/* estimated_ambient = 最终融合温度 T_fused_final 的别名，为兼容 HCSR04.c 声速补偿 */
float estimated_ambient = 20.0f;
/* env_temp_esp32 和 env_hum_esp32 已在上方定义，HCSR04.c 也会 extern 引用 */

/* ==================== 【雷达扫描历史缓存 - 解决"扫过去没点"的核心根因】 ====================
 * 之前的Bug：每帧 OLED_Clear 后只画【当前角度】1个点，点显示80ms(1帧)就消失，肉眼以为没画！
 * 修复：每个角度(0~180°)存最近1次的测距值+时间戳，每帧把【所有角度】的历史点全重绘
 *        → 扫过的扇形区域拖影保留，和真雷达一模一样，障碍物一眼可见 */
#define RADAR_HIST_SIZE  181   /* 对应角度 0~180 */
typedef struct {
    uint16_t dist_mm;          /* 距离 mm, 0xFFFF=invalid */
    uint32_t last_seen_ms;     /* 上次扫描到的时间戳(HAL tick)，用于过期清理 */
} RadarHist_t;
#define RADAR_HIST_EXPIRE_MS  15000  /* 15秒没再扫到的旧点自动清掉(避免画面脏) */

/* ================ 【Flash驱动OLED显示效果 - 全局变量】 ================
 * 1) 角度预计算表: cos_val[a] / sin_val[a] = 角度 a(0~180) 的 cos/sin 值
 *    从Flash读，每帧不用算三角函数 → CPU占用减少80% → 帧率翻倍
 * 2) 3圈历史雷达扫描 (环形缓冲区, Flash存掉电)
 *    每扫完一圈 RING2→覆盖, RING1→RING2, RING0→RING1, 新圈→RING0
 *    显示时: RING2(最旧)=极小点, RING1(中)=小点, RING0(最新)=十字, 当前角度=大方块
 *    → 和真雷达一样的时间衰减拖影效果!
 * ==================================================================== */
float flash_cos_table[181];   /* 索引0~180 = 角度0°~180° cos */
float flash_sin_table[181];   /* 索引0~180 = 角度0°~180° sin */
RadarHist_t radar_ring0[RADAR_HIST_SIZE];  /* 最新圈 */
RadarHist_t radar_ring1[RADAR_HIST_SIZE];  /* 中旧圈 */
RadarHist_t radar_ring2[RADAR_HIST_SIZE];  /* 最旧圈 */
uint8_t  flash_ok = 0;       /* Flash初始化OK标识，0=不读写防止乱操作 */
uint32_t last_flash_save_ms = 0;  /* 上次写回Flash的时间戳，节流防止写太勤坏块 */

RadarHist_t radar_history[RADAR_HIST_SIZE];  /* 索引0=0°，索引180=180° */

/* ========== 超声波距离 ========== */
uint16_t dist_scan = 0;  /* 扫描头超声波距离(mm)，随舵机旋转变化 */
uint16_t dist_base = 0;  /* 面包板基准超声波距离(mm)，固定位置作为参考 */

/* ========== MPU6050 数据 ========== */
MPU6050_Data MPU6050_SCAN;   /* 扫描头MPU（I2C1，随舵机旋转） */
MPU6050_Data MPU6050_BREAD;  /* 面包板基准MPU（I2C2，固定） */

/* ========== 【新增V2】舵机机械状态 + 可信度模型 ========== */
/**
  * 物理背景：舵机+扫描头是热熔胶+杜邦线临时固定，每次换向瞬间会回弹、抖动
  * 模型：梯形可信度函数：
  *   · 方向变化的0~100ms      → conf = 0.1  (机械回弹期，数据极不可靠)
  *   · 方向变化100~300ms过渡 → conf 线性从0.1→1.0
  *   · 方向变化>300ms       → conf = 1.0  (稳定匀速期)
  */
int8_t   servo_last_dir = 1;          /* 上次扫描方向，用来检测换向 */
uint32_t servo_dir_change_ms = 0;     /* 【记录】最后一次换向的绝对时间戳(HAL_GetTick) */
float    current_servo_conf = 0.5f;   /* 每次主循环计算一次，供超声波和OLED显示用 */
float    current_vib_level = 0.0f;    /* 双MPU差分残余振动幅度(°/s)，供超声波+OLED用 */

/* ========== ESP32 数据帧解析状态机 ========== */
static ParseState state = WAIT_FRAME_H;
static uint8_t rx_buf[9];
static uint8_t rx_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  发送数据到VOFA+（Flyweight协议：纯数字逗号分隔，\r\n结尾）
  *         格式：扫描Roll,扫描Pitch,基准Roll,基准Pitch,扫描距离mm,基准距离mm,环境温度°C,环境湿度%
  *         每一行都是8个数字，VOFA+中选择"FireWater"模式解析
  */
void vofa_send_all(MPU6050_Data *scan, MPU6050_Data *bread,
                   uint16_t scan_mm, uint16_t base_mm)
{
    char tx_buf[128];
    float s_dist = (scan_mm == HCSR04_DIST_INVALID) ? -1.0f : (float)scan_mm;
    float b_dist = (base_mm == HCSR04_DIST_INVALID) ? -1.0f : (float)base_mm;
    
    snprintf(tx_buf, sizeof(tx_buf), "%.2f,%.2f,%.2f,%.2f,%.0f,%.0f,%.1f,%.1f\r\n",
             scan->roll,  scan->pitch,
             bread->roll, bread->pitch,
             s_dist, b_dist,
             env_temp_esp32, env_hum_esp32);
    HAL_UART_Transmit(&huart1, (uint8_t*)tx_buf, strlen(tx_buf), 100);
}

/**
  * @brief  从 USART2 硬件串口读取一个字节（非阻塞，含错误处理）
  * @retval 接收到的字节(0~254)，如果没有数据或错误返回 0xFF
  * @note   直接读 USART2 寄存器，比 HAL_UART_Receive 高效不阻塞
  *         必须处理 ORE(溢出) 和 FE(帧错误)，否则会卡死接收
  */
static inline uint8_t usart2_read_byte(void)
{
    uint32_t sr = USART2->SR;
    
    /* 【关键】溢出错误 ORE(第3位)：新数据来之前旧数据没读走
     * 如果不清除，ORE 会一直置位，RXNE 不再更新，永远收不到新数据 */
    if (sr & (1 << 3)) {
        /* 清 ORE: 先读 SR 再读 DR 即可清除 */
        volatile uint8_t dummy = (uint8_t)(USART2->DR & 0xFF);
        (void)dummy;  /* 防止编译器警告 */
        return 0xFF;
    }
    
    /* 帧错误 FE(第1位): 停止位不对，可能是波特率不匹配或干扰 */
    if (sr & (1 << 1)) {
        volatile uint8_t dummy = (uint8_t)(USART2->DR & 0xFF);
        (void)dummy;
        return 0xFF;
    }
    
    /* RXNE(第5位) = 1: 接收缓冲区有数据 */
    if (sr & (1 << 5)) {
        /* 读 DR 寄存器会自动清除 RXNE 标志 */
        return (uint8_t)(USART2->DR & 0xFF);
    }
    return 0xFF;  /* 无数据 */
}

/**
  * @brief  解析 ESP32 发来的二进制帧数据
  * 
  * 帧格式：| 0xAA | 0x55 | TEMP_H | TEMP_L | HUM_H | HUM_L | CHK | 0x0D | 0x0A |
  * 说明：
  *   - 帧头：0xAA 0x55
  *   - TEMP：温度 x100（int16，大端序），例 25.50°C -> 2550 = 0x09F6
  *   - HUM：湿度 x100（int16，大端序），例 65.20% -> 6520 = 0x1978
  *   - CHK：0xAA^0x55^TEMP_H^TEMP_L^HUM_H^HUM_L
  *   - 帧尾：0x0D 0x0A (\r\n)
  * 
  * 使用 USART2 硬件串口（PA3=RX），稳定可靠，波特率9600
  */
void parse_esp32_data(void)
{
    uint8_t byte;
    
    /* 循环读取 USART2 所有已接收字节，避免硬件溢出 */
    while (1) {
        byte = usart2_read_byte();
        if (byte == 0xFF) break;  /* 无新数据，退出循环 */
        
        switch (state) {
        case WAIT_FRAME_H:
            if (byte == 0xAA) state = WAIT_FRAME_L;
            break;
            
        case WAIT_FRAME_L:
            if (byte == 0x55) {
                state = WAIT_DATA;
                rx_index = 0;
            } else {
                state = WAIT_FRAME_H;
            }
            break;
            
        case WAIT_DATA:
            rx_buf[rx_index++] = byte;
            /* 收满7字节：TEMP_H TEMP_L HUM_H HUM_L CHK 0x0D 0x0A */
            if (rx_index >= 7) {
                state = WAIT_FRAME_H;  /* 无论成功失败，下一次从帧头开始找 */
                
                /* 检查帧尾 */
                if ((rx_buf[5] == 0x0D) && (rx_buf[6] == 0x0A)) {
                    /* 计算校验：0xAA ^ 0x55 ^ 4个数据字节 */
                    uint8_t chk_calc = 0xAA ^ 0x55 ^ rx_buf[0] ^ rx_buf[1] ^ rx_buf[2] ^ rx_buf[3];
                    if (chk_calc == rx_buf[4]) {
                        /* 校验通过，更新温湿度 */
                        int16_t temp_raw = (int16_t)((rx_buf[0] << 8) | rx_buf[1]);
                        int16_t hum_raw  = (int16_t)((rx_buf[2] << 8) | rx_buf[3]);
                        env_temp_esp32 = temp_raw / 100.0f;
                        env_hum_esp32  = hum_raw  / 100.0f;
                        env_data_valid = 1;
                        
                        /* 输出成功信息到电脑串口 */
                        char dbg2[80];
                        snprintf(dbg2, sizeof(dbg2),
                            "[ESP32] 收到数据: 温度=%.1f°C 湿度=%.1f%%\r\n",
                            env_temp_esp32, env_hum_esp32);
                        HAL_UART_Transmit(&huart1, (uint8_t*)dbg2, strlen(dbg2), 100);
                    } else {
                        char dbg2[80];
                        snprintf(dbg2, sizeof(dbg2),
                            "[ESP32] 校验错误: 计算=%02X 收到=%02X\r\n",
                            chk_calc, rx_buf[4]);
                        HAL_UART_Transmit(&huart1, (uint8_t*)dbg2, strlen(dbg2), 100);
                    }
                } else {
                    char dbg2[80];
                    snprintf(dbg2, sizeof(dbg2),
                        "[ESP32] 帧尾错误: %02X %02X (应为 0D 0A)\r\n",
                        rx_buf[5], rx_buf[6]);
                    HAL_UART_Transmit(&huart1, (uint8_t*)dbg2, strlen(dbg2), 100);
                }
            }
            break;
            
        default:
            state = WAIT_FRAME_H;
            break;
        }
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
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Fix: 所有串口输出字符串改为英文，避免Keil ARMCC中文编码错误 */
  HAL_UART_Transmit(&huart1, (uint8_t*)"=== Servo Radar System Boot ===\r\n", 32, 100);
  HAL_UART_Transmit(&huart1, (uint8_t*)"UART1(PC): PA9/PA10 115200bps\r\n", 32, 100);
  HAL_UART_Transmit(&huart1, (uint8_t*)"UART2(ESP32): PA3(RX) 9600bps\r\n", 32, 100);

  MPU6050_Init(&hi2c1);
  uint8_t ret = MPU6050_ReadID(&hi2c1);
  MPU6050_Init(&hi2c2);
  uint8_t ret2 = MPU6050_ReadID(&hi2c2);

  HCSR04_Init();
  Servo_Init();  /* Servo init: PWM to PA0 */
  HAL_UART_Transmit(&huart1, (uint8_t*)"Servo Init OK, pos 90deg\r\n", 26, 100);

  OLED_Init(&hi2c1);
  OLED_Clear(&hi2c1);
  char buf[32];
  OLED_ShowString(&hi2c1, 0, 0, "Starting.");
  snprintf(buf, sizeof(buf), "MPU:S%02X B%02X", ret, ret2);
  OLED_ShowString(&hi2c1, 0, 16, buf);

  /* ============== 【Flash驱动OLED显示效果 - 初始化阶段】 ==============
   * 做4件事：①读Flash ID验证硬件 ②预计算/读取cos/sin角度表 ③恢复上次雷达图 ④恢复3圈历史拖影
   * 任何一步失败不会崩，只是OLED显示效果和原来一样(降级模式) */
  {
    OLED_ShowString(&hi2c1, 0, 32, "Flash INIT...    ");
    OLED_Refresh(&hi2c1);
    uint8_t fret = W25QXX_Init();
    if (fret == W25QXX_OK) {
      flash_ok = 1;
      uint16_t fid = W25QXX_ReadID();
      snprintf(buf, sizeof(buf), "Flash OK ID:%04X ", fid);
      OLED_ShowString(&hi2c1, 0, 32, buf);
      OLED_Refresh(&hi2c1);

      /* -------- 2. 角度表: 存在就直接读, 不存在就现场计算再写Flash(只算1次, 终身受用) -------- */
      uint8_t magic[4];
      W25QXX_Read(FLASH_ADDR_ANGLE_TABLE, magic, 4);
      if (magic[0]=='A' && magic[1]=='G' && magic[2]=='L' && magic[3]=='E') {
        /* Flash里已经有预计算表 → 直接读, 1ms搞定! (不用再算181次cosf/sinf) */
        W25QXX_Read(FLASH_ADDR_ANGLE_TABLE + 4, (uint8_t*)flash_cos_table, sizeof(float)*181);
        W25QXX_Read(FLASH_ADDR_ANGLE_TABLE + 4 + sizeof(float)*181, (uint8_t*)flash_sin_table, sizeof(float)*181);
      } else {
        /* 第一次用 → 现场算181个角度的cos/sin, 存Flash, 下次开机就不用算了 */
        OLED_ShowString(&hi2c1, 0, 40, "Build ANG table...");
        OLED_Refresh(&hi2c1);
        for (uint16_t a = 0; a <= 180; a++) {
          /* 和原来单帧换算完全一致: OLED角度 = 180°-舵机角度, 弧度制 */
          float rad = (180.0f - (float)a) * ((float)M_PI / 180.0f);
          flash_cos_table[a] = cosf(rad);
          flash_sin_table[a] = sinf(rad);
        }
        /* 写Flash: 前4字节写MAGIC "AGLE" 标记这是有效表, 下次不用重算 */
        W25QXX_SectorErase(FLASH_ADDR_ANGLE_TABLE);
        uint8_t m[4] = {'A','G','L','E'};
        W25QXX_Write(FLASH_ADDR_ANGLE_TABLE, m, 4);
        W25QXX_Write(FLASH_ADDR_ANGLE_TABLE + 4, (uint8_t*)flash_cos_table, sizeof(float)*181);
        W25QXX_Write(FLASH_ADDR_ANGLE_TABLE + 4 + sizeof(float)*181, (uint8_t*)flash_sin_table, sizeof(float)*181);
      }

      /* -------- 3. 恢复上次断电前的雷达障碍物图 → 开机OLED直接显示扇形, 不用重新扫! -------- */
      magic[0]=0; magic[1]=0; magic[2]=0; magic[3]=0;
      W25QXX_Read(FLASH_ADDR_LAST_FRAME, magic, 4);
      if (magic[0]=='R' && magic[1]=='D' && magic[2]=='R' && magic[3]=='!') {
        W25QXX_Read(FLASH_ADDR_LAST_FRAME + 4, (uint8_t*)radar_history, sizeof(RadarHist_t)*RADAR_HIST_SIZE);
      }
      /* -------- 4. 恢复3圈历史 → 时间衰减拖影效果, 开机和断电前显示一模一样 -------- */
      magic[0]=0; magic[1]=0; magic[2]=0;
      W25QXX_Read(FLASH_ADDR_HIST_RING0, magic, 3);
      if (magic[0]=='R' && magic[1]=='0' && magic[2]=='!') {
        W25QXX_Read(FLASH_ADDR_HIST_RING0 + 3, (uint8_t*)radar_ring0, sizeof(RadarHist_t)*RADAR_HIST_SIZE);
      }
      W25QXX_Read(FLASH_ADDR_HIST_RING1, magic, 3);
      if (magic[0]=='R' && magic[1]=='1' && magic[2]=='!') {
        W25QXX_Read(FLASH_ADDR_HIST_RING1 + 3, (uint8_t*)radar_ring1, sizeof(RadarHist_t)*RADAR_HIST_SIZE);
      }
      W25QXX_Read(FLASH_ADDR_HIST_RING2, magic, 3);
      if (magic[0]=='R' && magic[1]=='2' && magic[2]=='!') {
        W25QXX_Read(FLASH_ADDR_HIST_RING2 + 3, (uint8_t*)radar_ring2, sizeof(RadarHist_t)*RADAR_HIST_SIZE);
      }

      /* 清除临时提示行残留 */
      OLED_ShowString(&hi2c1, 0, 40, "                ");
    } else {
      /* Flash硬件没接好/读不到ID → 降级运行, OLED显示效果和原来一样, 不会崩 */
      flash_ok = 0;
      OLED_ShowString(&hi2c1, 0, 32, "Flash FAIL Deg.");
    }
    OLED_Refresh(&hi2c1);
  }
  OLED_Refresh(&hi2c1);
  HAL_UART_Transmit(&huart1, (uint8_t*)"MPU6050 Init OK\r\n", 18, 100);
  HAL_Delay(1500);

  /* ===== 自动零偏校准（100次取均值） ===== */
  MPU6050_SCAN.roll_offset = 0; MPU6050_SCAN.pitch_offset = 0;
  MPU6050_BREAD.roll_offset = 0; MPU6050_BREAD.pitch_offset = 0;
  OLED_Clear(&hi2c1);
  OLED_ShowString(&hi2c1, 0, 0, "Calibrating...");
  OLED_ShowString(&hi2c1, 0, 16, "Keep still!");
  OLED_Refresh(&hi2c1);

  float rs=0, ps=0, rb=0, pb=0;
  MPU6050_Data cs, cb;
  /* 【关键】必须将临时结构体的offset清零，否则栈上残留的随机值会污染校准结果 */
  cs.roll_offset = 0; cs.pitch_offset = 0;
  cb.roll_offset = 0; cb.pitch_offset = 0;
  for (uint8_t i = 0; i < 100; i++) {
      MPU6050_Update(&hi2c1, &cs); MPU6050_Update(&hi2c2, &cb);
      rs += cs.roll; ps += cs.pitch; rb += cb.roll; pb += cb.pitch;
      if ((i+1) % 10 == 0) {
          snprintf(buf, sizeof(buf), "Progress: %d%%", (i+1));
          OLED_ShowString(&hi2c1, 0, 32, buf);
          OLED_Refresh(&hi2c1);
      }
      HAL_Delay(10);
  }
  MPU6050_SCAN.roll_offset = rs/100; MPU6050_SCAN.pitch_offset = ps/100;
  MPU6050_BREAD.roll_offset = rb/100; MPU6050_BREAD.pitch_offset = pb/100;

  /* 【补充】初始化新增的滤波状态变量，避免未定义行为 */
  MPU6050_SCAN.last_update_ms = 0;
  MPU6050_SCAN.gyro_roll = 0;       MPU6050_SCAN.gyro_pitch = 0;
  MPU6050_SCAN.avg_idx = 0;         MPU6050_SCAN.first_frame = 1;
  MPU6050_SCAN.still_count = 0;
  MPU6050_BREAD.last_update_ms = 0;
  MPU6050_BREAD.gyro_roll = 0;      MPU6050_BREAD.gyro_pitch = 0;
  MPU6050_BREAD.avg_idx = 0;        MPU6050_BREAD.first_frame = 1;
  MPU6050_BREAD.still_count = 0;
  /* 将滑动平均窗口预填为校准后初值（不是0，避免开机几秒内结果被0拉低） */
  for (uint8_t i = 0; i < MPU_AVG_WINDOW; i++) {
      MPU6050_SCAN.avg_roll[i] = rs/100;   MPU6050_SCAN.avg_pitch[i] = ps/100;
      MPU6050_BREAD.avg_roll[i] = rb/100;  MPU6050_BREAD.avg_pitch[i] = pb/100;
  }
  /* -------- 初始化雷达扫描历史缓存 --------
   * 【Flash优化】如果Flash已经成功恢复了上次断电的雷达图，就不清空，开机OLED直接显示！
   *              只有Flash没恢复到数据时，才初始化为全空 */
  {
    uint8_t need_clear = 1;
    if (flash_ok) {
      uint8_t magic[4];
      W25QXX_Read(FLASH_ADDR_LAST_FRAME, magic, 4);
      if (magic[0]=='R' && magic[1]=='D' && magic[2]=='R' && magic[3]=='!') need_clear = 0;
      /* 3圈历史：Flash没恢复的圈就清空，恢复了的就保留 */
      magic[0]=0; magic[1]=0; magic[2]=0;
      W25QXX_Read(FLASH_ADDR_HIST_RING0, magic, 3);
      if (!(magic[0]=='R' && magic[1]=='0' && magic[2]=='!')) {
        for (uint16_t a=0;a<RADAR_HIST_SIZE;a++) {radar_ring0[a].dist_mm=HCSR04_DIST_INVALID; radar_ring0[a].last_seen_ms=0;}
      }
      magic[0]=0; magic[1]=0; magic[2]=0;
      W25QXX_Read(FLASH_ADDR_HIST_RING1, magic, 3);
      if (!(magic[0]=='R' && magic[1]=='1' && magic[2]=='!')) {
        for (uint16_t a=0;a<RADAR_HIST_SIZE;a++) {radar_ring1[a].dist_mm=HCSR04_DIST_INVALID; radar_ring1[a].last_seen_ms=0;}
      }
      magic[0]=0; magic[1]=0; magic[2]=0;
      W25QXX_Read(FLASH_ADDR_HIST_RING2, magic, 3);
      if (!(magic[0]=='R' && magic[1]=='2' && magic[2]=='!')) {
        for (uint16_t a=0;a<RADAR_HIST_SIZE;a++) {radar_ring2[a].dist_mm=HCSR04_DIST_INVALID; radar_ring2[a].last_seen_ms=0;}
      }
    }
    if (need_clear) {
      for (uint16_t a = 0; a < RADAR_HIST_SIZE; a++) {
          radar_history[a].dist_mm = HCSR04_DIST_INVALID;
          radar_history[a].last_seen_ms = 0;
      }
    }
  }

  OLED_Clear(&hi2c1);
  OLED_ShowString(&hi2c1, 0, 0, "Calibration OK");
  snprintf(buf, sizeof(buf), "S:R%.1f P%.1f", rs/100, ps/100);
  OLED_ShowString(&hi2c1, 0, 16, buf);
  snprintf(buf, sizeof(buf), "B:R%.1f P%.1f", rb/100, pb/100);
  OLED_ShowString(&hi2c1, 0, 32, buf);
  OLED_Refresh(&hi2c1);
  HAL_Delay(2000);

  /* USER CODE END 2 */

  /* ===== 校准完成，舵机开始自动扫描 ===== */
  uint16_t servo_angle = 0;       /* 当前舵机角度 */
  int8_t  servo_dir = 1;          /* 扫描方向：1=增加，-1=减小 */
  uint8_t servo_step = 2;         /* 每步角度（度） */
  uint32_t last_tick = HAL_GetTick(); /* 上次循环时间戳 */
  Servo_SetAngle(0);              /* 先转到0度 */

  /* 【Flash-显示增强】换向次数计数：每换向2次=完整扫完一圈(0→180→0)，推历史环
   * 每扫完1圈：RING2→丢，RING1→RING2，RING0→RING1，当前radar_history→RING0 */
  uint8_t  rev_count = 0;   /* 换向计数，到2=1圈 */

  /* 【初始化V2】舵机状态+机械可信度模型的初值
     上电后前300ms视为不稳定过渡期，可信度从0.1升到1.0 */
  servo_last_dir = servo_dir;
  servo_dir_change_ms = HAL_GetTick();

  /* 等待舵机转到0度（1秒），等待期间持续轮询ESP32串口 */
  uint32_t t0 = HAL_GetTick();
  while (HAL_GetTick() - t0 < 1000) {
      parse_esp32_data();
  }
  HAL_UART_Transmit(&huart1, (uint8_t*)"开始自动扫描模式\r\n", 18, 100);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 0. 【关键】每次大操作前先处理ESP32数据，增加接收成功率 */
    parse_esp32_data();

    /* ================================================================
     * 1. 温度融合物理模型（精度提升核心：ESP32+MPU双温度源融合）
     * 物理原理：
     *   芯片温度 = 局部真实气温 + 芯片自加热温升ΔT
     *   ΔT稳定后近似恒定，因此可用ESP32气温校准ΔT
     * ================================================================ */
    /* 先读取最新MPU芯片温度（高实时性） */
    mpu_chip_temp = MPU6050_SCAN.temperature;

    /* --- 收到新ESP32数据时：校准芯片自加热温升ΔT --- */
    if (env_data_valid) {
        /* 单次校准值：ΔT_one = T_chip - T_esp32 */
        float delta_T_raw = mpu_chip_temp - env_temp_esp32;
        /* 合理性范围检查：ΔT不会低于3°C或超过20°C，否则是异常数据 */
        if (delta_T_raw >= MIN_VALID_HEAT_RISE && delta_T_raw <= MAX_VALID_HEAT_RISE) {
            /* 把单次校准值放入4点滑动平均窗口，抑制偶然误差 */
            heat_rise_hist[heat_rise_idx] = delta_T_raw;
            heat_rise_idx = (heat_rise_idx + 1) % HEAT_RISE_AVG_WIN;
            float sum_hr = 0;
            for (uint8_t i = 0; i < HEAT_RISE_AVG_WIN; i++) sum_hr += heat_rise_hist[i];
            delta_T_selfheat = sum_hr / (float)HEAT_RISE_AVG_WIN;
            heat_rise_calibrated = 1;
        }
        /* 调试输出：校准过程可追踪（Fix: 全英文避免Keil编码错误） */
        char dbg[128];
        snprintf(dbg, sizeof(dbg),
            "[TempFusion] ESP32: T%.1fC H%.1f%% | MPUchip:%.1fC | dT:%.1fC | %s\r\n",
            env_temp_esp32, env_hum_esp32, mpu_chip_temp, delta_T_selfheat,
            heat_rise_calibrated ? "Calibrated" : "1stWait");
        HAL_UART_Transmit(&huart1, (uint8_t*)dbg, strlen(dbg), 100);
        env_data_valid = 0;
    }

    /* --- 实时局部温度估算（用校准后的ΔT反推） --- */
    /* 如果已校准过：T_local = T_chip - ΔT_calibrated
       如果从未校准：用默认温升10°C估算 */
    if (heat_rise_calibrated) {
        T_local_estimated = mpu_chip_temp - delta_T_selfheat;
    } else {
        T_local_estimated = mpu_chip_temp - DEFAULT_HEAT_RISE_TEMP;
    }

    /* --- 最终融合温度（权重融合，兼顾长期准确+实时） --- */
    /* ESP32(70%) + MPU实时估算(30%)，即使ESP32更新慢也能保持温度变化敏感度 */
    T_fused_final = TEMP_FUSION_ALPHA * env_temp_esp32 +
                    (1.0f - TEMP_FUSION_ALPHA) * T_local_estimated;

    /* 为保持HCSR04.c兼容：把最终融合温度赋值给原接口变量estimated_ambient */
    estimated_ambient = T_fused_final;

    /* 处理串口 */
    parse_esp32_data();

    /* 3. 更新MPU6050姿态数据 */
    MPU6050_Update(&hi2c1, &MPU6050_SCAN);
    parse_esp32_data();  /* I2C操作间隙检查串口 */
    MPU6050_Update(&hi2c2, &MPU6050_BREAD);
    parse_esp32_data();

    /* ============== 3.5 【新增】双MPU差分计算残余振动幅度 ==============
     * 物理原理：面包板MPU是固定在面包板上的 → 它测量到的角速度=桌面/面包板整体晃动
     *          扫描头MPU（舵机上）的角速度 = 舵机旋转角速度 + 桌面晃动 + 杜邦线拉的局部抖动
     *          两者差分 |scan_gyro - bread_gyro| = 杜邦线/热熔胶导致的局部残余抖动
     *          这个值越大 → 当前超声波测量结果越不可信
     * ===================================================================== */
    {
        float dgx = MPU6050_SCAN.gx - MPU6050_BREAD.gx;
        float dgy = MPU6050_SCAN.gy - MPU6050_BREAD.gy;
        float dgz = MPU6050_SCAN.gz - MPU6050_BREAD.gz;
        if (dgx < 0) dgx = -dgx;
        if (dgy < 0) dgy = -dgy;
        if (dgz < 0) dgz = -dgz;
        /* 取最大轴差分作为残余振动幅度(°/s) */
        current_vib_level = dgx;
        if (dgy > current_vib_level) current_vib_level = dgy;
        if (dgz > current_vib_level) current_vib_level = dgz;
    }

    /* ============== 3.6 【新增】计算当前舵机可信度（梯形函数） ============== */
    {
        uint32_t since_last_rev = HAL_GetTick() - servo_dir_change_ms;
        if (since_last_rev < SERVO_STABLE_PHASE1_MS) {
            current_servo_conf = SERVO_MIN_CONF;  /* 0~100ms = 0.1 */
        } else if (since_last_rev < SERVO_STABLE_PHASE2_MS) {
            /* 100~300ms: 线性从0.1插值到1.0 */
            float k = (float)(since_last_rev - SERVO_STABLE_PHASE1_MS)
                    / (float)(SERVO_STABLE_PHASE2_MS - SERVO_STABLE_PHASE1_MS);
            current_servo_conf = SERVO_MIN_CONF + k * (SERVO_MAX_CONF - SERVO_MIN_CONF);
        } else {
            current_servo_conf = SERVO_MAX_CONF;  /* >300ms = 1.0 */
        }
    }

    /* 4. 超声波测距【机械鲁棒V2版】：
     *   传入①舵机运动可信度（刚换向降权）②双MPU差分残余振动幅度（越大降权越狠）
     *   内部8层复合滤波（中值+跳变+卡尔曼+加权滑窗） */
    /* 扫描头(通道0)：Trig=PA1, Echo=PA2（舵机上，会振） */
    dist_scan = HCSR04_MeasureRobust(0, GPIOA, GPIO_PIN_1, GPIOA, GPIO_PIN_2,
                                     current_servo_conf, current_vib_level);
    parse_esp32_data();
    /* 面包板基准(通道1)：Trig=PA4, Echo=PB0（固定） */
    dist_base = HCSR04_MeasureRobust(1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0,
                                     1.0f, 0.0f);  /* 基准固定 → 最大可信度, 振动=0 */
    parse_esp32_data();

    /* -------- 把【当前角度】的测距结果存入雷达历史缓存 --------
     * 每个角度(0~180°)都单独存，后面每帧重绘所有角度 → 扇形拖影保留 */
    {
        uint16_t ang = servo_angle;
        if (ang > 180) ang = 180;  /* 越界保护 */
        /* 提前算一下可信度 (和后面障碍物判定的q_total完全一致) */
        radar_history[ang].dist_mm       = dist_scan;   /* 测到就存，不管可信度 */
        radar_history[ang].last_seen_ms  = HAL_GetTick();
    }

    /* ==================== 5. OLED 雷达扫描可视化显示（信息齐全+可信度版） ====================
     * 布局（128x64像素，3行数据 + 雷达区）：
     *   y=0~39  [40px高] 雷达扇形区：圆心(64,40)，最大半径40px，对应4000mm
     *                         参考圆10/20/30px=1m/2m/3m + 扫描线 + 障碍物点
     *   y=40~47 [第1行]    S1234 B5678 90          扫描距离mm/基准距离mm/舵机角度°
     *   y=48~55 [第2行]    TE28.5 H65%              ESP32气温 / 湿度
     *   y=56~63 [第3行]    TM40 V1.5 C90%           MPU芯片温度 / 振动幅度°/s / 数据可信度%
     *=========================================================================== */
    OLED_Clear(&hi2c1);

    /* ==========================================================
     * 雷达扇形区（y=0~39，高40像素）
     * 圆心(64,40)，最大半径=40像素 → 最大距离4000mm
     * 比例因子 = 40像素 / 4000mm = 0.01 像素/mm
     * ========================================================== */
    /* 3层参考圆：1m/2m/3m → 10px/20px/30px */
    OLED_DrawSemicircle(&hi2c1, 64, 40, 10, 1);  /* 1000mm */
    OLED_DrawSemicircle(&hi2c1, 64, 40, 20, 1);  /* 2000mm */
    OLED_DrawSemicircle(&hi2c1, 64, 40, 30, 1);  /* 3000mm */
    /* 参考线：左边界(180°)、中心(90°)、右边界(0°)，让雷达区看起来更像雷达 */
    OLED_DrawLine(&hi2c1, 64, 40, 24, 40, 1);    /* 左边界 180°：← */
    OLED_DrawLine(&hi2c1, 64, 40, 64, 10, 1);    /* 中心 90°：↑ */
    OLED_DrawLine(&hi2c1, 64, 40, 104, 40, 1);   /* 右边界 0°：→ */

    /* -------- VIVID LABELS: 1M/2M/3M distance rings + 0/90/180 angles + [OK] icon
     * (so user doesn't need to guess what the circles/lines mean) -------- */
    OLED_RadarUI_DrawLabels(&hi2c1, current_servo_conf, current_vib_level);

    /* ================================================================
     * 【Flash驱动OLED显示效果 - 3层时间衰减拖影 + 角度表加速】
     * 之前：每帧只画单圈历史点，15秒过期，没有层次感
     * 现在：3圈环形缓冲(RING0最新→RING1中→RING2最旧)
     *       RING2(最旧)=极小点(半透明感) → RING1(中)=小点 → RING0(最新)=十字 → 当前=大方块
     *       和真雷达的时间衰减拖影一模一样！
     *       用Flash预计算角度表(cos/sin) → 不用每帧算181次三角函数 → 帧率翻倍！
     * ================================================================ */
    {
        uint32_t now_ms = HAL_GetTick();
        /* 用角度表还是实时算？flash_ok=1就用表(快)，否则实时算(降级) */
        for (uint16_t a = 0; a < RADAR_HIST_SIZE; a++) {
            /* 跳过无效/过期的历史点（15秒没再扫到的清掉，避免画面脏） */
            if (radar_history[a].dist_mm == HCSR04_DIST_INVALID) continue;
            if ((now_ms - radar_history[a].last_seen_ms) > RADAR_HIST_EXPIRE_MS) continue;

            /* 每个历史角度→像素坐标（用Flash角度表加速！） */
            float c, s;
            if (flash_ok) {
                c = flash_cos_table[a];
                s = flash_sin_table[a];
            } else {
                float rad = (180.0f - (float)a) * ((float)M_PI / 180.0f);
                c = cosf(rad); s = sinf(rad);
            }
            float ratio_h;
            uint16_t d = radar_history[a].dist_mm;
            if      (d < 100)   ratio_h = 1.0f;
            else if (d < 4000)  ratio_h = (float)d * 0.01f;
            else                ratio_h = 40.0f;
            int16_t hx = 64 + (int16_t)(ratio_h * c);
            int16_t hy = 40 - (int16_t)(ratio_h * s);
            if (hy > 41) hy = 41;
            if (hy < 0)   hy = 0;
            if (hx < 0)   hx = 0;
            if (hx > 127) hx = 127;
            /* 历史点统一画【单点】：旧点别抢当前点的风头 */
            OLED_DrawPoint(&hi2c1, (uint8_t)hx, (uint8_t)hy, 1);
        }

        /* ======== 【Flash核心效果】3圈时间衰减拖影 ========
         * RING2(最旧, 2~3圈前)=极小点 → RING1(中, 1~2圈前)=小点 → RING0(最新, 上1圈)=十字
         * 显示顺序：先画最旧的(最暗)，再画新的(最亮)，最后当前角度盖在上面
         * 和真雷达扫描效果一模一样：越近扫到的越亮，越久远的越暗！ */
        if (flash_ok) {
            /* --- RING2(最旧): 极小点(只画1个点, 距离<3m才画) --- */
            for (uint16_t a = 0; a < RADAR_HIST_SIZE; a++) {
                if (radar_ring2[a].dist_mm == HCSR04_DIST_INVALID) continue;
                if (radar_ring2[a].dist_mm > 3000) continue;
                int16_t hx = 64 + (int16_t)(flash_cos_table[a] * (float)radar_ring2[a].dist_mm * 0.01f);
                int16_t hy = 40 - (int16_t)(flash_sin_table[a] * (float)radar_ring2[a].dist_mm * 0.01f);
                if (hx>=0 && hx<=127 && hy>=0 && hy<=41)
                    OLED_DrawPoint(&hi2c1, (uint8_t)hx, (uint8_t)hy, 1);
            }
            /* --- RING1(中): 小十字(距离<3.5m才画) --- */
            for (uint16_t a = 0; a < RADAR_HIST_SIZE; a++) {
                if (radar_ring1[a].dist_mm == HCSR04_DIST_INVALID) continue;
                if (radar_ring1[a].dist_mm > 3500) continue;
                float r = (float)radar_ring1[a].dist_mm * 0.01f;
                int16_t hx = 64 + (int16_t)(flash_cos_table[a] * r);
                int16_t hy = 40 - (int16_t)(flash_sin_table[a] * r);
                if (hx>=1 && hx<=126 && hy>=1 && hy<=40) {
                    OLED_DrawPoint(&hi2c1, (uint8_t)hx, (uint8_t)hy, 1);
                    OLED_DrawPoint(&hi2c1, (uint8_t)(hx-1), (uint8_t)hy, 1);
                    OLED_DrawPoint(&hi2c1, (uint8_t)(hx+1), (uint8_t)hy, 1);
                    OLED_DrawPoint(&hi2c1, (uint8_t)hx, (uint8_t)(hy-1), 1);
                    OLED_DrawPoint(&hi2c1, (uint8_t)hx, (uint8_t)(hy+1), 1);
                }
            }
            /* --- RING0(最新, 上1圈): 大方块(距离<4m才画) --- */
            for (uint16_t a = 0; a < RADAR_HIST_SIZE; a++) {
                if (radar_ring0[a].dist_mm == HCSR04_DIST_INVALID) continue;
                if (radar_ring0[a].dist_mm > 4000) continue;
                float r = (float)radar_ring0[a].dist_mm * 0.01f;
                int16_t hx = 64 + (int16_t)(flash_cos_table[a] * r);
                int16_t hy = 40 - (int16_t)(flash_sin_table[a] * r);
                if (hx>=1 && hx<=126 && hy>=1 && hy<=40) {
                    /* 3x3大方块 */
                    for (int8_t dx=-1; dx<=1; dx++)
                        for (int8_t dy=-1; dy<=1; dy++)
                            OLED_DrawPoint(&hi2c1, (uint8_t)(hx+dx), (uint8_t)(hy+dy), 1);
                }
            }
        }
    }

    /* 扫描角度线（随舵机角度实时旋转）：0~180°舵机 → OLED上180°~0° */
    float oled_angle_rad, oled_c, oled_s;
    if (flash_ok) {
        /* 用角度表：servo_angle就是索引(0~180) */
        oled_c = flash_cos_table[servo_angle];
        oled_s = flash_sin_table[servo_angle];
    } else {
        oled_angle_rad = (180.0f - (float)servo_angle) * ((float)M_PI / 180.0f);
        oled_c = cosf(oled_angle_rad);
        oled_s = sinf(oled_angle_rad);
    }
    int16_t line_x = 64 + (int16_t)(40.0f * oled_c);
    int16_t line_y = 40 - (int16_t)(40.0f * oled_s);
    if (line_y < 0) line_y = 0;
    if (line_x < 0) line_x = 0;
    if (line_x > 127) line_x = 127;
    OLED_DrawLine(&hi2c1, 64, 40, (uint8_t)line_x, (uint8_t)line_y, 1);

    /* Obstacle point: GRADED SIZE by distance + quality (vivid, 3 levels)
     *   <0.5m + high trust => BIG 3x3 SQUARE (danger warning, stand out!)
     *   0.5~2m             => normal cross (+)
     *   >2m or low trust   => small single dot (don't scare user w/ noisy far data)
     * Skip drawing when quality < 0.4 (avoids misleading ghost obstacles) */
    {
        float vc = 1.0f;
        if (current_vib_level > 1.0f) {
            vc = 1.0f - 0.1f * (current_vib_level - 1.0f);
            if (vc < 0.1f) vc = 0.1f;
        }
        float q_total = current_servo_conf * vc;
        if (q_total < 0.05f) q_total = 0.05f;

        if (dist_scan != HCSR04_DIST_INVALID && q_total > 0.15f) {
            /* 比例：1000mm=1米=10像素(对齐参考圆半径10)，最大4米=40像素(对齐参考线端点) */
            float ratio;
            if      (dist_scan < 100) ratio = 1.0f;   /* 盲区<10cm按10cm算，强制画出来避免"贴脸看不到" */
            else if (dist_scan < 4000) ratio = (float)dist_scan * 0.01f;
            else                       ratio = 40.0f;
            int16_t px = 64 + (int16_t)(ratio * cosf(oled_angle_rad));
            int16_t py = 40 - (int16_t)(ratio * sinf(oled_angle_rad));
            /* 【关键Fix：Debug兜底】哪怕不在扇形区，只要合理就强制缩到边界，至少有个点提示用户 */
            if (py > 40) py = 40;      /* 圆心线以下不算雷达区，但至少画在圆心边缘 */
            if (px < 0)  px = 0;
            if (px > 127) px = 127;
            OLED_RadarUI_DrawObstacle(&hi2c1, px, py, dist_scan, q_total);
        }
        /* 【Debug辅助】哪怕可信度太低被过滤，只要测距有效，强制在圆心画个极小的提示点，避免用户以为模块坏了 */
        else if (dist_scan != HCSR04_DIST_INVALID) {
            OLED_DrawPoint(&hi2c1, 64, 40, 1);
        }
    }

    /* Line1(y=40): D1=ScanHead:mm D2=BreadBoard:mm A=Angle:deg (ALL explicit) */
    OLED_RadarUI_Line1_DistAngle(&hi2c1,
                                 dist_scan, dist_base, servo_angle,
                                 HCSR04_DIST_INVALID);

    /* Line2(y=48): AIR=fused ambient temp(used for sound speed, MOST IMPORTANT TEMP
     *                RH=relative humidity (percent)
     * prefix "?" = not calibrated yet via ESP32 packet */
    OLED_RadarUI_Line2_Weather(&hi2c1,
                                T_fused_final,  /* fused temp, not raw esp32 */
                                env_hum_esp32,
                                heat_rise_calibrated);

    /* Line3(y=56): Mechanical status (for your hotglue+dupont build)
     *                Q = Data quality score 0-100 (higher=trust)
     *                V = Vibration in DPS (Degree Per Second, gyro standard unit)
     * Removed cryptic TM/C% abbrev - intuitive Q(quality)+V(dps) */
    OLED_RadarUI_Line3_MechStatus(&hi2c1,
                                   current_servo_conf,
                                   current_vib_level);

    OLED_Refresh(&hi2c1);

    /* ======== 【Flash节流存回】每10秒把当前雷达图+3圈历史写回Flash ========
     * 为什么节流？W25QXX写太频繁会加速Flash磨损，10秒写1次够用
     * 写什么？radar_history(当前圈) + radar_ring0/1/2(3圈历史)
     * 掉电后开机：直接恢复最后一次保存的雷达图，不用重新扫！ */
    if (flash_ok && (HAL_GetTick() - last_flash_save_ms) > 10000) {
        last_flash_save_ms = HAL_GetTick();
        /* 写当前圈: 前4字节MAGIC "RDR!" 标记有效 */
        W25QXX_SectorErase(FLASH_ADDR_LAST_FRAME);
        uint8_t m_last[4] = {'R','D','R','!'};
        W25QXX_Write(FLASH_ADDR_LAST_FRAME, m_last, 4);
        W25QXX_Write(FLASH_ADDR_LAST_FRAME + 4, (uint8_t*)radar_history, sizeof(RadarHist_t)*RADAR_HIST_SIZE);
        /* 写3圈历史: 前3字节MAGIC "R0!" / "R1!" / "R2!" */
        W25QXX_SectorErase(FLASH_ADDR_HIST_RING0);
        uint8_t m0[3] = {'R','0','!'};
        W25QXX_Write(FLASH_ADDR_HIST_RING0, m0, 3);
        W25QXX_Write(FLASH_ADDR_HIST_RING0 + 3, (uint8_t*)radar_ring0, sizeof(RadarHist_t)*RADAR_HIST_SIZE);
        W25QXX_SectorErase(FLASH_ADDR_HIST_RING1);
        uint8_t m1[3] = {'R','1','!'};
        W25QXX_Write(FLASH_ADDR_HIST_RING1, m1, 3);
        W25QXX_Write(FLASH_ADDR_HIST_RING1 + 3, (uint8_t*)radar_ring1, sizeof(RadarHist_t)*RADAR_HIST_SIZE);
        W25QXX_SectorErase(FLASH_ADDR_HIST_RING2);
        uint8_t m2[3] = {'R','2','!'};
        W25QXX_Write(FLASH_ADDR_HIST_RING2, m2, 3);
        W25QXX_Write(FLASH_ADDR_HIST_RING2 + 3, (uint8_t*)radar_ring2, sizeof(RadarHist_t)*RADAR_HIST_SIZE);
    }
    parse_esp32_data();

    /* 6. 串口发送VOFA数据（统一8列，Flyweight协议） */
    vofa_send_all(&MPU6050_SCAN, &MPU6050_BREAD, dist_scan, dist_base);

    /* 7. 舵机自动扫描（每80ms更新一次角度） */
    if (HAL_GetTick() - last_tick >= 80) {
        last_tick = HAL_GetTick();
        servo_angle += servo_dir * servo_step;
        if (servo_angle >= 180) {
            servo_angle = 180;
            if (servo_dir != -1) {             /* 【新增】检测到换向：记录时间戳 */
                servo_dir = -1;
                servo_last_dir = -1;
                servo_dir_change_ms = HAL_GetTick();
                rev_count++;
            }
        } else if (servo_angle <= 0) {
            servo_angle = 0;
            if (servo_dir != 1) {              /* 【新增】检测到换向：记录时间戳 */
                servo_dir = 1;
                servo_last_dir = 1;
                servo_dir_change_ms = HAL_GetTick();
                rev_count++;
            }
        }
        /* 【Flash-显示增强】每换向2次 = 完整扫完1圈(0→180→0)：推进3层历史环形缓冲区 */
        if (rev_count >= 2 && flash_ok) {
            rev_count = 0;
            /* RING2→丢弃, RING1→RING2, RING0→RING1, 当前radar_history→RING0 */
            memcpy(radar_ring2, radar_ring1, sizeof(RadarHist_t)*RADAR_HIST_SIZE);
            memcpy(radar_ring1, radar_ring0, sizeof(RadarHist_t)*RADAR_HIST_SIZE);
            memcpy(radar_ring0, radar_history, sizeof(RadarHist_t)*RADAR_HIST_SIZE);
        }
        Servo_SetAngle(servo_angle);
    }
    
    /* 8. 【关键】代替HAL_Delay(80)，空转时持续轮询软件串口，避免错过ESP32数据 */
    while (HAL_GetTick() - last_tick < 70) {
        parse_esp32_data();
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  htim2.Init.Prescaler = 719;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
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
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA4 - 超声波Trig输出
   * PA3保留给USART2_RX（ESP32通讯），在HAL_UART_MspInit中配置 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
/**
  * @brief USART2 Initialization (9600, RX-only for ESP32)
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);
}