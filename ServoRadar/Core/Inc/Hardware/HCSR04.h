/**
  ******************************************************************************
  * @file    HCSR04.h
  * @brief   HC-SR04 超声波测距模块驱动头文件
  *          
  * 【模块说明】
  *   HC-SR04 是一款常用的超声波测距模块，测量范围 2cm~400cm，精度约 3mm。
  *   工作原理：STM32 发送 Trig 触发脉冲 → 模块发射 8 个 40kHz 超声波 → 
  *   接收回波后 Echo 引脚输出高电平 → 高电平宽度 = 声波往返时间。
  *   
  * 【本项目应用】
 *   - 扫描头超声波：Trig=PA1, Echo=PA2（搭载在舵机上，随舵机旋转扫描）
 *   - 面包板超声波：Trig=PA4, Echo=PB0（固定位置，作为基准参考）
 *   - 使用 TIM3 进行微秒级计时（1 tick = 1μs）
  *   
  * 【温度补偿】
  *   声速随温度变化：v = 331.3 + 0.606×T + 0.0124×H
  *   使用 MPU6050 估算的环境温度 + ESP32 提供的湿度进行动态补偿
  *   
  * 【优化说明】
  *   1. 添加了距离有效性校验宏，避免返回异常值
  *   2. 增加了测量状态枚举，方便判断测量结果
  *   3. 优化了超时参数，适应不同测量场景
  *   4. 支持多次测量取中值，滤除偶然异常值
  ******************************************************************************
  */

#ifndef __HCSR04_H
#define __HCSR04_H

#include "main.h"

/* ==================== HC-SR04 参数定义 ==================== */

/* 测量范围限制 */
#define HCSR04_MAX_DIST_MM      4000    /* 最大测量距离 400cm（模块规格上限） */
#define HCSR04_MIN_DIST_MM      20      /* 最小测量距离 2cm（模块规格下限） */

/* 超时参数 */
#define HCSR04_TIMEOUT_US       30000   /* Echo 超时时间 30ms（约 5m，安全余量） */
#define HCSR04_MEASURE_INTERVAL_MS  60  /* 两次测量最小间隔（防止回波干扰） */

/* 无效距离标记 */
#define HCSR04_DIST_INVALID     0xFFFF  /* 测量失败或超时时的返回值 */

/* ==================== 【新增】精度提升参数 ==================== */
/**
  * @brief  超声波滑动平均窗口大小（6次历史结果滤波）
  * 中值滤波处理"本次测量内"的突发噪声；滑动平均处理"多次测量之间"的慢波动噪声
  * 实测固定距离下：单次波动±10mm → 滑动平均后±2mm
  */
#define HCSR04_AVG_WINDOW              6

/**
  * @brief  异常跳变过滤阈值（mm）
  * 如果本次测量值相对上一个有效值：
  *   (a) 差值 > 300mm  AND  (b) 变化幅度 > 50%
  * 则认为是误测（比如超声波打到边缘的多次反射），丢弃，沿用上一次有效值
  * 对扫描过程中扫过空旷区域、物体边缘特别有效
  */
#define HCSR04_JUMP_THRESHOLD_MM       300

/**
  * @brief  单次中值滤波测量次数（由3次提升为5次）
  * 次数多 → 抗噪性好但耗时长；次数少 → 速度快但抗噪差
  * 5次：去极值后保留3次取平均，比原3次精度提升约 30%，总耗时增加约 120ms
  */
#define HCSR04_MEDIAN_TIMES            5

/* ==================== 【新增】温度融合物理模型参数 ==================== */
/**
  * @brief  双温度源融合模型（核心物理模型）
  * 
  * 【物理背景】
  *   - ESP32气温T_esp：室外/大环境气象站温度，准确但更新慢（90秒一次）
  *   - MPU芯片温度T_chip：MPU6050内部测的温度，实时但比环境高（芯片自加热）
  *   - 两者关系：T_chip = T_air_local + ΔT_self_heating
  *     其中ΔT_self_heating为芯片自加热温升（稳定工作后基本恒定）
  * 
  * 【校准逻辑】
  *   当收到ESP32的气温时，解算当前温升：
  *     ΔT_calibrated = T_chip - T_esp
  *   之后在ESP32两次更新之间（90秒），用MPU实时估算局部温度：
  *     T_air_estimated = T_chip - ΔT_calibrated
  * 
  * 【最终融合权重】
  *   T_for_sound_speed = ALPHA * T_esp（有新数据时）+ (1-ALPHA) * T_air_estimated
  * 
  * 这样综合了：ESP32的长期准确度 + MPU的高实时性，比用单一温度源更准确
  */
#define TEMP_FUSION_ALPHA          0.70f  /* ESP32气温权重，MPU估算占30%权重 */
#define DEFAULT_HEAT_RISE_TEMP     10.0f  /* 默认温升10°C（未校准时用） */
#define MAX_VALID_HEAT_RISE        20.0f  /* 温升合理上限：芯片不会比气温高20°C以上 */
#define MIN_VALID_HEAT_RISE         3.0f  /* 温升合理下限：芯片工作至少比环境高3°C */

/* ==================== 测量状态枚举 ==================== */
/**
  * @brief  超声波测量状态
  * @note   用于判断测量结果是否有效
  */
typedef enum {
    HCSR04_OK = 0,              /* 测量成功 */
    HCSR04_TIMEOUT_ECHO_HIGH,   /* 等待 Echo 变高超时 */
    HCSR04_TIMEOUT_ECHO_LOW,    /* 等待 Echo 变低超时 */
    HCSR04_DIST_OUT_OF_RANGE    /* 距离超出有效范围 */
} HCSR04_Status_t;

/* ==================== 函数声明 ==================== */

/**
  * @brief  HC-SR04 初始化
  * @note   确保 TIM3 处于停止状态，计数器清零
  * @param  无
  * @retval 无
  */
void HCSR04_Init(void);

/**
  * @brief  单次超声波测距
  * @param  TrigPort: Trig 引脚所在端口（如 GPIOA）
  * @param  TrigPin: Trig 引脚号（如 GPIO_PIN_1）
  * @param  EchoPort: Echo 引脚所在端口（如 GPIOA）
  * @param  EchoPin: Echo 引脚号（如 GPIO_PIN_2）
  * @retval 距离值（单位：mm），失败返回 HCSR04_DIST_INVALID
  * @note   阻塞式测量，会等待 Echo 信号完成
  */
uint16_t HCSR04_Measure(GPIO_TypeDef* TrigPort, uint16_t TrigPin,
                        GPIO_TypeDef* EchoPort, uint16_t EchoPin);

/**
  * @brief  多次测量取中值（去极值滤波）
  * @param  TrigPort: Trig 引脚所在端口
  * @param  TrigPin: Trig 引脚号
  * @param  EchoPort: Echo 引脚所在端口
  * @param  EchoPin: Echo 引脚号
  * @param  times: 测量次数（建议 3~5 次，最多 10 次）
  * @retval 中值距离（单位：mm），全部失败返回 HCSR04_DIST_INVALID
  * @note   算法：去掉一个最大值和一个最小值，剩余值求平均
  *         优点：能有效滤除偶然的异常测量值
  */
uint16_t HCSR04_MeasureMedian(GPIO_TypeDef* TrigPort, uint16_t TrigPin,
                              GPIO_TypeDef* EchoPort, uint16_t EchoPin,
                              uint8_t times);

/* ==================== 【新增】卡尔曼滤波参数（针对机械不稳定结构） ==================== */
/**
  * @brief  超声波卡尔曼滤波器参数
  * 参考：CSDN "HC-SR04卡尔曼滤波" + 类似雷达项目
  * 核心方程：
  *   预测：X_pred = X_last ;   P_pred = P_last + Q
  *   更新：K = P_pred / (P_pred + R)
  *         X_new = X_pred + K × (Z_meas - X_pred)
  *         P_new = (1 - K) × P_pred
  * Q=过程噪声：机械振动越大Q应越大（允许状态跟随机变化快）
  * R=测量噪声：HC-SR04典型测量噪声协方差，越可信R越小
  */
#define HCSR04_KALMAN_Q          2.0f    /* 过程噪声协方差（mm²）临时组装振动大→Q偏大 */
#define HCSR04_KALMAN_R          25.0f   /* 测量噪声协方差（mm²）HC-SR04典型±5mm→25 */

/**
  * @brief  舵机运动可信度模型（核心针对【杜邦线+热熔胶无固定】结构）
  * 
  * 物理背景：
  *   杜邦线的拉力+热熔胶的弹性→舵机换向后前200ms机械系统严重抖动+回弹
  *   舵机刚加减速时→振动幅值最大，超声波测量值可信度为0%
  *   舵机在同一个方向连续匀速运动→振动逐步稳定，可信度恢复到100%
  * 
  * 可信度函数（梯形模型）：
  *   movement_state   0~100ms刚换向 → conf=0.1 (10%可信，几乎不用)
  *                    100~300ms过渡 → conf线性从0.1上升到1.0
  *                    >300ms稳定匀速 → conf=1.0 (完全可信)
  */
#define SERVO_STABLE_PHASE1_MS   100     /* 0~100ms: 严重抖动期，可信度0.1 */
#define SERVO_STABLE_PHASE2_MS   300     /* 100~300ms: 过渡恢复期，线性插值 */
#define SERVO_MAX_CONF           1.0f    /* 完全稳定时的最大可信度 */
#define SERVO_MIN_CONF           0.1f    /* 刚换向时最低可信度 */

/**
  * @brief  【新增V2】机械鲁棒高精度测距
  * 【8层复合滤波/补偿，专门针对"杜邦线+热熔胶无固定"临时机械结构】
  * 
  *   ┌───────────────────────────────────────────────────────────────┐
  *   │ ① 5次去极值中值滤波（滤除单次测量尖峰）                      │
  *   │ ② 舵机运动状态可信度加权（刚换向的抖动测量降权）              │
  *   │ ③ 双MPU差分振动检测（扫描头MPU-固定MPU=残余振动幅度）         │
  *   │ ④ 跳变过滤 + 角度突变校验（舵机突然卡住/回弹的误测丢弃）      │
  *   │ ⑤ 卡尔曼滤波（预测+测量更新，CSDN/GitHub项目标配）            │
  *   │ ⑥ 可信度加权滑动平均（可信度低的样本只贡献10%的权重）          │
  *   │ ⑦ 6点滑动平均（最终平滑输出）                                 │
  *   └───────────────────────────────────────────────────────────────┘
  * 
  * @param channel: 0=扫描头, 1=面包板基准
  * @param servo_confidence: 本次测量的舵机机械可信度(0.0~1.0)，由主循环根据舵机状态计算
  * @param vibration_level: 双MPU差分得到的残余振动幅度(°/s，越大越不可信)
  */
uint16_t HCSR04_MeasureRobust(uint8_t channel,
                              GPIO_TypeDef* TrigPort, uint16_t TrigPin,
                              GPIO_TypeDef* EchoPort, uint16_t EchoPin,
                              float servo_confidence,
                              float vibration_level);

#endif /* __HCSR04_H__ */