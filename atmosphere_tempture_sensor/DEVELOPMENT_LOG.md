# 开发日志 — 室内外空气质量监测系统 (STM32F103 + ESP32-S3)

> 项目路径：
> - STM32 端：`G:\stm32project\atmosphere_tempture_sensor`（Keil MDK-ARM V5.32, STM32CubeMX 6.17.0）
> - ESP32 端：`G:\esp32project\hello_world`（ESP-IDF v6.0.2, ESP32-S3）
> - 最近更新：2026-08-01

---

## 一、项目概述

基于 **STM32F103C8T6 + ESP32-S3** 的室内外空气质量监测系统：

- **STM32**（采集/显示端）：MQ-135（CO2/CO/VOC 估算）+ **AHT20（温湿度）+ BMP280（气压/温度）**；OLED 显示（3 页循环）；USART1 与 ESP32 通信；USART3 接 VOFA+ 调试上位机；W25QXX Flash 存储校准值。
- **ESP32**（云端分析端）：UART1 接收 STM32 数据；WiFi 获取 OpenWeatherMap 室外 AQI；室内外对比生成建议回传；内置 Web 服务器（HTTP API + 实时页面）。

```
MQ-135 + AHT20 + BMP280 ──→ STM32(采集/OLED/VOFA+) ──UART 115200──→ ESP32-S3
                                                                     │ WiFi 室外 AQI
                                                                     │ 对比分析 → 建议
                                                                     ↓
ESP32-S3 ──UART 115200──→ STM32 ──→ OLED 显示建议 + 数据
```

## 二、硬件接线

### STM32 外设
| 外设 | 引脚 | 说明 |
|------|------|------|
| ADC1 | PA0 (ADC1_IN0) | MQ-135 AO 输出 |
| I2C1 | PB6 (SCL), PB7 (SDA) | OLED(0x3C) + AHT20(0x38) + BMP280(0x76/0x77) 共总线 |
| SPI1 | PA4(NSS) PA5(SCK) PA6(MISO) PA7(MOSI) | W25QXX Flash |
| USART1 | PA9 (TX), PA10 (RX) | 与 ESP32 通信, 115200 8N1 |
| USART3 | PB10 (TX), PB11 (RX) | VOFA+ 调试上位机, 115200 |

### ESP32-S3 ↔ STM32 连接（GPIO14/15）
| ESP32-S3 | 连接 | STM32 |
|----------|------|-------|
| GPIO14 (TX) | ──→ | PA10 (USART1_RX) |
| GPIO15 (RX) | ←── | PA9 (USART1_TX) |
| GND | ──→ | GND（必须共地） |

> ⚠️ 原 GPIO17/16 因排针反复插拔接触不良导致通信反复中断，已更换为 GPIO14/15。

### MQ-135 板载负载电阻（实测）
万用表 20K 档测 **A0-GND ≈ 4.3~4.5 kΩ**，代码中 `MQ135_RLOAD = 4.4f`（kΩ）。
> ⚠️ 每个 MQ135 模块 RLOAD 不同，务必实测后修改 `MQ135.h`。

## 三、通信协议（当前）

### STM32 → ESP32（每 2 秒一帧，含环境数据）
```
MQ135:ADC=171,V=0.138,RS=148.73,RSR=1.227,CO2=307.95,CO=5.39,ALC=4.20,TOL=3.64,NH4=4.81,ACE=2.97,AQ=0,TEMP=29.34,HUMI=83.74,PRES=1006.28\r\n
```
- ADC/V/RS/RSR：MQ135 原始计算；CO2/CO/ALC/TOL/NH4/ACE：六种气体估算；AQ：等级(0-5)
- TEMP/HUMI/PRES：AHT20/BMP280 实测环境数据

### ESP32 → STM32（每条间隔 20ms，避免 STM32 缓冲覆盖）
```
OUTDOOR:CO2=400.00,CO=0.10,ALC=1.00,TOL=1.00,NH4=1.00,ACE=1.00\r\n   （室外数据，每 2 秒补发）
ADVICE:Humid 84%! Ventilate to reduce\r\n                            （建议文本）
FUSION:CO2=307.95,CO=5.39,ALC=4.20,TOL=3.64,NH4=4.81,ACE=2.97,AQ=0\r\n（融合回显）
```

## 四、软件实现

### STM32 端（Core/Src/main.c + Hardware/）
- 主循环每 2 秒：ADC 8 次平均 → Rs → **温湿度补偿**（AHT20/BMP280 实测）→ Rs/R0 → 六气体 → 5 次滑动平均 → 等级
- **OLED 3 页循环**（每 5 秒）：页1 气体（等级/CO2/CO）、页2 VOC（酒精/甲苯/氨气/丙酮）、页3 环境（温度/湿度/气压）；**建议界面**（建议 + CO2 + 温湿度）与传感器页交替 10 秒
- **VOFA+ 调试串口**（USART3）：STM32 原始计算（ADC/V/Rs/RSR/R0）+ 室内六气体 + 环境 + **ESP32 融合回显对比** + 室外数据 + 室内外对比 + 建议
- USART1 接收中断逐字节缓冲，主循环解析 FUSION/OUTDOOR（**不在中断做 I2C**）
- 上电校准：Flash 读 R0（magic+checksum）→ 有效免校准；无效则 10 秒热机 + 校准写 Flash；USART3 收 '1' 可强制重校

### ESP32 端（main/hello_world_main.c）
- `mq135_processing_task`（优先级 5）：UART 行缓冲接收 → 解析（含 TEMP/HUMI/PRES）→ 发 OUTDOOR/ADVICE/FUSION（消息间 20ms 间隔）
- `outdoor_update_task`（优先级 4）：WiFi + 每 5 分钟拉取室外 AQI（HTTP，cJSON 判空）；失败用 mock 数据
- **Web 服务器**（端口 80）：`/`（HTML 页面）、`/api/indoor`、`/api/outdoor`、`/api/compare`、`/api/all`（JSON）
- 建议算法：CO2 差 >200 → 通风；CO2>1000 → 强制通风；湿度>75% → 除湿通风；温度>32℃ → 降温通风

## 五、开发历程与关键问题修复

| # | 问题 | 根因 | 修复 |
|---|------|------|------|
| 1 | ESP32 完全无响应 | 卡在 DOWNLOAD 下载模式，固件未运行 | 重烧 + DTR/RTS 复位 |
| 2 | STM32 收不到 ESP32 | USART1_IRQHandler 缺失 + NVIC 未使能 | 补 IRQHandler + EnableIRQ |
| 3 | 中断里做 I2C 冲突 | 回调里阻塞 OLED 操作 | 回调只缓冲置标志 |
| 4 | ESP32 崩溃重启 | cJSON NULL 解引用 | 全字段判空 |
| 5 | WiFi 阻塞通信 | wifi_init 在 app_main 阻塞 | 移入后台任务 |
| 6 | UART 半帧丢数据 | 整帧一次性解析 | 行缓冲 |
| 7 | PPM 仅 5~22 | R0 未校准 + SCONE 旧值 | 自动校准 + SCONE=400 |
| 8 | 300ppm 误判 Hazardous | 阈值按旧标定 | 真实 CO2 分级 |
| 9 | 数据周期变 5.7 秒 | 建议显示期间暂停采样 | 采样/发送解耦 |
| 10 | CO2 恒定 357 不变 | 气体公式 `powf(ratio/a, 1/b)` 错误 | 修正为 `a*powf(ratio, b)` |
| 11 | 数值爆表(CO=1000) | 多气体曲线指数过陡 + b 硬编码 | 参数保守化 + b 引用宏 |
| 12 | OLED 只显示一页 | ESP32 建议霸屏 / 单页设计 | 建议/传感器 10s 交替 + 3 页循环 |
| 13 | ESP32 连发消息被覆盖 | 3 条消息无间隔，STM32 缓冲覆盖 | 消息间加 20ms 间隔 |
| 14 | GPIO17/16 反复断连 | 排针接触不良 + GND 不稳 | 换 GPIO14/15 |
| 15 | HTTPS 请求失败 | TLS 证书未配置 | 改 http:// |
| 16 | 首次 fetch 等 5 分钟 | last_outdoor_update 初始化问题 | 首次立即 fetch |
| 17 | API Key 抄错 | a/e 首字母错误 | 修正 key |
| 18 | W25QXX CS 失效 | PA4 配成 SPI1_NSS 复用(AF) | W25QXX_Init 重配 GPIO 输出 |

## 六、实测效果（2026-08-01 串口监测）

```
I (461975) Received: MQ135:ADC=322,...,CO2=810.40,CO=1000.00,...,TEMP=29.07,HUMI=83.74,PRES=1006.00
I (461998) Sent advice: ADVICE:CO2 high! Indoor 810 vs Outdoor 400. Ventilate!
I (462007) Sent fusion: FUSION:CO2=810.40,...,AQ=1
```
- 新参数修正后：CO2 约 300~600（ratio 1.0~1.8），CO/酒精/甲苯/氨气/丙酮 2~6（不再爆表）
- 温湿度/气压：AHT20+BMP280 实测（29.3℃ / 83.7% / 1006 hPa）
- 建议逻辑：CO2 高 → 通风；湿度高 → 除湿通风；室内外对比
- 全链路 2 秒周期稳定，双向通信正常（GPIO14/15）

## 七、借鉴的开源项目

| 项目 | 借鉴点 |
|------|--------|
| [Bobbo117/MQ135-Air-Quality-Sensor](https://github.com/Bobbo117/MQ135-Air-Quality-Sensor) | RLOAD 实测、室外校准 R0、滚动平均 |
| [StratoSense/Air-Quality-Monitoring-System](https://github.com/StratoSense/Air-Quality-Monitoring-System) | 多传感器融合 + OLED + 云平台 |
| [maramroueched/IoT-air-quality-ST32-ESP32](https://github.com/maramroueched/-Iot-air-quality-and-cry-detection-system-stm32-esp32) | STM32+ESP32 双 MCU 架构 |

## 八、后续扩展方向

1. **NDIR CO2 传感器（MH-Z19）**：替代 MQ135 获取真 CO2 精度（MQ135 本质为定性趋势传感器）
2. **云端历史曲线**：ESP32 上报 ThingSpeak / Blynk
3. **继电器自动通风**：根据建议自动控制排风扇/净化器
4. **室外温湿度**：调用 OpenWeatherMap weather API 补齐
5. **SNTP 时间同步**：修复 Web 时间戳 1970 问题
6. **MQTT 接入**：Home Assistant 等智能家居

## 九、环境与构建

- STM32：Keil MDK-ARM V5.32 打开 `MDK-ARM\atmosphere_tempture_sensor.uvprojx`，Rebuild + Download（ST-Link）
- ESP32：`cd G:\esp32project\hello_world && idf.py -p COM口 flash`（ESP-IDF v6.0.2，目标 esp32s3）
- 串口调试：ESP32 日志经原生 USB（COM18, USB-Serial-JTAG）输出；STM32 调试经 USART3（COM16, CH340, VOFA+）
- Web 界面：`http://192.168.1.21`（ESP32 IP）
