# 开发日志 — 室内外空气质量监测系统 (STM32F103 + ESP32-S3)

> 项目路径：
> - STM32 端：`G:\stm32project\atmosphere_tempture_sensor`（Keil MDK-ARM V5.32, STM32CubeMX 6.17.0）
> - ESP32 端：`G:\esp32project\hello_world`（ESP-IDF v6.0.2, ESP32-S3）
> - 完成日期：2026-07-31

---

## 一、项目概述

基于 **STM32F103C8T6 + ESP32-S3** 的室内外空气质量监测系统：

- **STM32**（传感器采集端）：驱动 MQ-135 传感器（ADC 采集 CO2/CO），OLED (SSD1306, I2C) 显示；USART1 与 ESP32 通信；W25QXX Flash 存储 MQ135 校准值。
- **ESP32**（云端分析端）：UART1 接收 STM32 数据；WiFi 连接 OpenWeatherMap 获取室外 AQI；分析室内外差异，生成通风建议发回 STM32 显示。

```
MQ-135 ──→ STM32(采集/OLED) ──UART 115200──→ ESP32-S3
                                                │ WiFi 获取室外 AQI (OpenWeatherMap)
                                                │ 分析室内外差异 → 生成建议
                                                ↓
ESP32-S3 ──UART 115200──→ STM32 ──→ OLED 显示建议
```

## 二、硬件接线

### STM32 外设
| 外设 | 引脚 | 说明 |
|------|------|------|
| ADC1 | PA0 (ADC1_IN0) | MQ-135 AO 输出 |
| I2C1 | PB6 (SCL), PB7 (SDA) | SSD1306 OLED (0x3C) |
| SPI1 | PA4(NSS) PA5(SCK) PA6(MISO) PA7(MOSI) | W25QXX Flash |
| USART1 | PA9 (TX), PA10 (RX) | 与 ESP32 通信, 115200 8N1 |

### ESP32-S3 ↔ STM32 连接
| ESP32-S3 | 连接 | STM32 |
|----------|------|-------|
| GPIO4 (TX) | ──→ | PA10 (USART1_RX) |
| GPIO5 (RX) | ←── | PA9 (USART1_TX) |
| GND | ──→ | GND（必须共地） |

### MQ-135 板载负载电阻（实测）
万用表 20K 档测 **A0-GND ≈ 4.3~4.5 kΩ**，代码中 `MQ135_RLOAD = 4.4f`（kΩ）。
> ⚠️ 每个 MQ135 模块 RLOAD 不同，务必实测后修改 `MQ135.h`。

## 三、通信协议

### STM32 → ESP32（每 2 秒一帧）
```
MQ135:PPM=433.37,TEMP=25.00,HUMI=50.00,AQI=433.37,LEVEL=Excellent\r\n
```
- TEMP/HUMI 为预留字段（当前固定 25.00/50.00，后续可接 DHT11/DHT22 提供真实值）

### ESP32 → STM32（建议文本，`\n` 结尾）
```
Ventilate room now! Open windows!\n
```

## 四、软件实现

### STM32 端（Core/Src/main.c + Hardware/MQ135.c）
- 主循环每 2 秒采样：ADC 8 次平均 → Rs/R0 计算 → CO2/CO 浓度 → **最近 5 次滑动平均** → 等级判定 → OLED 显示 + UART 发送
- 建议界面：OLED 上部显示 ESP32 建议（最多 3 行），底部同屏显示 CO2/CO/Level
- USART1 接收中断：逐字节缓冲，遇 `\n` 置标志，主循环显示（**不在中断里做 I2C**）
- 上电校准：W25QXX 读取上次校准 R0（magic + checksum 校验）→ 有效则免校准；无效则 10 秒热机 + 10 次采样校准，结果写入 Flash

### ESP32 端（main/hello_world_main.c）
- `stm32_communication_task`（优先级 5）：UART 行缓冲接收 → 解析 → 分析 → 回发建议
- `outdoor_data_fetch_task`（优先级 4）：WiFi 连接 + 每 10 分钟拉取室外 AQI（OpenWeatherMap），cJSON 解析（全字段判空防崩溃）
- WiFi 连接移入室外任务，**不阻塞 STM32 通信**
- 建议算法：综合室内 AQI 分级 + 室内外差异（通风评分）

## 五、开发历程与关键问题修复

| # | 问题 | 根因 | 修复 |
|---|------|------|------|
| 1 | ESP32 完全无响应，OLED 只有 "Suggestion:" 无内容 | ESP32 上次烧录后卡在 **DOWNLOAD 下载模式**（`boot:0x0 waiting for download`），固件从未运行 | 重新烧录 + DTR/RTS 复位从 flash 启动 |
| 2 | STM32 无法接收 ESP32 数据 | `USART1_IRQHandler` 缺失（弱定义死循环）+ NVIC 未使能 | `stm32f1xx_it.c` 补 IRQHandler，`HAL_NVIC_EnableIRQ` |
| 3 | OLED 显示乱码/与 I2C 冲突 | 中断回调里做阻塞 I2C 操作 | 回调只缓冲字节+置标志，主循环显示 |
| 4 | 建议被传感器界面瞬间覆盖 | 显示无优先级 | 建议优先显示停留 5 秒，底部同时显示传感器数据 |
| 5 | ESP32 偶发崩溃重启 | cJSON 解析 `->valueint` 未判空，API 异常时 NULL 解引用 | 全部字段判空 |
| 6 | WiFi 连不上时 ESP32 不响应 STM32 | `wifi_init_sta()` 阻塞在 app_main，通信任务未创建 | WiFi 移入后台任务 |
| 7 | UART 半帧丢数据 | 一次 read 整帧解析 | 逐字节行缓冲，遇 `\r\n` 再解析 |
| 8 | PPM 仅 5~22（正常应 400） | R0 未校准 + 曲线参数 SCONE=116.6 是旧库值 | 上电自动校准 R0 + `SCONE=400`（清洁空气≈400ppm） |
| 9 | 300ppm 被误判 Hazardous | 等级阈值按旧低 ppm 标定 | 按真实 CO2 分级（<600/1000/1500/2000/3000） |
| 10 | 数据更新周期变 5.7 秒 | 建议显示 5 秒期间暂停采样 | 采样/发送与 OLED 显示解耦，恒定 2 秒 |
| 11 | 通风评分算而不用 | 建议文本只看 indoor_aqi | 建议算法整合通风评分（室内外差异） |
| 12 | PPM 抖动 ±30 | 单次采样 | ADC 8 次平均 + 5 次滑动平均 |

## 六、实测效果（2026-07-31 串口监测）

```
I (21566) Received from STM32: MQ135:PPM=439.72,TEMP=25.00,HUMI=50.00,AQI=439.72,LEVEL=Excellent
I (21576) Indoor AQI: 53 | Outdoor AQI: 1 | Difference: 52
I (21576) Ventilation Score: 90% | Health Level: 2 - Good
I (21596) Suggestion: Ventilate room now! Open windows!
```

- 室内 CO2 稳定在 **407~440 ppm**（校准后真实值，波动 ±15 以内）
- 室外 AQI=1（东莞凤岗，OpenWeatherMap）
- 室内外差异 → 通风评分 90% → 建议通风（逻辑正确）
- 全链路 2 秒周期稳定运行，无崩溃、无丢帧

## 七、借鉴的开源项目

| 项目 | 借鉴点 |
|------|--------|
| [Bobbo117/MQ135-Air-Quality-Sensor](https://github.com/Bobbo117/MQ135-Air-Quality-Sensor) | RLOAD 实测、室外校准 R0、滚动平均、48h 热机 |
| [StratoSense/Air-Quality-Monitoring-System](https://github.com/StratoSense/Air-Quality-Monitoring-System) | 多传感器融合 + OLED AQI + 云平台方向 |
| [maramroueched/IoT-air-quality-ST32-ESP32](https://github.com/maramroueched/-Iot-air-quality-and-cry-detection-system-stm32-esp32) | STM32+ESP32 双 MCU 架构 |

## 八、后续扩展方向

1. **DHT11/DHT22 真实温湿度**：替换固定 TEMP/HUMI，MQ135 读数随温湿度修正
2. **云端历史曲线**：ESP32 上报 ThingSpeak / Blynk
3. **继电器自动通风**：根据建议自动控制排风扇/净化器
4. **Web 界面**：ESP32 内置 HTTP 服务器显示实时曲线
5. **更多气体**：MQ-2/MQ-7（烟雾/CO），PMS5003（PM2.5）
6. **重新校准入口**：当前无按键，校准值存 Flash 后需擦除或改码才重校

## 九、环境与构建

- STM32：Keil MDK-ARM V5.32 打开 `MDK-ARM\atmosphere_tempture_sensor.uvprojx`，Rebuild + Download（ST-Link）
- ESP32：`cd G:\esp32project\hello_world && idf.py -p COM口 flash`（ESP-IDF v6.0.2，目标 esp32s3）
- 串口调试：ESP32 日志经 USB 串口 (COM17, CH343) 输出，115200
