<<<<<<< HEAD
# STM32 实验室 / 项目集

> 芯片：STM32F103C8T6 (Blue Pill / 最小系统板)  
> 开发环境：Keil MDK-ARM V5.32 / VSCode + CMake / STM32CubeIDE  
> 调试器：ST-Link V2

---

## 📁 目录一览

```
stm32-lab/
├── vscode_stm32/          # ✅ VSCode+CMake+HAL主工程：LED+OLED+编码器+PWM+串口
├── BuleUSCar/             # ✅ HAL库：蓝牙遥控小车 + 超声波避障（输入捕获测距）
├── FlashSystemDataProtect/ # ✅ HAL库：W25Q64 Flash断电保护按钮计数器
├── 智能风口系统/           # ✅ 标准库：舵机+电机+密码+LED控制（最复杂项目）
├── 智能小车/               # ✅ 标准库：5路红外循迹小车
├── roundoled/             # ✅ HAL库：OLED显示（硬件I2C）
├── stm32led/              # ✅ HAL库：LED控制
├── stm32_pc13/            # ✅ HAL库：板载PC13 LED闪烁
├── 蓝牙循迹小车/           # ❌ 未完成
├── LED/                   # ❌ CubeMX框架，未写代码
├── LED1/                  # ❌ HAL框架，需补充PB3-PB5
├── ledshark/              # ❌ 只有空循环
├── MPUFlASH/              # ❌ CubeMX框架
├── flowspeedledcontrol/   # ❌ CubeMX框架
├── stm新建工程模板-LED闪烁/  # ✅ 标准库
├── LED 流水灯/             # ✅ 标准库
├── 蜂鸣器/                 # ✅ 标准库
├── OLED显示屏/             # ✅ 标准库
├── PWM驱动LED呼吸灯/        # ✅ 标准库
├── 定时器定时中断/          # ✅ 标准库
├── 定时器外部时钟/          # ✅ 标准库
├── 输入捕获模式测频率/      # ✅ 标准库
├── PWMI模式测频率占空比/    # ✅ 标准库
├── 串口发送/               # ✅ 标准库
├── 串口发送+接受/          # ✅ 标准库
├── 串口收发HEX数据数据包/    # ✅ 标准库
├── 串口收发文本数据包/      # ❌ 未完成
├── 1-3 Delay函数模块/      # 🔧 工具库：延时函数（标准库版本）
├── Sydtem/                 # 🔧 工具库（标准库版本）
├── project/LED闪烁/        # 🔧 只有库文件，无代码
├── AI_DEBUG/               # 🖼 图片存档，不是代码
└── README.md               # ← 当前文档
```

---

## 🎯 核心项目说明（按完整度排序）

### 1. vscode_stm32 — VSCode + CMake + HAL 主工程

**项目效果：**  
3路LED（TIM1 PWM）控制亮度和动画模式，OLED（I2C）显示状态，旋转编码器（TIM3硬件编码器模式）调亮度/切模式，串口（USART1）定时上报+命令控制，支持板载3个按键直接切LED模式。

**亮点技术：**
| 技术点 | 实现方式 |
|---|---|
| OLED按需刷新 | `last_mode`/`last_bright`/`last_led_mode` dirty标记，仅在变化时刷新 |
| 非阻塞延时 | `HAL_GetTick()`时间戳，不卡死主循环 |
| TIM3硬件编码器 | 正交信号自动计数，不用软件判方向 |
| TIM1 PWM | 3路通道，CCR寄存器调占空比控制亮度 |
| TIM2定时中断 | 每2秒串口上报系统状态 |
| PB5编码器按键中断 | EXTI外部中断，切换编码器工作模式（默认/亮度/模式） |
| 串口命令 | `1`/`2`/`3`切LED模式，`0`关机，`+`/`-`调亮度 |

**踩过的坑：**
- CMake找不到源文件 → 绝对路径 `${CMAKE_CURRENT_SOURCE_DIR}/Hardware/OLED.c`
- main函数闭合的`}`丢了 → `MX_XXX_Init`全部报错"invalid storage class"
- PB5中断不触发 → STM32F1的AFIO_EXTICR需显式把EXTI5映射到GPIOB，PA0正常是因为默认映射
- 编码器按键坏了 → 物理短路，按不按都导通，直接读PB5永远低电平
- OLED刷新太频繁 → 每轮刷要200ms，按键检测被拖死，加dirty标记解决

**开发环境：**  
VSCode + CMake + Ninja，烧录用ST-Link（launch.json/tasks.json已配好）  
→ 详细烧录流程：`vscode_stm32/烧录说明.md`

---

### 2. 智能风口系统 — 最复杂的标准库项目

**项目效果：**  
通过串口发送密码控制风门开关（舵机）和电机正反转，带呼吸灯状态指示、紧急停止、密码锁机。

**核心功能：**
- **舵机**（PA0, TIM2_CH1）：0°~180°控制风门开合
- **电机**（PA1方向 + PB0方向 + PB6 PWM调速 + PA4使能）：正转进风/反转出风
- **LED指示**（PA6绿色呼吸灯/PA7红色报警灯）
- **紧急按钮**（PA3）：按下立即停机
- **串口协议**（115200bps）：`A密码E`验证密码，3次错误锁机10秒
- **密码**：默认 87654321

**代码量：** ~700行  
**库类型：** 标准外设库（不是HAL库）

---

### 3. FlashSystemDataProtect — Flash断电保存

**项目效果：**  
OLED显示按钮按下次数，断电时自动保存到W25Q64 Flash，重新上电后从上次的值继续累加。

**核心功能：**
- **OLED显示**：标题 / 当前次数 / 上次断电值 / Flash状态
- **W25Q64 Flash**（SPI接口PA4~PA7）：保存按钮计数
- **PA1按键**：+1 并自动保存到Flash
- **PA0按键**：归零并保存
- **Magic + 校验和**：判断Flash里存的数据是否有效

**特别的坑：**
1. **结构体对齐问题**：`uint8_t + uint32_t + uint8_t`用自然对齐后sizeof=9不是6，padding字节随机值导致校验和永远不一致 → 加 `__packed` 去掉对齐填充
2. **SPI读Flash ID**：SPI是全双工的，`HAL_SPI_Transmit`+`HAL_SPI_Receive`分两步会丢掉第一个有效字节 → 用 `HAL_SPI_TransmitReceive` 一次性收发
3. **OLED刷新阻塞**：每轮主循环刷OLED要200ms，按键被拖死 → 加dirty标记，只在按钮按下时刷

**最后状态：** 跑通，开发日志：`DEVELOPMENT_LOG.md`

---

### 4. 智能小车 — 5路红外循迹小车（标准库）

**项目效果：**  
用5路红外循迹传感器检测地面黑线，根据5路传感器的组合状态判断小车位置，每5ms执行一次循迹决策。

---

### 5. BuleUSCar — 蓝牙遥控小车（HAL库）

**项目效果：**  
蓝牙遥控小车 + 超声波自动避障。

**亮点：**
- **TIM1输入捕获测距**（上升沿+下降沿），比DWT时钟周期计时更精确
- **蓝牙通信**：手机APP发命令（F前进/L左转/R右转/S停止）
- **避障逻辑**：距离<20cm时强制停车，不响应蓝牙指令
- **HSI内部时钟**：没用外部晶振

**与其他蓝牙小车项目的区别：**
| 项目 | 测距方式 | OLED | 时钟 |
|---|---|---|---|
| BuleUSCar | TIM1输入捕获 | 无 | HSI内部 |
| bluetooth_OLED_Uitrasonic | DWT时钟周期 | 有 | HSE外部 |

---

### 6. LED / PWM / 定时器等基础训练项目

这些是跟着教程做的基础训练，用来掌握单个外设：

| 项目 | 学什么 | 状态 |
|---|---|---|
| LED 流水灯 | GPIO输出控制 | ✅ |
| 蜂鸣器 | GPIO推挽输出 | ✅ |
| OLED显示屏 | I2C通信 + 字库 | ✅ |
| PWM驱动LED呼吸灯 | PWM原理，占空比控制 | ✅ |
| 定时器定时中断 | SysTick/TIM中断 | ✅ |
| 定时器外部时钟 | 外部脉冲计数 | ✅ |
| 输入捕获模式测频率 | 捕获上升沿测频率 | ✅ |
| PWMI模式测频率占空比 | PWM输入模式 | ✅ |
| 串口发送/收发 | USART异步通信 | ✅ |
| 串口收发HEX/文本数据包 | 协议解析 | ✅(HEX) / ❌(文本) |

---

## 🔄 标准库 vs HAL库对比

工作区两种库的项目都有。整理如下：

| 维度 | 标准外设库 | HAL库 |
|---|---|---|
| 操作方式 | 直接操作寄存器结构体 | 用统一API函数 |
| 生成工具 | 手写 | STM32CubeMX图形配置生成 |
| 项目示例 | 智能风口系统/智能小车/LED流水灯... | vscode_stm32/BuleUSCar/FlashSystem... |
| 代码量 | 相对少 | 生成代码多，用户代码少 |
| 可读性 | 需要理解寄存器 | API语义清晰 |
| 新手友好度 | 偏底层，门槛高 | 有CubeMX图形工具，上手快 |
| 可移植性 | 同一芯片内可复用 | 跨芯片/跨平台好 |

---

## 🛠 开发工具链

- **STM32CubeMX 6.17.0**：图形化配置引脚/时钟/外设，生成初始化代码
- **Keil MDK-ARM V5.32**：标准库项目的IDE
- **VSCode + CMake + Ninja**：HAL库项目的IDE（vscode_stm32）
- **STM32CubeIDE**：也能用
- **ST-Link V2**：SWD方式烧录+调试
- **USB-TTL模块**：串口调试
- **串口调试助手**：PC端看串口数据

---

## 📐 标准流程：新建一个HAL库项目（vscode_stm32路子）

1. 打开STM32CubeMX，选STM32F103C8T6
2. 配置引脚/外设/时钟（SYSCLK=72MHz，HSE 8MHz×9）
3. Project Manager里选STM32CubeIDE或Makefile
4. 生成代码
5. 用VSCode打开，把初始化函数加到`/* USER CODE BEGIN 2 */`
6. 在`while(1)`里写主逻辑
7. 要加新驱动模块，文件放到`Hardware/`，CMakeLists.txt里加路径
8. Ctrl+Shift+B构建，F5烧录调试
9. 出问题先看串口输出，再看OLED调试信息

---

## 🎓 学习路线（回顾）

**入门阶段**：标准库项目
```
GPIO输出（LED/蜂鸣器）
  ↓
定时器中断/PWM（呼吸灯）
  ↓
I2C + OLED显示
  ↓
串口通信（发送/接收/协议解析）
  ↓
输入捕获（测频率/占空比）
```

**中级阶段**：HAL库项目
```
CubeMX配置流程掌握
  ↓
vscode_stm32（PWM+编码器+OLED+串口+EXTI中断综合应用）
  ↓
FlashSystemDataProtect（SPI Flash读写+数据校验+断电保存）
  ↓
BuleUSCar（蓝牙+超声波+PWM电机控制）
```

**高级阶段**：尚未完成的方向
```
RT-Thread/FreeRTOS（多任务调度）
  ↓
ADC/DAC（模拟信号处理）
  ↓
CAN通信（工业级）
  ↓
自定义上位机 + STM32 完整系统
```

---

## 🚫 常见坑 & 解决方法速查

| 问题 | 可能原因 | 解决方法 |
|---|---|---|
| 编译通过但板子上不跑 | 外设时钟没开 / 引脚没配置 / while(1)有死循环 | 用LED或串口一步步定位 |
| OLED只闪一下就黑 | OLED行号从0开始传了，`Line-1`溢出成255数组越界 | OLED行号从1开始 |
| 按键响应迟钝 | 主循环里有`HAL_Delay()`或刷新OLED等阻塞代码 | 加dirty标记，只在需要时刷新 |
| 串口收不到数据 | `HAL_UART_Receive_IT`没持续调用 / 回调没重开接收 | 回调函数末尾要再开一次接收 |
| Flash读回来全是FF | SPI引脚配置错 / 读ID方法错 / W25Q64没接3.3V | 检查引脚和SPI收发方式 |
| Flash保存的数据读不出来 | 结构体有对齐填充，校验和不一致 | 加`__packed` |
| 编码器旋转没反应 | TIM3编码器模式没正确使能 / 引脚接反 | 读`TIM3->CNT`寄存器看有没有变化 |
| PB5按键中断没反应 | AFIO_EXTICR没把EXTI5映射到GPIOB / 按键物理坏了 | 直接杜邦线短接PB5和GND测试 |
| 定时器溢出中断反复进入 | 启动了`HAL_TIM_Base_Start_IT`但没写`HAL_TIM_PeriodElapsedCallback` | 要么写回调，要么别开中断 |
| CMake找不到源文件 | 用相对路径 | 改成`${CMAKE_CURRENT_SOURCE_DIR}/xxx`绝对路径 |

---

## 📌 下一步计划

- 把蓝牙循迹小车和文本数据包这两个未完成的项目补完
- vscode_stm32可以加更多功能：比如OLED动画效果、多任务调度
- 想学RTOS可以在vscode_stm32基础上加FreeRTOS
- 编码器按键坏了可以换一个新的，或者改成板载按键切换模式

---

## 📖 更多细节

每个项目目录下都有 `项目总结.md`，里面有：
- 接线图引脚分配
- 项目效果描述
- 完成状态
- 用的是标准库还是HAL库
- 最后修改日期

vscode_stm32/目录下还有 `烧录说明.md`（VSCode烧录和调试指南）
=======
Welcome to my repository! It currently contains my learning projects for STM microcontrollers. Feel free to discuss and learn together with me. :)
— An embedded engineer on a journey of continuous learning.
欢迎来到我的项目仓库，仓库内的目前都是我的学习stm单片机的项目，欢迎跟我一起讨论交流学习：）
—— 一位正在努力学习的嵌入式工程师
>>>>>>> origin/main
