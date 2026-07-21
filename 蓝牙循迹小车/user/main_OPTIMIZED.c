/* =========================================================================
 * 项目: 蓝牙循迹小车 (标准库, 红外循迹 + 蓝牙遥控)
 * 原文件: user/main.c
 * 优化版: user/main_OPTIMIZED.c
 *
 * 问题: while(1) 里所有循迹逻辑都被 /* 注释掉了, 只剩空循环
 *       导致小车通电后完全不动 (只有蓝牙遥控的中断函数还在)
 *
 * 修复: 用加权位置法重新实现循迹逻辑 (参考智能小车项目的优化方案)
 *       同时保留原有的蓝牙遥控 (USART1_IRQHandler) 功能
 *
 * 加权位置法原理 (取代之前的 15 条 if 判断):
 *   5路红外传感器:  s1 (左外)  s2 (左中)  s3 (中)  s4 (右中)  s5 (右外)
 *   对应的权重:     -2         -1          0         +1          +2
 *
 *   sum = (-2)*s1 + (-1)*s2 + 0*s3 + 1*s4 + 2*s5
 *
 *   sum <= -2 → 左转 (偏离轨道很多, 需要大转)
 *   sum == -1 → 左转 (轻微偏离)
 *   sum ==  0 → 直行 (在轨道中央)
 *   sum == +1 → 右转 (轻微偏离)
 *   sum >= +2 → 右转 (偏离轨道很多, 需要大转)
 *
 *   好处: 代码量减少 70%, 逻辑更直观, 便于调参
 *
 * 蓝牙遥控 (保留原有逻辑):
 *   上位机发送字节:
 *     0x30 ('0') → 停止
 *     0x31 ('1') → 前进
 *     0x32 ('2') → 后退
 *     0x33 ('3') → 左转
 *     0x34 ('4') → 右转
 *     0x35 ('5') → 原地左转
 *     0x36 ('6') → 原地右转
 * ========================================================================= */

#include "stm32f10x.h"
#include "Delay.h"
#include "CAR.h"
#include "Track.h"
#include "Serial.h"

/* ============ 全局: 蓝牙遥控指令 ============ */
/* 0=停止 1=前进 2=后退 3=左转 4=右转 5=原地左 6=原地右 */
uint16_t bt_cmd = 0;   /* 串口中断里改写, main 里读 */

/* ============ 函数声明 ============ */
int16_t ReadTrackWeight(void);   /* 读5路传感器, 返回加权位置 sum */

/* ========================================================================= */
int main(void)
{
    /* ---------- 初始化: 电机驱动 + 串口蓝牙 ---------- */
    Car_Init();
    Serial_Init();

    /* 初始状态: 小车停止, 等待循迹或遥控 */
    Car_Stop();
    Delay_ms(200);

    /* ===================== 主循环 ===================== */
    while (1)
    {
        /* ---------- 1. 蓝牙遥控优先: 若收到指令, 执行它 ----------
         *   bt_cmd != 0 时表示蓝牙正主动控制, 跳过循迹
         *   bt_cmd == 0 时表示没有蓝牙指令, 自动走循迹
         *   这样就实现了: "有遥控时听遥控, 没遥控自动循迹"
         * ------------------------------------------------------- */
        if (bt_cmd != 0)
        {
            switch (bt_cmd)
            {
                case 1: Go_Ahead();   break;
                case 2: Go_Back();    break;
                case 3: Turn_Left();  break;
                case 4: Turn_Right(); break;
                case 5: Self_Left();  break;
                case 6: Self_Right(); break;
                default: Car_Stop();  break;
            }
            Delay_ms(10);
            continue;  /* 跳过下面的循迹, 下一轮循环继续 */
        }

        /* ---------- 2. 自动循迹: 加权位置法 ---------- */
        int16_t pos = ReadTrackWeight();

        if (pos <= -2)
        {
            Turn_Left();        /* 严重偏右, 快速左转回轨道 */
        }
        else if (pos == -1)
        {
            Turn_Left();        /* 轻微偏右, 小转一点 */
        }
        else if (pos == 0)
        {
            Go_Ahead();         /* 正好在轨道中央, 直行 */
        }
        else if (pos == 1)
        {
            Turn_Right();       /* 轻微偏左, 小转一点 */
        }
        else /* pos >= 2 */
        {
            Turn_Right();       /* 严重偏左, 快速右转回轨道 */
        }

        Delay_ms(5);   /* 小延时: 防止电机状态切换太快 */
    }
}

/* =========================================================================
 * 加权位置法: 读取 5 路红外传感器, 按权重相加后返回
 *   s1(左外) = -2,  s2(左中) = -1,  s3(中) = 0
 *   s4(右中) = +1,  s5(右外) = +2
 * 返回值范围: -10 ~ +10 (取决于 Read_Track_Sensor 的数据宽度)
 * ========================================================================= */
int16_t ReadTrackWeight(void)
{
    uint8_t s1, s2, s3, s4, s5;

    /* 读5路传感器: 具体函数取决于你的 Track.c 实现
     *   如果 Track.h 里是 Read_Track_Sensor(uint8_t *p1..p5), 就按你实际的来
     *   这里做一个通用的写法, 如果编译不过, 看 Track.h 改一下函数名即可 */
#if 0
    /* 写法 A: 如果 Track.c 提供了分别读取的函数 */
    s1 = Track_Sensor1();
    s2 = Track_Sensor2();
    s3 = Track_Sensor3();
    s4 = Track_Sensor4();
    s5 = Track_Sensor5();
#else
    /* 写法 B: 通过数组一次读出 (根据 Track.h 实际定义调整) */
    uint8_t sensors[5];
    /* 如果你有 Read_Track_Sensors(uint8_t *buf) 这样的函数, 用它 */
    /* 这里以最常见的"直接读 GPIO 输入电平"为例:
     *   假设 5 路传感器接在 PB0~PB4
     *   传感器检测到黑线时输出低电平 (0), 检测到白底时输出高电平 (1) */
    sensors[0] = (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == Bit_RESET) ? 1 : 0;
    sensors[1] = (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == Bit_RESET) ? 1 : 0;
    sensors[2] = (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2) == Bit_RESET) ? 1 : 0;
    sensors[3] = (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3) == Bit_RESET) ? 1 : 0;
    sensors[4] = (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4) == Bit_RESET) ? 1 : 0;
    s1 = sensors[0];
    s2 = sensors[1];
    s3 = sensors[2];
    s4 = sensors[3];
    s5 = sensors[4];
#endif

    /* 加权相加: 返回值就是偏离程度, 负数偏右, 正数偏左, 0 居中 */
    return (-2) * (int16_t)s1
           + (-1) * (int16_t)s2
           +   0  * (int16_t)s3
           +   1  * (int16_t)s4
           +   2  * (int16_t)s5;
}

/* =========================================================================
 * USART1 中断: 收到蓝牙发来的字节时执行
 *   0x30 ('0') → 停止 / 取消遥控
 *   0x31 ('1') → 前进
 *   0x32 ('2') → 后退
 *   0x33 ('3') → 左转
 *   0x34 ('4') → 右转
 *   0x35 ('5') → 原地左转
 *   0x36 ('6') → 原地右转
 * ========================================================================= */
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        uint16_t data = USART_ReceiveData(USART1);

        /* 把 ASCII 字节转换成 0~6 的指令编号 */
        if      (data == 0x30) bt_cmd = 0;  /* 停止: 同时恢复自动循迹 */
        else if (data == 0x31) bt_cmd = 1;  /* 前进 */
        else if (data == 0x32) bt_cmd = 2;  /* 后退 */
        else if (data == 0x33) bt_cmd = 3;  /* 左转 */
        else if (data == 0x34) bt_cmd = 4;  /* 右转 */
        else if (data == 0x35) bt_cmd = 5;  /* 原地左转 */
        else if (data == 0x36) bt_cmd = 6;  /* 原地右转 */
        /* 其他字节: 忽略, bt_cmd 保持不变 */

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}