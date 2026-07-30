/**
  ******************************************************************************
  * @file    OLED.h
  * @brief   SSD1306 OLED 驱动头文件 (128x64, I2C 接口)
  *          
  * 【模块说明】
  *   SSD1306 是一款常用的 128x64 像素单色 OLED 显示驱动芯片，支持 I2C/SPI 接口。
  *   本项目使用 I2C 接口，通过 hi2c1 总线与 OLED 通信。
  *   
  * 【显示原理】
  *   - OLED 显存分为 8 页（Page 0~7），每页 128 列
  *   - 每页的 8 行对应 1 个字节的 8 个 bit
  *   - 写入数据时，先写命令（0x00 前缀），再写数据（0x40 前缀）
  *   
  * 【本项目应用】
  *   - 显示 MPU6050 姿态角数据
  *   - 显示超声波测距结果
  *   - 显示 ESP32 传来的温湿度数据
  *   - 显示初始化状态（MPU ID）
  *   
  * 【优化说明】
  *   1. 支持任意 I2C 句柄（不绑定特定 I2C 总线）
  *   2. 增加显存缓冲区（GRAM），先写入缓冲区再一次性刷新到 OLED
  *   3. 支持 6x8 点阵字符显示
  *   4. 支持数字、浮点数显示
  ******************************************************************************
  */

#ifndef __OLED_H__
#define __OLED_H__

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ==================== OLED I2C 地址 ==================== */
#define OLED_I2C_ADDR       0x78  /* 7 位地址 0x3C 左移 1 位 */

/* ==================== OLED 尺寸参数 ==================== */
#define OLED_WIDTH          128   /* 宽度 128 像素 */
#define OLED_HEIGHT         64    /* 高度 64 像素 */
#define OLED_PAGE_NUM       8     /* 页数：64/8=8 页 */

/* ==================== 函数声明 ==================== */

/**
  * @brief  初始化 OLED 显示屏
  * @param  hi2c: I2C 句柄指针
  * @retval 无
  * @note   发送初始化命令序列，开启显示，清屏
  */
void        OLED_Init(I2C_HandleTypeDef* hi2c);

/**
  * @brief  清空显存（全部写 0）
  * @param  hi2c: I2C 句柄指针
  * @retval 无
  * @note   只清空缓冲区，需调用 OLED_Refresh 才能显示到屏幕
  */
void        OLED_Clear(I2C_HandleTypeDef* hi2c);

/**
  * @brief  刷新显存到 OLED 屏幕
  * @param  hi2c: I2C 句柄指针
  * @retval 无
  * @note   将 GRAM 缓冲区数据逐页发送到 OLED
  */
void        OLED_Refresh(I2C_HandleTypeDef* hi2c);

/**
  * @brief  在指定坐标画点
  * @param  hi2c: I2C 句柄指针
  * @param  x: X 坐标（0~127）
  * @param  y: Y 坐标（0~63）
  * @param  color: 颜色（1=亮，0=灭）
  * @retval 无
  */
void        OLED_DrawPoint(I2C_HandleTypeDef* hi2c, uint8_t x, uint8_t y, uint8_t color);

/**
  * @brief  显示单个字符
  * @param  hi2c: I2C 句柄指针
  * @param  x: X 坐标（0~126）
  * @param  y: Y 坐标（0~56）
  * @param  chr: 字符（ASCII 码）
  * @param  size: 字符大小（当前仅支持 6）
  * @retval 无
  */
void        OLED_ShowChar(I2C_HandleTypeDef* hi2c, uint8_t x, uint8_t y, uint8_t chr, uint8_t size);

/**
  * @brief  显示字符串
  * @param  hi2c: I2C 句柄指针
  * @param  x: 起始 X 坐标
  * @param  y: 起始 Y 坐标
  * @param  str: 字符串指针（仅支持 ASCII 字符）
  * @retval 无
  * @note   自动换行，超出屏幕时停止显示
  */
void        OLED_ShowString(I2C_HandleTypeDef* hi2c, uint8_t x, uint8_t y, char *str);

/**
  * @brief  显示无符号数字
  * @param  hi2c: I2C 句柄指针
  * @param  x: X 坐标
  * @param  y: Y 坐标
  * @param  num: 要显示的数字
  * @param  len: 显示位数
  * @retval 无
  */
void        OLED_ShowNum(I2C_HandleTypeDef* hi2c, uint8_t x, uint8_t y, uint32_t num, uint8_t len);

/**
  * @brief  显示浮点数
  * @param  hi2c: I2C 句柄指针
  * @param  x: X 坐标
  * @param  y: Y 坐标
  * @param  num: 要显示的浮点数
  * @param  decimal: 小数位数
  * @retval 无
  */
void        OLED_ShowFloat(I2C_HandleTypeDef* hi2c, uint8_t x, uint8_t y, float num, uint8_t decimal);

/**
  * @brief  显示带符号数字
  * @param  hi2c: I2C 句柄指针
  * @param  x: X 坐标
  * @param  y: Y 坐标
  * @param  num: 要显示的数字（可正可负）
  * @param  len: 显示位数
  * @retval 无
  */
void        OLED_ShowSignedNum(I2C_HandleTypeDef* hi2c, uint8_t x, uint8_t y, int32_t num, uint8_t len);

/**
  * @brief  绘制直线（Bresenham算法）
  * @param  hi2c: I2C 句柄指针
  * @param  x1,y1: 起点坐标
  * @param  x2,y2: 终点坐标
  * @param  color: 1=亮 0=灭
  * @retval 无
  * @note   新增：用于绘制雷达扫描线、扇形边框等
  */
void        OLED_DrawLine(I2C_HandleTypeDef* hi2c, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color);

/**
  * @brief  绘制圆弧（上半圆，用于雷达扇形边界）
  * @param  hi2c: I2C 句柄指针
  * @param  cx,cy: 圆心坐标
  * @param  r: 半径
  * @param  color: 1=亮 0=灭
  * @retval 无
  * @note   新增：雷达扫描的扇形参考圆，只绘制上半圆（y <= cy）
  */
void        OLED_DrawSemicircle(I2C_HandleTypeDef* hi2c, uint8_t cx, uint8_t cy, uint8_t r, uint8_t color);

#endif /* __OLED_H__ */