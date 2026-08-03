/**
  ******************************************************************************
  * @file    W25QXX.h
  * @brief   W25QXX 外部 Flash 驱动头文件
  *          来源: mpu-flash-template 项目优化
  *          优化: 标准化 API, 增加错误处理, 适配 atmosphere_tempture_sensor 项目
  ******************************************************************************
  */

#ifndef __W25QXX_H__
#define __W25QXX_H__

#include "main.h"
#include <stdint.h>
#include <string.h>

/* Flash 片选引脚 (使用 PA4, 避免与其他外设冲突) */
#define W25QXX_CS_PORT      GPIOA
#define W25QXX_CS_PIN       GPIO_PIN_4

/* Flash 型号定义 */
#define W25Q80_ID           0xEF13
#define W25Q16_ID           0xEF14
#define W25Q32_ID           0xEF15
#define W25Q64_ID           0xEF16
#define W25Q128_ID          0xEF17

/* Flash 参数 */
#define W25QXX_PAGE_SIZE    256         /* 页大小 (字节) */
#define W25QXX_SECTOR_SIZE  4096        /* 扇区大小 (4KB) */
#define W25QXX_BLOCK_SIZE   65536       /* 块大小 (64KB) */

/* Flash 指令集 */
#define W25X_WriteEnable    0x06
#define W25X_WriteDisable   0x04
#define W25X_ReadStatusReg  0x05
#define W25X_ReadData       0x03
#define W25X_PageProgram    0x02
#define W25X_SectorErase    0x20
#define W25X_BlockErase     0xD8
#define W25X_ChipErase      0xC7
#define W25X_DeviceID       0xAB
#define W25X_ManufactID     0x90
#define W25X_JedecID        0x9F
#define W25X_BusyFlag       0x01

/* 错误码 */
#define W25QXX_OK           0
#define W25QXX_ERR_TIMEOUT  1
#define W25QXX_ERR_ID       2
#define W25QXX_ERR_VERIFY   3

/* 函数声明 */
uint8_t     W25QXX_Init(void);
uint16_t    W25QXX_ReadID(void);
uint8_t     W25QXX_ReadSR(void);
void        W25QXX_WaitBusy(void);
uint8_t     W25QXX_Read(uint32_t addr, uint8_t* buf, uint16_t len);
uint8_t     W25QXX_Write(uint32_t addr, uint8_t* buf, uint16_t len);
uint8_t     W25QXX_SectorErase(uint32_t addr);
uint8_t     W25QXX_ChipErase(void);

#endif /* __W25QXX_H__ */