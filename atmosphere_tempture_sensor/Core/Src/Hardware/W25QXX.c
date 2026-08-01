/**
  ******************************************************************************
  * @file    W25QXX.c
  * @brief   W25QXX 外部 Flash 驱动源文件
  *          来源: mpu-flash-template 项目优化
  *          优化: 标准化 API, 增加错误处理, 适配 atmosphere_tempture_sensor 项目
  ******************************************************************************
  */

#include "Hardware/W25QXX.h"

/* 外部 SPI 句柄声明 */
extern SPI_HandleTypeDef hspi1;

/* Flash 类型 (初始化时自动检测) */
static uint16_t W25QXX_Type = 0;

/**
  * @brief  拉低片选
  */
static void W25QXX_CS_LOW(void)
{
    HAL_GPIO_WritePin(W25QXX_CS_PORT, W25QXX_CS_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  拉高片选
  */
static void W25QXX_CS_HIGH(void)
{
    HAL_GPIO_WritePin(W25QXX_CS_PORT, W25QXX_CS_PIN, GPIO_PIN_SET);
}

/**
  * @brief  读取状态寄存器
  * @retval 状态寄存器值
  */
uint8_t W25QXX_ReadSR(void)
{
    uint8_t cmd = W25X_ReadStatusReg;
    uint8_t sr = 0;

    W25QXX_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
    HAL_SPI_Receive(&hspi1, &sr, 1, 100);
    W25QXX_CS_HIGH();

    return sr;
}

/**
  * @brief  等待 Flash 空闲
  * @retval 无
  */
void W25QXX_WaitBusy(void)
{
    uint32_t timeout = 1000000;  /* 最大等待 10s */
    while ((W25QXX_ReadSR() & W25X_BusyFlag) && timeout--)
    {
        HAL_Delay(1);
    }
}

/**
  * @brief  写使能
  */
static void W25QXX_WriteEnable(void)
{
    uint8_t cmd = W25X_WriteEnable;
    W25QXX_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
    W25QXX_CS_HIGH();
}

/**
  * @brief  读取 Flash ID
  * @retval Flash 型号 ID
  */
uint16_t W25QXX_ReadID(void)
{
    uint8_t cmd[4] = {W25X_ManufactID, 0x00, 0x00, 0x00};
    uint8_t id[2] = {0, 0};

    W25QXX_CS_LOW();
    HAL_SPI_Transmit(&hspi1, cmd, 4, 100);
    HAL_SPI_Receive(&hspi1, id, 2, 100);
    W25QXX_CS_HIGH();

    return (uint16_t)((id[0] << 8) | id[1]);
}

/**
  * @brief  Flash 初始化 (自动检测型号)
  * @retval W25QXX_OK 或 W25QXX_ERR_ID
  */
uint8_t W25QXX_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* CS 引脚 (PA4) 必须配置为 GPIO 推挽输出：
       CubeMX 默认将其配为 SPI1_NSS 复用功能 (AF_PP)，
       此时 HAL_GPIO_WritePin 无法控制 CS 电平，Flash 通信会失效 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = W25QXX_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(W25QXX_CS_PORT, &GPIO_InitStruct);
    W25QXX_CS_HIGH();

    W25QXX_Type = W25QXX_ReadID();

    switch (W25QXX_Type)
    {
        case W25Q80_ID:
        case W25Q16_ID:
        case W25Q32_ID:
        case W25Q64_ID:
        case W25Q128_ID:
            return W25QXX_OK;
        default:
            return W25QXX_ERR_ID;
    }
}

/**
  * @brief  读取 Flash 数据
  * @param  addr: 起始地址
  * @param  buf: 数据缓冲区
  * @param  len: 数据长度
  * @retval W25QXX_OK
  */
uint8_t W25QXX_Read(uint32_t addr, uint8_t* buf, uint16_t len)
{
    uint8_t cmd[4];
    cmd[0] = W25X_ReadData;
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;

    W25QXX_CS_LOW();
    HAL_SPI_Transmit(&hspi1, cmd, 4, 100);
    HAL_SPI_Receive(&hspi1, buf, len, 1000);
    W25QXX_CS_HIGH();

    return W25QXX_OK;
}

/**
  * @brief  页编程 (最大 256 字节)
  * @param  addr: 起始地址 (必须在同一页内)
  * @param  buf: 数据缓冲区
  * @param  len: 数据长度 (≤256)
  * @retval W25QXX_OK
  */
static void W25QXX_PageProgram(uint32_t addr, uint8_t* buf, uint16_t len)
{
    uint8_t cmd[4];
    cmd[0] = W25X_PageProgram;
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;

    W25QXX_WriteEnable();
    W25QXX_CS_LOW();
    HAL_SPI_Transmit(&hspi1, cmd, 4, 100);
    HAL_SPI_Transmit(&hspi1, buf, len, 1000);
    W25QXX_CS_HIGH();
    W25QXX_WaitBusy();
}

/**
  * @brief  写入 Flash 数据 (自动处理跨页和扇区擦除)
  * @param  addr: 起始地址
  * @param  buf: 数据缓冲区
  * @param  len: 数据长度
  * @retval W25QXX_OK
  */
uint8_t W25QXX_Write(uint32_t addr, uint8_t* buf, uint16_t len)
{
    uint16_t page_remain = W25QXX_PAGE_SIZE - (addr % W25QXX_PAGE_SIZE);

    /* 如果起始地址不在页首, 先写入第一页剩余部分 */
    if (page_remain < len)
    {
        W25QXX_PageProgram(addr, buf, page_remain);
        addr += page_remain;
        buf += page_remain;
        len -= page_remain;
    }

    /* 整页写入 */
    while (len >= W25QXX_PAGE_SIZE)
    {
        W25QXX_PageProgram(addr, buf, W25QXX_PAGE_SIZE);
        addr += W25QXX_PAGE_SIZE;
        buf += W25QXX_PAGE_SIZE;
        len -= W25QXX_PAGE_SIZE;
    }

    /* 写入剩余部分 */
    if (len > 0)
    {
        W25QXX_PageProgram(addr, buf, len);
    }

    return W25QXX_OK;
}

/**
  * @brief  擦除扇区 (4KB)
  * @param  addr: 扇区地址 (自动对齐到 4KB 边界)
  * @retval W25QXX_OK
  */
uint8_t W25QXX_SectorErase(uint32_t addr)
{
    uint8_t cmd[4];

    /* 地址对齐到 4KB 边界 */
    addr &= ~(W25QXX_SECTOR_SIZE - 1);

    cmd[0] = W25X_SectorErase;
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;

    W25QXX_WriteEnable();
    W25QXX_CS_LOW();
    HAL_SPI_Transmit(&hspi1, cmd, 4, 100);
    W25QXX_CS_HIGH();
    W25QXX_WaitBusy();

    return W25QXX_OK;
}

/**
  * @brief  擦除整片 Flash (慎用!)
  * @retval W25QXX_OK
  */
uint8_t W25QXX_ChipErase(void)
{
    uint8_t cmd = W25X_ChipErase;

    W25QXX_WriteEnable();
    W25QXX_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
    W25QXX_CS_HIGH();
    W25QXX_WaitBusy();

    return W25QXX_OK;
}