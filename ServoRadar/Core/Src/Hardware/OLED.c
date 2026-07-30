/**
  ******************************************************************************
  * @file    OLED.c
  * @brief   SSD1306 OLED 驱动实现 (128x64, I2C 接口)
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
  * 【优化说明】
  *   1. 支持任意 I2C 句柄（不绑定特定 I2C 总线）
  *   2. 增加显存缓冲区（GRAM），先写入缓冲区再一次性刷新到 OLED
  *   3. 支持 6x8 点阵字符显示
  *   4. 支持数字、浮点数显示
  ******************************************************************************
  */

#include "Hardware/OLED.h"

/* 6x8 ASCII字符字库 */
static const unsigned char Font6x8[][6] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // sp
    {0x00, 0x00, 0x00, 0x2f, 0x00, 0x00}, // !
    {0x00, 0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x00, 0x14, 0x7f, 0x14, 0x7f, 0x14}, // #
    {0x00, 0x24, 0x2a, 0x7f, 0x2a, 0x12}, // $
    {0x00, 0x62, 0x64, 0x08, 0x13, 0x23}, // %
    {0x00, 0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x00, 0x1c, 0x22, 0x41, 0x00}, // (
    {0x00, 0x00, 0x41, 0x22, 0x1c, 0x00}, // )
    {0x00, 0x14, 0x08, 0x3E, 0x08, 0x14}, // *
    {0x00, 0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    {0x00, 0x00, 0x00, 0xA0, 0x60, 0x00}, // ,
    {0x00, 0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x00, 0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x00, 0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x00, 0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x00, 0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x00, 0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x00, 0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x00, 0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x00, 0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x00, 0xAC, 0x6C, 0x00, 0x00}, // ;
    {0x00, 0x00, 0x08, 0x14, 0x22, 0x41}, // <
    {0x00, 0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x00, 0x41, 0x22, 0x14, 0x08, 0x00}, // >
    {0x00, 0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x00, 0x32, 0x49, 0x59, 0x51, 0x3E}, // @
    {0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C}, // A
    {0x00, 0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x00, 0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x00, 0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x00, 0x7F, 0x09, 0x09, 0x09, 0x01}, // F
    {0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
    {0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x00, 0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x00, 0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x00, 0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M
    {0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x00, 0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x00, 0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x00, 0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x00, 0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F}, // W
    {0x00, 0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x00, 0x07, 0x08, 0x70, 0x08, 0x07}, // Y
    {0x00, 0x61, 0x51, 0x49, 0x45, 0x43}, // Z
    {0x00, 0x00, 0x7F, 0x41, 0x41, 0x00}, // [
    {0x00, 0x55, 0x2A, 0x55, 0x2A, 0x55}, // backslash
    {0x00, 0x00, 0x41, 0x41, 0x7F, 0x00}, // ]
    {0x00, 0x04, 0x02, 0x01, 0x02, 0x04}, // ^
    {0x00, 0x40, 0x40, 0x40, 0x40, 0x40}, // _
    {0x00, 0x00, 0x01, 0x02, 0x04, 0x00}, // `
    {0x00, 0x20, 0x54, 0x54, 0x54, 0x78}, // a
    {0x00, 0x7F, 0x48, 0x44, 0x44, 0x38}, // b
    {0x00, 0x38, 0x44, 0x44, 0x44, 0x20}, // c
    {0x00, 0x38, 0x44, 0x44, 0x48, 0x7F}, // d
    {0x00, 0x38, 0x54, 0x54, 0x54, 0x18}, // e
    {0x00, 0x08, 0x7E, 0x09, 0x01, 0x02}, // f
    {0x00, 0x0C, 0x52, 0x52, 0x52, 0x3E}, // g
    {0x00, 0x7F, 0x08, 0x04, 0x04, 0x78}, // h
    {0x00, 0x00, 0x44, 0x7D, 0x40, 0x00}, // i
    {0x00, 0x20, 0x40, 0x44, 0x3D, 0x00}, // j
    {0x00, 0x7F, 0x10, 0x28, 0x44, 0x00}, // k
    {0x00, 0x00, 0x41, 0x7F, 0x40, 0x00}, // l
    {0x00, 0x7C, 0x04, 0x18, 0x04, 0x78}, // m
    {0x00, 0x7C, 0x08, 0x04, 0x04, 0x78}, // n
    {0x00, 0x38, 0x44, 0x44, 0x44, 0x38}, // o
    {0x00, 0x7C, 0x14, 0x14, 0x14, 0x08}, // p
    {0x00, 0x08, 0x14, 0x14, 0x18, 0x7C}, // q
    {0x00, 0x7C, 0x08, 0x04, 0x04, 0x08}, // r
    {0x00, 0x48, 0x54, 0x54, 0x54, 0x20}, // s
    {0x00, 0x04, 0x3F, 0x44, 0x40, 0x20}, // t
    {0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C}, // u
    {0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C}, // v
    {0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C}, // w
    {0x00, 0x44, 0x28, 0x10, 0x28, 0x44}, // x
    {0x00, 0x0C, 0x50, 0x50, 0x50, 0x3C}, // y
    {0x00, 0x44, 0x64, 0x54, 0x4C, 0x44}, // z
    {0x00, 0x00, 0x08, 0x77, 0x41, 0x00}, // {
    {0x00, 0x00, 0x00, 0x77, 0x00, 0x00}, // |
    {0x00, 0x41, 0x77, 0x08, 0x00, 0x00}, // }
    {0x00, 0x02, 0x01, 0x02, 0x04, 0x02}, // ~
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}  // DEL
};

/* ==================== 显存缓冲区 ==================== */
/* 8 页 × 128 列，每字节对应 8 个像素行 */
static uint8_t OLED_GRAM[OLED_PAGE_NUM][OLED_WIDTH];

/* 私有 I2C 句柄（保存当前使用的 I2C 总线） */
static I2C_HandleTypeDef* s_hi2c = NULL;

/**
  * @brief  发送命令到 OLED
  * @param  cmd: 命令字节
  * @retval 无
  * @note   I2C 数据格式：[0x00][命令]，0x00 表示后续字节是命令
  */
static void OLED_WriteCommand(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};
    HAL_I2C_Master_Transmit(s_hi2c, OLED_I2C_ADDR, buf, 2, HAL_MAX_DELAY);
}

/**
  * @brief  发送数据到 OLED
  * @param  dat: 数据字节
  * @retval 无
  * @note   I2C 数据格式：[0x40][数据]，0x40 表示后续字节是显示数据
  */
static void OLED_WriteData(uint8_t dat)
{
    uint8_t buf[2] = {0x40, dat};
    HAL_I2C_Master_Transmit(s_hi2c, OLED_I2C_ADDR, buf, 2, HAL_MAX_DELAY);
}

/**
  * @brief  设置显存坐标（列和页）
  * @param  x: 列地址（0~127）
  * @param  y: 页地址（0~7）
  * @retval 无
  * @note   SSD1306 显存按页组织，每页 8 行
  */
static void OLED_SetPos(uint8_t x, uint8_t y)
{
    OLED_WriteCommand(0xB0 | y);              /* 设置页地址 */
    OLED_WriteCommand(((x & 0xF0) >> 4) | 0x10);  /* 设置列高 4 位 */
    OLED_WriteCommand(x & 0x0F);              /* 设置列低 4 位 */
}

/**
  * @brief  次方函数（用于数字显示）
  * @param  m: 底数
  * @param  n: 指数
  * @retval m^n
  */
static uint32_t OLED_Pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while(n--) result *= m;
    return result;
}

/**
  * @brief  初始化 OLED 显示屏
  * @param  hi2c: I2C 句柄指针
  * @retval 无
  * 
  * 【初始化流程】
  *   1. 保存 I2C 句柄
  *   2. 延时 100ms，等待 OLED 上电稳定
  *   3. 发送一系列初始化命令（关闭显示 → 配置参数 → 开启显示）
  *   4. 清屏并刷新显示
  * 
  * 【关键命令说明】
  *   - 0xAE/0xAF：关闭/开启显示
  *   - 0x81 + 0xFF：设置对比度（最大值）
  *   - 0xA1：段重映射（左右翻转）
  *   - 0xC8：COM 输出扫描方向（上下翻转）
  *   - 0x8D + 0x14：启用内部 DC-DC 转换器
  */
void OLED_Init(I2C_HandleTypeDef* hi2c)
{
    s_hi2c = hi2c;  /* 保存 I2C 句柄 */
    
    HAL_Delay(100);  /* 等待 OLED 上电稳定 */
    
    /* 发送初始化命令序列 */
    OLED_WriteCommand(0xAE); // 关闭显示
    OLED_WriteCommand(0x20); // 设置内存寻址模式
    OLED_WriteCommand(0x10); // 页寻址模式
    OLED_WriteCommand(0xB0); // 设置页起始地址
    OLED_WriteCommand(0xC8); // 设置 COM 输出扫描方向（从下到上）
    OLED_WriteCommand(0x00); // 设置低列地址
    OLED_WriteCommand(0x10); // 设置高列地址
    OLED_WriteCommand(0x40); // 设置显示起始行
    OLED_WriteCommand(0x81); // 设置对比度控制
    OLED_WriteCommand(0xFF); // 对比度最大值
    OLED_WriteCommand(0xA1); // 设置段重映射（左右翻转）
    OLED_WriteCommand(0xA6); // 设置正常显示（非反色）
    OLED_WriteCommand(0xA8); // 设置多路复用率
    OLED_WriteCommand(0x3F); // 1/64 duty（64 行全部使用）
    OLED_WriteCommand(0xD3); // 设置显示偏移
    OLED_WriteCommand(0x00); // 无偏移
    OLED_WriteCommand(0xD5); // 设置显示时钟分频比
    OLED_WriteCommand(0xF0); // 最高频率，最小分频
    OLED_WriteCommand(0xD9); // 设置预充电周期
    OLED_WriteCommand(0x22); // 预充电时间
    OLED_WriteCommand(0xDA); // 设置 COM 引脚硬件配置
    OLED_WriteCommand(0x12); // 交替 COM 配置
    OLED_WriteCommand(0xDB); // 设置 VCOMH 取消选择级别
    OLED_WriteCommand(0x20); // VCOMH 电压
    OLED_WriteCommand(0x8D); // 设置电荷泵
    OLED_WriteCommand(0x14); // 启用电荷泵（必须开启才能显示）
    OLED_WriteCommand(0xAF); // 开启显示
    
    OLED_Clear(hi2c);    /* 清空显存 */
    OLED_Refresh(hi2c);  /* 刷新到屏幕 */
}

/**
  * @brief  清空显存（全部写 0）
  * @param  hi2c: I2C 句柄指针
  * @retval 无
  * @note   只清空缓冲区，需调用 OLED_Refresh 才能显示到屏幕
  */
void OLED_Clear(I2C_HandleTypeDef* hi2c)
{
    s_hi2c = hi2c;
    for(uint8_t i = 0; i < OLED_PAGE_NUM; i++)
    {
        for(uint8_t j = 0; j < OLED_WIDTH; j++)
        {
            OLED_GRAM[i][j] = 0x00;  /* 全部写 0，像素熄灭 */
        }
    }
}

/**
  * @brief  刷新显存到 OLED 屏幕
  * @param  hi2c: I2C 句柄指针
  * @retval 无
  * @note   将 GRAM 缓冲区数据逐页发送到 OLED，每次刷新整个屏幕
  */
void OLED_Refresh(I2C_HandleTypeDef* hi2c)
{
    s_hi2c = hi2c;
    for(uint8_t i = 0; i < OLED_PAGE_NUM; i++)
    {
        OLED_SetPos(0, i);  /* 设置页地址 */
        for(uint8_t j = 0; j < OLED_WIDTH; j++)
        {
            OLED_WriteData(OLED_GRAM[i][j]);  /* 发送 128 列数据 */
        }
    }
}

/**
  * @brief  在指定坐标画点
  * @param  hi2c: I2C 句柄指针
  * @param  x: X 坐标（0~127）
  * @param  y: Y 坐标（0~63）
  * @param  color: 颜色（1=亮，0=灭）
  * @retval 无
  * 
  * 【原理】
  *   OLED 显存按页组织，每页 8 行。
  *   Y 坐标决定在哪一页（y/8），以及该页的哪个 bit（y%8）。
  *   通过位操作设置对应 bit，不影响同一页的其他行。
  */
void OLED_DrawPoint(I2C_HandleTypeDef* hi2c, uint8_t x, uint8_t y, uint8_t color)
{
    s_hi2c = hi2c;
    if(x > 127 || y > 63) return;  /* 超出屏幕范围，直接返回 */
    
    uint8_t page = y / 8;   /* 计算在哪一页 */
    uint8_t bit = y % 8;    /* 计算在该页的哪个 bit */
    uint8_t mask = 1 << bit;  /* 生成位掩码 */
    
    if(color)
    {
        OLED_GRAM[page][x] |= mask;  /* 置 1，点亮像素 */
    }
    else
    {
        OLED_GRAM[page][x] &= ~mask; /* 清 0，熄灭像素 */
    }
}

/**
  * @brief  显示单个字符（6x8 点阵）
  * @param  hi2c: I2C 句柄指针
  * @param  x: X 坐标（0~126）
  * @param  y: Y 坐标（0~56）
  * @param  chr: 字符（ASCII 码）
  * @param  size: 字符大小（当前仅支持 6）
  * @retval 无
  * 
  * 【原理】
  *   从 Font6x8 字库中取出字符的点阵数据，逐 bit 绘制到显存。
  *   每个字符占 6 列 × 8 行，共 48 个像素。
  */
void OLED_ShowChar(I2C_HandleTypeDef* hi2c, uint8_t x, uint8_t y, uint8_t chr, uint8_t size)
{
    s_hi2c = hi2c;
    if(x > 126 || y > 56) return;  /* 超出可显示范围，直接返回 */
    
    uint8_t index = chr - ' ';  /* 计算字符在字库中的索引（空格 = 0） */
    
    for(uint8_t i = 0; i < 6; i++)  /* 遍历 6 列 */
    {
        for(uint8_t j = 0; j < 8; j++)  /* 遍历 8 行 */
        {
            if(Font6x8[index][i] & (1 << j))  /* 检查对应 bit 是否为 1 */
            {
                OLED_DrawPoint(hi2c, x + i, y + j, 1);  /* 点亮像素 */
            }
            else
            {
                OLED_DrawPoint(hi2c, x + i, y + j, 0);  /* 熄灭像素 */
            }
        }
    }
}

/**
  * @brief  显示字符串
  * @param  hi2c: I2C 句柄指针
  * @param  x: 起始 X 坐标
  * @param  y: 起始 Y 坐标
  * @param  str: 字符串指针（仅支持 ASCII 字符）
  * @retval 无
  * @note   自动换行，超出屏幕时停止显示
  */
void OLED_ShowString(I2C_HandleTypeDef* hi2c, uint8_t x, uint8_t y, char *str)
{
    s_hi2c = hi2c;
    uint8_t x0 = x;  /* 保存起始 X 坐标，用于换行 */
    
    while(*str >= ' ' && *str <= '~')  /* 遍历字符串，直到非 ASCII 字符 */
    {
        if(x > 126)  /* 超出右边界，换行 */
        {
            x = x0;
            y += 8;
        }
        if(y > 56) return;  /* 超出下边界，停止显示 */
        
        OLED_ShowChar(hi2c, x, y, *str, 6);  /* 显示单个字符 */
        str++;
        x += 6;  /* X 坐标前进 6 像素 */
    }
}

/**
  * @brief  显示无符号数字
  * @param  hi2c: I2C 句柄指针
  * @param  x: X 坐标
  * @param  y: Y 坐标
  * @param  num: 要显示的数字
  * @param  len: 显示位数
  * @retval 无
  * 
  * 【原理】
  *   从高位到低位逐位提取数字，转换为 ASCII 字符显示。
  *   高位的 0 显示为空格，避免前导零影响美观。
  */
void OLED_ShowNum(I2C_HandleTypeDef* hi2c, uint8_t x, uint8_t y, uint32_t num, uint8_t len)
{
    s_hi2c = hi2c;
    uint8_t enshow = 0;  /* 标记是否已经开始显示非零数字 */
    
    for(uint8_t t = 0; t < len; t++)
    {
        /* 提取当前位的数字：先除以 10^(len-t-1)，再取模 10 */
        uint8_t temp = (num / OLED_Pow(10, len - t - 1)) % 10;
        
        if(enshow == 0 && t < (len - 1))  /* 还未开始显示，且不是最后一位 */
        {
            if(temp == 0)
            {
                OLED_ShowChar(hi2c, x + t * 6, y, ' ', 6);  /* 显示空格（隐藏前导零） */
                continue;
            }
            else
            {
                enshow = 1;  /* 遇到第一个非零数字，开始显示 */
            }
        }
        OLED_ShowChar(hi2c, x + t * 6, y, temp + '0', 6);  /* 显示数字字符 */
    }
}

/**
  * @brief  显示浮点数
  * @param  hi2c: I2C 句柄指针
  * @param  x: X 坐标
  * @param  y: Y 坐标
  * @param  num: 要显示的浮点数
  * @param  decimal: 小数位数
  * @retval 无
  * 
  * 【原理】
  *   使用 sprintf 将浮点数格式化为字符串，再调用 OLED_ShowString 显示。
  *   注意：sprintf 会占用较多 Flash 空间，如果代码空间紧张可改用整数运算。
  */
void OLED_ShowFloat(I2C_HandleTypeDef* hi2c, uint8_t x, uint8_t y, float num, uint8_t decimal)
{
    s_hi2c = hi2c;
    char str[20];
    sprintf(str, "%.*f", decimal, num);  /* 格式化为指定小数位数的字符串 */
    OLED_ShowString(hi2c, x, y, str);
}

/**
  * @brief  显示带符号数字
  * @param  hi2c: I2C 句柄指针
  * @param  x: X 坐标
  * @param  y: Y 坐标
  * @param  num: 要显示的数字（可正可负）
  * @param  len: 显示位数
  * @retval 无
  * 
  * 【原理】
  *   先显示 '+' 或 '-' 符号，再显示数字的绝对值。
  *   适用于显示角度、温度等可能为负的物理量。
  */
void OLED_ShowSignedNum(I2C_HandleTypeDef* hi2c, uint8_t x, uint8_t y, int32_t num, uint8_t len)
{
    s_hi2c = hi2c;
    if(num >= 0)
    {
        OLED_ShowChar(hi2c, x, y, '+', 6);  /* 显示正号 */
        OLED_ShowNum(hi2c, x + 6, y, num, len);  /* 显示数字 */
    }
    else
    {
        OLED_ShowChar(hi2c, x, y, '-', 6);  /* 显示负号 */
        OLED_ShowNum(hi2c, x + 6, y, -num, len);  /* 显示绝对值 */
    }
}

/**
  * @brief  绘制直线（Bresenham整数算法，无浮点运算）
  * 
  * 【新增 - 与原驱动区别】
  * 原驱动只有字符/数字显示，无法绘制雷达扫描线和扇形边框。
  * 使用 Bresenham 算法，纯整数运算，在Cortex-M3上高效运行（约几十周期/像素）。
  * 用于：
  *   - 雷达扇形的两条边界直线（0°和180°方向）
  *   - 实时扫描角度指示线（随舵机角度旋转）
  * 
  * @param  x1,y1: 起点坐标 (0~127, 0~63)
  * @param  x2,y2: 终点坐标
  * @param  color: 1=点亮像素, 0=熄灭像素
  */
void OLED_DrawLine(I2C_HandleTypeDef* hi2c, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color)
{
    int16_t dx = (int16_t)((x2 >= x1) ? (x2 - x1) : (x1 - x2));  /* 绝对差值 */
    int16_t dy = (int16_t)((y2 >= y1) ? (y2 - y1) : (y1 - y2));
    
    int16_t sx = (x1 < x2) ? 1 : -1;  /* X方向步进方向 */
    int16_t sy = (y1 < y2) ? 1 : -1;  /* Y方向步进方向 */
    
    int16_t err = dx - dy;  /* Bresenham误差项 */
    int16_t e2;
    
    /* 防止无限循环：限定最多画200个像素（屏幕对角线约142像素） */
    uint8_t cnt = 0;
    while (cnt++ < 200)
    {
        OLED_DrawPoint(hi2c, x1, y1, color);
        
        /* 到达终点，退出 */
        if ((x1 == x2) && (y1 == y2)) break;
        
        /* Bresenham误差判断 */
        e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x1 = (uint8_t)(x1 + sx); }
        if (e2 <  dx) { err += dx; y1 = (uint8_t)(y1 + sy); }
    }
}

/**
  * @brief  绘制上半圆弧（雷达扇形参考圆）
  * 
  * 【新增 - 与原驱动区别】
  * 原驱动无绘图能力。本函数采用中点圆算法（Bresenham圆变种），纯整数运算。
  * 只绘制上半圆（y <= cy），因为：
  *   - OLED屏幕下边缘放数值信息（距离、角度、温湿度）
  *   - 上半圆模拟雷达扇形扫描视野（0°~180°）
  * 
  * 用途：绘制3层参考距离圈（例如1m/2m/3m），直观判断障碍物距离。
  * 
  * @param  cx,cy: 圆心坐标（通常 cx=64, cy=50 让上半圆显示在屏幕上半部）
  * @param  r: 圆半径（像素）
  * @param  color: 1=亮, 0=灭
  */
void OLED_DrawSemicircle(I2C_HandleTypeDef* hi2c, uint8_t cx, uint8_t cy, uint8_t r, uint8_t color)
{
    int16_t x = 0;
    int16_t y = (int16_t)r;
    int16_t d = 1 - (int16_t)r;  /* 中点圆判别式 */
    
    /* 同样用cnt防止极端情况死循环 */
    uint8_t cnt = 0;
    while ((x <= y) && (cnt++ < 150))
    {
        /* 上半圆：只绘制 y <= cy 的点（因为圆心在下方，y越小越靠屏幕顶部）
         * 八对称性只取4个上半部分象限的点：左、右、左上、右上 */
        
        /* 水平方向：(cx+x, cy-y) 和 (cx-x, cy-y) - 顶部附近 */
        if ((int16_t)cy - y >= 0) {
            OLED_DrawPoint(hi2c, (uint8_t)(cx + x), (uint8_t)(cy - y), color);
            if (x != 0) OLED_DrawPoint(hi2c, (uint8_t)(cx - x), (uint8_t)(cy - y), color);
        }
        /* 斜向45度：(cx+y, cy-x) 和 (cx-y, cy-x) */
        if ((int16_t)cy - x >= 0) {
            OLED_DrawPoint(hi2c, (uint8_t)(cx + y), (uint8_t)(cy - x), color);
            if (y != 0) OLED_DrawPoint(hi2c, (uint8_t)(cx - y), (uint8_t)(cy - x), color);
        }
        
        /* 中点圆下一步 */
        if (d < 0) {
            d += 2 * x + 3;
        } else {
            d += 2 * (x - y) + 5;
            y--;
        }
        x++;
    }
}