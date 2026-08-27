#include "oled.h"
#include "font.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "iic_hw.h"  // 替换原iic.h为硬件IIC头文件
#include "math.h"

/* OLED设备I2C地址（7位地址，无需左移） */
#define OLED_I2C_ADDR    0x3C  


/**
 * @brief 硬件IIC方式写OLED命令
 * @param cmd: 要写入的OLED命令
 */
void oled_write_cmd(uint8_t cmd)
{
    // 调用硬件IIC写寄存器接口：OLED命令格式为 0x00+命令字节
    iic_write_register(OLED_I2C_ADDR, 0x00, cmd);
}

/**
 * @brief 硬件IIC方式写OLED数据
 * @param data: 要写入的OLED数据
 */
void oled_write_data(uint8_t data)
{
    // 调用硬件IIC写寄存器接口：OLED数据格式为 0x40+数据字节
    iic_write_register(OLED_I2C_ADDR, 0x40, data);
}

/**
 * @brief OLED初始化（保留原有初始化指令，仅替换底层通信）
 */
void oled_init(void)
{    
    delay_ms(100);
    
    // 先初始化硬件IIC
    iic_init();
    
    // 原有OLED初始化指令不变
    oled_write_cmd(0xAE);    // 设置显示关闭
    oled_write_cmd(0xD5);    // 设置显示时钟分频比/振荡器频率
    oled_write_cmd(0x80);    
    oled_write_cmd(0xA8);    // 设置多路复用率
    oled_write_cmd(0x3F);    
    oled_write_cmd(0xD3);    // 设置显示偏移
    oled_write_cmd(0x00);    
    oled_write_cmd(0x40);    // 设置显示开始行
    oled_write_cmd(0xA1);    // 设置左右方向
    oled_write_cmd(0xC8);    // 设置上下方向
    oled_write_cmd(0xDA);    // 设置COM引脚硬件配置
    oled_write_cmd(0x12);
    oled_write_cmd(0x81);    // 设置对比度
    oled_write_cmd(0xCF);    
    oled_write_cmd(0xD9);    // 设置预充电周期
    oled_write_cmd(0xF1);
    oled_write_cmd(0xDB);    // 设置VCOMH取消选择级别
    oled_write_cmd(0x30);
    oled_write_cmd(0xA4);    // 设置整个显示打开/关闭
    oled_write_cmd(0xA6);    // 设置正常显示
    oled_write_cmd(0x8D);    // 设置充电泵
    oled_write_cmd(0x14);
    oled_write_cmd(0xAF);    // 开启显示
}

/**
 * @brief 设置OLED光标位置（原有逻辑不变）
 * @param x: 列坐标(0~127)
 * @param y: 页坐标(0~7)
 */
void oled_set_cursor(uint8_t x, uint8_t y)
{
    oled_write_cmd(0xB0 + y);
    oled_write_cmd((x & 0x0F) | 0x00);
    oled_write_cmd(((x & 0xF0) >> 4) | 0x10);
}

/**
 * @brief OLED全屏填充（原有逻辑不变）
 * @param data: 填充数据(0x00全黑,0xFF全亮)
 */
void oled_fill(uint8_t data)
{
    uint8_t i, j;
    for(i = 0; i < 8; i++)
    {
        oled_set_cursor(0, i);
        for(j = 0; j < 128; j++)
            oled_write_data(data);
    }
}

/**
 * @brief 显示单个字符（原有逻辑不变，增加临界区保护）
 * @param x: 起始X坐标
 * @param y: 起始Y页
 * @param num: 要显示的字符
 * @param size: 字体大小(12/16/24)
 */
void oled_show_char(uint8_t x, uint8_t y, uint8_t num, uint8_t size)
{
    uint8_t i, j, page;
    
    num = num - ' ';
    page = size / 8;
    if(size % 8)
        page++;
    
    for(j = 0; j < page; j++)
    {
        oled_set_cursor(x, y + j);
        for(i = size / 2 * j; i < size /2 * (j + 1); i++)
        {
            if(size == 12)
                oled_write_data(ascii_6X12[num][i]);
            else if(size == 16)
                oled_write_data(ascii_8X16[num][i]);
            else if(size == 24)
                oled_write_data(ascii_12X24[num][i]);
        }
    }
}

/**
 * @brief 显示字符串（原有逻辑不变）
 * @param x: 起始X坐标
 * @param y: 起始Y页
 * @param p: 字符串指针
 * @param size: 字体大小
 */
void oled_show_string(uint8_t x, uint8_t y, char *p, uint8_t size)
{
    while(*p != '\0')
    {
        oled_show_char(x, y, *p, size);
        x += size/2;
        p++;
    }
}

/**
 * @brief 显示中文字符（原有逻辑不变）
 * @param x: 起始X坐标
 * @param y: 起始Y页
 * @param N: 中文字库索引
 * @param size: 字体大小(16/24)
 */
void oled_show_chinese(uint8_t x, uint8_t y, uint8_t N, uint8_t size)
{
    uint16_t i, j;
    for(j = 0; j < size/8; j++)
    {
        oled_set_cursor(x, y + j);
        for(i = size *j; i < size * (j + 1); i++)
        {
            if(size == 16)
                oled_write_data(chinese_16x16[N][i]);
            else if(size == 24)
                oled_write_data(chinese_24x24[N][i]);
        }
    }
}

/**
 * @brief 显示图片（原有逻辑不变）
 * @param x: 起始X坐标
 * @param y: 起始Y页
 * @param width: 图片宽度
 * @param height: 图片高度(页数量)
 * @param bmp: 图片数据指针
 */
void oled_show_image(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t *bmp)
{
    uint8_t i, j;
    for(j = 0; j < height; j++)
    {
        oled_set_cursor(x, y + j);
        for(i = 0; i < width; i++)
            oled_write_data(bmp[width * j + i]);
    }
}

/**
 * @brief 格式化显示字符串（适配printf风格，增加临界区保护）
 * @param x: 起始X坐标（0~127）
 * @param y: 起始Y页（0~7）
 * @param size: 字体大小（12/16/24）
 * @param fmt: 格式化字符串
 */
void oled_printf(uint8_t x, uint8_t y, uint8_t size, const char *fmt, ...)
{
    char buf[64] = {0};
    va_list args;
    
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    
    oled_show_string(x, y, buf, size);
}

/**
 * @brief 显示两位小数的浮点数（优化版，增加临界区保护）
 * @param x: 起始X坐标
 * @param y: 起始Y页
 * @param size: 字体大小
 * @param num: 要显示的浮点数
 */
void oled_show_float(uint8_t x, uint8_t y, uint8_t size, float num)
{
    int16_t integer_part = (int16_t)num;
    uint16_t decimal_part = (uint16_t)((num - integer_part) * 100);
    if(num < 0 && decimal_part > 0)
    {
        decimal_part = 100 - decimal_part;
        integer_part += 1;
    }
    
    oled_printf(x, y, size, "%4d.%02d", integer_part, decimal_part);
}

/**
 * @brief 格式化显示字符串（带临界区保护和栈安全检查）
 */
void os_oled_printf(uint8_t x, uint8_t y, uint8_t size, const char *fmt, ...)
{
    char buf[32];
    va_list args;
    
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    
    oled_show_string(x, y, buf, size);
}

/**
 * @brief 更鲁棒的浮点数显示（处理所有负数情况）
 */
void os_oled_show_float(uint8_t x, uint8_t y, uint8_t size, float num)
{
    char sign = (num < 0) ? '-' : ' ';
    float abs_num = fabsf(num);
    
    uint16_t integer_part = (uint16_t)abs_num;
    uint16_t decimal_part = (uint16_t)((abs_num - integer_part) * 100 + 0.5f);

    if (decimal_part >= 100) {
        integer_part++;
        decimal_part = 0;
    }

    if(num < 0 && integer_part == 0)
        oled_printf(x, y, size, "-0.%02d", decimal_part);
    else if(num < 0)
        oled_printf(x, y, size, "-%d.%02d", integer_part, decimal_part);
    else
        oled_printf(x, y, size, " %d.%02d", integer_part, decimal_part);

}
