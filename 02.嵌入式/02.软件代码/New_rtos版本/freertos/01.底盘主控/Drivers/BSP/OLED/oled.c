#include "oled.h"
#include "font.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "iic_hw.h"
#include "math.h"

#define OLED_I2C_ADDR    0x3C

void oled_write_cmd(uint8_t cmd)
{

    iic_write_register(OLED_I2C_ADDR, 0x00, cmd);
}

void oled_write_data(uint8_t data)
{

    iic_write_register(OLED_I2C_ADDR, 0x40, data);
}

void oled_init(void)
{
    delay_ms(100);

    iic_init();

    oled_write_cmd(0xAE);
    oled_write_cmd(0xD5);
    oled_write_cmd(0x80);
    oled_write_cmd(0xA8);
    oled_write_cmd(0x3F);
    oled_write_cmd(0xD3);
    oled_write_cmd(0x00);
    oled_write_cmd(0x40);
    oled_write_cmd(0xA1);
    oled_write_cmd(0xC8);
    oled_write_cmd(0xDA);
    oled_write_cmd(0x12);
    oled_write_cmd(0x81);
    oled_write_cmd(0xCF);
    oled_write_cmd(0xD9);
    oled_write_cmd(0xF1);
    oled_write_cmd(0xDB);
    oled_write_cmd(0x30);
    oled_write_cmd(0xA4);
    oled_write_cmd(0xA6);
    oled_write_cmd(0x8D);
    oled_write_cmd(0x14);
    oled_write_cmd(0xAF);
}

void oled_set_cursor(uint8_t x, uint8_t y)
{
    oled_write_cmd(0xB0 + y);
    oled_write_cmd((x & 0x0F) | 0x00);
    oled_write_cmd(((x & 0xF0) >> 4) | 0x10);
}

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

void oled_show_string(uint8_t x, uint8_t y, char *p, uint8_t size)
{
    while(*p != '\0')
    {
        oled_show_char(x, y, *p, size);
        x += size/2;
        p++;
    }
}

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

void oled_printf(uint8_t x, uint8_t y, uint8_t size, const char *fmt, ...)
{
    char buf[64] = {0};
    va_list args;

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    oled_show_string(x, y, buf, size);
}

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

void os_oled_printf(uint8_t x, uint8_t y, uint8_t size, const char *fmt, ...)
{
    char buf[32];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    oled_show_string(x, y, buf, size);
}

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
