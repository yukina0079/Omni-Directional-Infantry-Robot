#include "oled.h"
#include "font.h"
#include "iic.h"

void oled_gpio_init(void)
{

    static uint8_t initialized = 0;
    if (!initialized) {
        i2c_init();
        initialized = 1;
    }
}

void oled_write_cmd(uint8_t cmd)
{

    uint8_t buf[2] = {0x00, cmd};

    HAL_I2C_Master_Transmit(&hi2c2, 0x78, buf, 2, 100);
}

void oled_write_data(uint8_t data)
{

    uint8_t buf[2] = {0x40, data};
    HAL_I2C_Master_Transmit(&hi2c2, 0x78, buf, 2, 100);
}

void oled_init(void)
{
    oled_gpio_init();
    delay_ms(100);

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

    oled_fill(0x00);
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
    for (i = 0; i < 8; i++) {
        oled_set_cursor(0, i);
        for (j = 0; j < 128; j++) {
            oled_write_data(data);
        }
    }
}

void oled_show_char(uint8_t x, uint8_t y, uint8_t num, uint8_t size)
{
    uint8_t i, j, page;
    num = num - ' ';
    page = size / 8;
    if (size % 8) page++;

    for (j = 0; j < page; j++) {
        oled_set_cursor(x, y + j);
        for (i = size / 2 * j; i < size / 2 * (j + 1); i++) {
            if (size == 12)
                oled_write_data(ascii_6X12[num][i]);
            else if (size == 16)
                oled_write_data(ascii_8X16[num][i]);
            else if (size == 24)
                oled_write_data(ascii_12X24[num][i]);
        }
    }
}

void oled_show_string(uint8_t x, uint8_t y, char *p, uint8_t size)
{
    while (*p != '\0') {
        oled_show_char(x, y, *p, size);
        x += size / 2;
        p++;
    }
}

void oled_show_chinese(uint8_t x, uint8_t y, uint8_t N, uint8_t size)
{
    uint16_t i, j;
    for (j = 0; j < size / 8; j++) {
        oled_set_cursor(x, y + j);
        for (i = size * j; i < size * (j + 1); i++) {
            if (size == 16)
                oled_write_data(chinese_16x16[N][i]);
            else if (size == 24)
                oled_write_data(chinese_24x24[N][i]);
        }
    }
}

void oled_show_image(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t *bmp)
{
    uint8_t i, j;
    for (j = 0; j < height; j++) {
        oled_set_cursor(x, y + j);
        for (i = 0; i < width; i++) {
            oled_write_data(bmp[width * j + i]);
        }
    }
}
