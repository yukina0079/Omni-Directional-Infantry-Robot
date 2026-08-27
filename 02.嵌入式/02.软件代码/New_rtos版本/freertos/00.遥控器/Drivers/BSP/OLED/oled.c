#include "oled.h"
#include "font.h"
#include "iic.h"              

// ========== 硬件 I2C 初始化（替代原软件 GPIO 初始化） ==========
void oled_gpio_init(void)
{
    // 硬件 I2C 的 GPIO 和外设初始化由 i2c_init() 完成
    // 添加静态标志避免重复初始化（可选）
    static uint8_t initialized = 0;
    if (!initialized) {
        i2c_init();
        initialized = 1;
    }
}

// ========== 硬件 I2C 发送命令 ==========
void oled_write_cmd(uint8_t cmd)
{
    // 命令格式：控制字节 0x00 + 命令字节
    uint8_t buf[2] = {0x00, cmd};
    // 设备地址 0x78 为 8 位地址（含写位），HAL 库期望 7 位地址左移 1 位，故直接使用 0x78
    HAL_I2C_Master_Transmit(&hi2c2, 0x78, buf, 2, 100);
}

// ========== 硬件 I2C 发送数据 ==========
void oled_write_data(uint8_t data)
{
    // 数据格式：控制字节 0x40 + 数据字节
    uint8_t buf[2] = {0x40, data};
    HAL_I2C_Master_Transmit(&hi2c2, 0x78, buf, 2, 100);
}

// ========== 其余函数保持不变 ==========
void oled_init(void)
{
    oled_gpio_init();           // 硬件 I2C 初始化
    delay_ms(100);

    oled_write_cmd(0xAE);       // 关闭显示
    oled_write_cmd(0xD5);       // 设置时钟分频
    oled_write_cmd(0x80);
    oled_write_cmd(0xA8);       // 设置多路复用率
    oled_write_cmd(0x3F);
    oled_write_cmd(0xD3);       // 设置显示偏移
    oled_write_cmd(0x00);
    oled_write_cmd(0x40);       // 设置显示起始行
    oled_write_cmd(0xA1);       // 段重映射（左右反转）
    oled_write_cmd(0xC8);       // COM 扫描方向（上下反转）
    oled_write_cmd(0xDA);       // 设置 COM 硬件引脚配置
    oled_write_cmd(0x12);
    oled_write_cmd(0x81);       // 设置对比度
    oled_write_cmd(0xCF);
    oled_write_cmd(0xD9);       // 设置预充电周期
    oled_write_cmd(0xF1);
    oled_write_cmd(0xDB);       // 设置 VCOMH 电压
    oled_write_cmd(0x30);
    oled_write_cmd(0xA4);       // 全局显示开启
    oled_write_cmd(0xA6);       // 正常显示（非反色）
    oled_write_cmd(0x8D);       // 电荷泵设置
    oled_write_cmd(0x14);       // 开启电荷泵
    oled_write_cmd(0xAF);       // 开启显示

    oled_fill(0x00);            // 清屏
}

void oled_set_cursor(uint8_t x, uint8_t y)
{
    oled_write_cmd(0xB0 + y);                 // 设置页地址
    oled_write_cmd((x & 0x0F) | 0x00);        // 设置列低地址
    oled_write_cmd(((x & 0xF0) >> 4) | 0x10); // 设置列高地址
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

