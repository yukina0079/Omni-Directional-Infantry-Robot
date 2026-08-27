#include "oled.h"
#include "font.h"
#include <stdio.h>  // 用于sprintf函数
#include <string.h> // 用于strlen函数
void oled_gpio_init(void)
{
		GPIO_InitTypeDef gpio_initstruct;//定义结构体参数
		OLED_I2C_SCL_CLK();
		OLED_I2C_SDA_CLK();
	
		gpio_initstruct.Pin = OLED_I2C_SCL_PIN;//配置引脚号
		gpio_initstruct.Mode = GPIO_MODE_OUTPUT_PP;//配置工作模式 推挽输出
		gpio_initstruct.Pull = GPIO_PULLUP;//配置上拉下拉
		gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;//配置输出速度
		HAL_GPIO_Init(OLED_I2C_SCL_PORT,&gpio_initstruct);
	
		gpio_initstruct.Pin = OLED_I2C_SDA_PIN;//配置引脚号
		HAL_GPIO_Init(OLED_I2C_SDA_PORT,&gpio_initstruct);
}
void oled_i2c_start(void)
{
		OLED_SCL_SET();
		OLED_SDA_SET();
		OLED_SDA_RESET();
		OLED_SCL_RESET();	
	

}
void oled_i2c_stop(void)
{
		OLED_SCL_SET();
		OLED_SDA_RESET();
		OLED_SDA_SET();

}

void oled_i2c_ack(void)
{
		OLED_SCL_SET();
		OLED_SCL_RESET();

}
void oled_write_byte(uint8_t data)
{
		uint8_t i,tmp;
		tmp = data;
		for(i=0;i<8;i++){
				if((tmp & 0x80) == 0x80)
						OLED_SDA_SET();
				else
						OLED_SDA_RESET();
				
				tmp = tmp << 1;
				OLED_SCL_SET();
				OLED_SCL_RESET();
		}
}

void oled_write_cmd(uint8_t cmd)
{
			oled_i2c_start();
			oled_write_byte(0x78);
			oled_i2c_ack();
			oled_write_byte(0x00);
			oled_i2c_ack();
			oled_write_byte(cmd);
			oled_i2c_ack();			
			oled_i2c_stop();			
}
void oled_write_data(uint8_t data)
{
			oled_i2c_start();
			oled_write_byte(0x78);
			oled_i2c_ack();
			oled_write_byte(0x40);
			oled_i2c_ack();
			oled_write_byte(data);
			oled_i2c_ack();			
			oled_i2c_stop();
}

void oled_init(void)
{
    oled_gpio_init();
    
    delay_ms(100);
    
    oled_write_cmd(0xAE);    //设置显示开启/关闭，0xAE关闭，0xAF开启

    oled_write_cmd(0xD5);    //设置显示时钟分频比/振荡器频率
    oled_write_cmd(0x80);    //0x00~0xFF

    oled_write_cmd(0xA8);    //设置多路复用率
    oled_write_cmd(0x3F);    //0x0E~0x3F

    oled_write_cmd(0xD3);    //设置显示偏移
    oled_write_cmd(0x00);    //0x00~0x7F

    oled_write_cmd(0x40);    //设置显示开始行，0x40~0x7F

    oled_write_cmd(0xA1);    //设置左右方向，0xA1正常，0xA0左右反置

    oled_write_cmd(0xC8);    //设置上下方向，0xC8正常，0xC0上下反置

    oled_write_cmd(0xDA);    //设置COM引脚硬件配置
    oled_write_cmd(0x12);

    oled_write_cmd(0x81);    //设置对比度
    oled_write_cmd(0xCF);    //0x00~0xFF

    oled_write_cmd(0xD9);    //设置预充电周期
    oled_write_cmd(0xF1);

    oled_write_cmd(0xDB);    //设置VCOMH取消选择级别
    oled_write_cmd(0x30);

    oled_write_cmd(0xA4);    //设置整个显示打开/关闭

    oled_write_cmd(0xA6);    //设置正常/反色显示，0xA6正常，0xA7反色

    oled_write_cmd(0x8D);    //设置充电泵
    oled_write_cmd(0x14);

    oled_write_cmd(0xAF);    //开启显示
 
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
/**
 * @brief  OLED显示小数（修复版，无编译警告）
 * @param  x: 起始横坐标 (0~127)
 * @param  y: 起始纵坐标 (0~7，对应OLED的8页)
 * @param  num: 要显示的小数（支持正负）
 * @param  int_len: 整数部分显示位数（不足补0，超过则显示全部）
 * @param  dec_len: 小数部分显示位数（不足补0，超过则截断）
 * @param  size: 字体大小 (12/16/24，需与现有字体数组匹配)
 * @retval 无
 */
void oled_show_float(uint8_t x, uint8_t y, float num, uint8_t int_len, uint8_t dec_len, uint8_t size)
{
    // 定义缓冲区，足够容纳正负号+整数位+小数点+小数位（最大支持16位）
    char float_buf[16] = {0};
    
    // 修复格式串：浮点数统一用 %.*f，整数部分补0通过字符串处理
    // 格式串说明：%.*f → 小数位数由dec_len指定
    sprintf(float_buf, "%.*f", dec_len, num);
    
    // 处理整数部分补0：如果整数位长度不足，在前面补0
    char temp_buf[16] = {0};
    char *dot_pos = strchr(float_buf, '.'); // 找到小数点位置
    if(dot_pos != NULL)
    {
        uint8_t current_int_len = dot_pos - float_buf; // 当前整数部分长度
        // 如果当前整数位 < 指定长度，补0
        if(current_int_len < int_len)
        {
            uint8_t fill_zero = int_len - current_int_len;
            // 先补0
            for(uint8_t k=0; k<fill_zero; k++)
            {
                temp_buf[k] = '0';
            }
            // 拼接原字符串
            strcat(temp_buf, float_buf);
            // 覆盖回float_buf
            strcpy(float_buf, temp_buf);
        }
    }
    
    // 调用已有的字符串显示函数
    oled_show_string(x, y, float_buf, size);
}



