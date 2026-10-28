#include "data.h"
#include "sys.h"
#include "delay.h"
#include "stdlib.h"
#include "math.h"
#include "usart.h"
#include "led.h"
#include "my_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "adc.h"
#include "encoder.h"
#include "oled.h"
#include "key.h"
#include "nrf24L01.h"

uint8_t key_num = {0};
uint8_t key_arr[10] = {1,1,1,1,1,1,1,1,1,1};
uint8_t key_prev_state[10] = {1,1,1,1,1,1,1,1,1,1};
char lx_buffer[20];
char ly_buffer[20];
char rx_buffer[20];
char ry_buffer[20];
char key_num_buffer[20];
char l_en[20];
char r_en[20];

uint16_t adc_value[4] = {0};
uint8_t Send_Out[FRAME13_LEN] = {0};
int16_t Joystick_value[4] = {0};
int16_t encoder_value[2] = {0};
float Joystick_Points_value[4];

void data_change(void)
{

		key_scan();

		encoder_value[0] = encoder_l();
		encoder_value[1] = encoder_r();

		Joystick_value[0] = map0_4096To128_128WithMaskOpt(adc_value[2]);
		Joystick_value[1] = map0_4096To128_128WithMaskOpt(adc_value[3]);
		Joystick_value[2] = map0_4096To128_128WithMaskOpt(adc_value[1]);
		Joystick_value[3] = map0_4096To128_128WithMaskOpt(adc_value[0]);

		Joystick_Points_value[0] = Joystick_Points_value[0] + Joystick_value[0] * Joystick_Points_kp;
		Joystick_Points_value[1] = Joystick_Points_value[1] + Joystick_value[1] * Joystick_Points_kp;

		Joystick_Points_value[1] = limit_float(Joystick_Points_value[1], -1.6f, 1.6f);

		floatToTwoSint8(Joystick_Points_value[0],&Send_Out[4],&Send_Out[5]);

		floatToTwoSint8Milli(Joystick_Points_value[1],&Send_Out[6],&Send_Out[7]);
		floatToTwoSint8(Joystick_value[2],&Send_Out[8],&Send_Out[9]);
		floatToTwoSint8(Joystick_value[3],&Send_Out[10],&Send_Out[11]);

		Send_Out[0]  = FRAME13_HEAD;
		Send_Out[1]  = key_num;
		Send_Out[2]  = encoder_value[0];
		Send_Out[3]  = encoder_value[1];
		Send_Out[F13_TAIL_IDX] = FRAME13_TAIL;

}

void data_print(void)
{
		for(int i=0;i<FRAME13_LEN;i++){
			printf("%2x,",Send_Out[i]);
		}printf("\r\n");

}
void oled_show_data(void)
{

}

int16_t map0_4096To128_128WithMaskOpt(uint16_t raw_value)
{
    int32_t mapped = (int32_t)raw_value * 256 / 4096 - 128;
    int32_t sign;

    mapped = (mapped > JOY_MAX) ? JOY_MAX : (mapped < -JOY_MAX) ? -JOY_MAX : mapped;

    sign   = (mapped < 0) ? -1 : 1;
    mapped = mapped * sign;

    if (mapped <= JOY_DEADZONE) {
        return 0;
    }

    mapped = (mapped - JOY_DEADZONE) * JOY_MAX / (JOY_MAX - JOY_DEADZONE);

    return (int16_t)(mapped * sign);
}

void floatToTwoSint8Milli(float num, uint8_t* high_byte, uint8_t* low_byte)
{
    if(num > 32.767f) num = 32.767f;
    if(num < -32.768f) num = -32.768f;
    int16_t scaled = (int16_t)(round(num * 1000.0f));
    *high_byte = (scaled >> 8) & 0xFF;
    *low_byte = scaled & 0xFF;
}

void floatToTwoSint8(float num, uint8_t* high_byte, uint8_t* low_byte)
{

    if(num > 327.67f) num = 327.67f;
    if(num < -327.68f) num = -327.68f;

    int16_t scaled = (int16_t)(round(num * 100.0f));

    *high_byte = (scaled >> 8) & 0xFF;
    *low_byte = scaled & 0xFF;
}

float twoSint8ToFloat(uint8_t high_byte, uint8_t low_byte)
{

    int16_t scaled = (int16_t)((high_byte << 8) | low_byte);

    return (float)scaled / 100.0f;
}

float limit_float(float val, float min, float max)
{
    val = (val < min) ? min : val;
    val = (val > max) ? max : val;
    return val;
}

uint8_t KeyArr_To_KeyNum(uint8_t key_arr[10])
{
    if(key_arr == NULL) return 0xFF;

    uint8_t key_num = 0;

    for(uint8_t i = 0; i < 8; i++)
    {

        if(key_arr[i] == 1)
        {
            key_num |= (1 << (7 - i));
        }

    }

    return key_num;
}

void KeyNum_To_BinStr(uint8_t key_num, char *bin_str)
{
    if(bin_str == NULL) return;
    uint8_t i;

    for(i = 0; i < 8; i++)
    {

        bin_str[i] = (key_num & (1 << (7 - i))) ? '1' : '0';
    }
    bin_str[8] = '\0';
}

void oled_show_keynum_bin(uint8_t x, uint8_t y, uint8_t key_num, uint8_t size)
{
    char bin_str[9] = {0};
    KeyNum_To_BinStr(key_num, bin_str);

    oled_show_string(x, y, "Key:", size);
    oled_show_string(x + size*2, y, bin_str, size);
}
