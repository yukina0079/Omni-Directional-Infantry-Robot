#ifndef  __DATA_H__
#define __DATA_H__
#include "sys.h"
#include "frame13.h"

#define Joystick_Points_kp 0.0001

#define JOY_MAX 127

#define JOY_DEADZONE 8

extern uint8_t key_num;
extern char key_num_buffer[20];
extern uint8_t key_prev_state[10];
extern uint8_t key_arr[10];
extern uint16_t adc_value[4];
extern int16_t Joystick_value[4];
extern int16_t encoder_value[2];
extern float Joystick_Points_value[4];
extern uint8_t Send_Out[FRAME13_LEN];

int16_t map0_4096To128_128WithMaskOpt(uint16_t raw_value);
void floatToTwoSint8(float num, uint8_t* high_byte, uint8_t* low_byte);
void floatToTwoSint8Milli(float num, uint8_t* high_byte, uint8_t* low_byte);
float twoSint8ToFloat(uint8_t high_byte, uint8_t low_byte);

uint8_t KeyArr_To_KeyNum(uint8_t key_arr[10]);
void KeyNum_To_BinStr(uint8_t key_num, char *bin_str);
void oled_show_keynum_bin(uint8_t x, uint8_t y, uint8_t key_num, uint8_t size);
float limit_float(float val, float min, float max);
void data_change(void);
void data_print(void);
void oled_show_data(void);

#endif
