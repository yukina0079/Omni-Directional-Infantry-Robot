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


/*
 * 13-byte frame, same layout on every link (see Drivers/frame13.h):
 *
 *   0x55  key  enl  enr  lxH lxL  lyH lyL  rxH rxL  ryH ryL  0xFF
 *
 * lx is the INTEGRATED yaw angle (rad*100), ly the integrated pitch angle
 * (rad*1000), rx/ry the instantaneous right stick (+-127).
 */
void data_change(void)
{
	
		// 锟斤拷取锟斤拷锟斤拷状态
		key_scan(); 
		
		// 锟斤拷取锟斤拷锟斤拷锟斤拷值
		encoder_value[0] = encoder_l();
		encoder_value[1] = encoder_r();	
	
		// 遥锟斤拷锟斤拷值-128 锟斤拷 +128
		Joystick_value[0] = map0_4096To128_128WithMaskOpt(adc_value[2]); // lx
		Joystick_value[1] = map0_4096To128_128WithMaskOpt(adc_value[3]); // ly
		Joystick_value[2] = map0_4096To128_128WithMaskOpt(adc_value[1]); // rl
		Joystick_value[3] = map0_4096To128_128WithMaskOpt(adc_value[0]); // ry
		
		// 遥锟斤拷锟斤拷锟斤拷锟斤拷值
		Joystick_Points_value[0] = Joystick_Points_value[0] + Joystick_value[0] * Joystick_Points_kp;// lx
		Joystick_Points_value[1] = Joystick_Points_value[1] + Joystick_value[1] * Joystick_Points_kp;// ly
	
		/*
		 * Wide enough that this integrator is never the travel limit.
		 * Real pitch ends are enforced on the motor board from the
		 * AS5600 absolute angle. 卤1.6 rad (~92 deg) covers any start
		 * pose inside a ~50 deg mechanical window.
		 */
		Joystick_Points_value[1] = limit_float(Joystick_Points_value[1], -1.6f, 1.6f);
//		Joystick_Points_value[0] = limit_float(Joystick_Points_value[0],-1.0f ,1.0f);
	

		floatToTwoSint8(Joystick_Points_value[0],&Send_Out[4],&Send_Out[5]);// lx
		/* 0.001 rad LSB (0.057 deg). The old *100 scale stepped 0.57 deg. */
		floatToTwoSint8Milli(Joystick_Points_value[1],&Send_Out[6],&Send_Out[7]);// ly
		floatToTwoSint8(Joystick_value[2],&Send_Out[8],&Send_Out[9]);		// rl
		floatToTwoSint8(Joystick_value[3],&Send_Out[10],&Send_Out[11]);		// ry
		
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
	
//        printf("%x,%d,%d\r\n",key_num,encoder_value[0],encoder_value[1]);
//		printf("%d,%d,%d,%d\r\n",adc_value[0],adc_value[1],adc_value[2],adc_value[3]);
//		printf("%.2f,%.2f\r\n",Joystick_Points_value[0],Joystick_Points_value[1]);
	
//		printf("%d,%d,%d,%d\r\n",Joystick_value[0],Joystick_value[1],Joystick_value[2],Joystick_value[3]);

//		HAL_UART_Transmit(&g_uart1_handle, Send_Out, sizeof(Send_Out), 1000);
//		printf("\r\n");
}
void oled_show_data(void)
{

//		oled_fill(0x00);

//		snprintf(lx_buffer, 8,"lx:%.2f",Joystick_Points_value[0]);
//		snprintf(ly_buffer, 8,"ly:%.2f",Joystick_Points_value[1]);
//		snprintf(rx_buffer, sizeof(rx_buffer),"ry:%+4d",Joystick_value[2]);
//		snprintf(ry_buffer, sizeof(ry_buffer),"rx:%+4d",Joystick_value[3]);
		
//		snprintf(l_en, sizeof(l_en),"len:%+4d",encoder_value[0]);
//		snprintf(r_en, sizeof(r_en),"ren:%+4d",encoder_value[1]);	
		
//		oled_show_string(0 , 0,l_en, 12);
//		oled_show_string(64, 0,r_en, 12);
		
//		oled_show_string(0,  2,lx_buffer, 12);
//		oled_show_string(64, 2,ry_buffer, 12);
//		oled_show_string(0,  4,ly_buffer, 12);
//		oled_show_string(64, 4,rx_buffer, 12);	
	
//		oled_show_keynum_bin(0, 6, key_num, 12);
}



/*
 * Maps a 12-bit ADC reading to [-128, 128] with a rescaled dead zone.
 *
 * The dead zone is not simply blanked. Blanking leaves a step: with the old
 * +-20 band, the reading went straight from 0 to 21 as the stick crossed the
 * edge, and since lx/ly are integrated at 500 Hz that was a jump from standstill
 * to 2.1 rad/s (120 deg/s). Small stick movements produced nothing, then a
 * lunge -- there was no low-rate region anywhere on the stick.
 *
 * Subtracting the dead zone and rescaling the remainder back to full span fixes
 * the shape: output leaves zero continuously (one count just outside the band,
 * about 0.05 rad/s) and still reaches +-JOY_MAX (127) at the mechanical stops, so
 * no top-end authority is given up. See JOY_DEADZONE in data.h for why the band
 * has to exist at all.
 */
int16_t map0_4096To128_128WithMaskOpt(uint16_t raw_value)
{
    int32_t mapped = (int32_t)raw_value * 256 / 4096 - 128;
    int32_t sign;

    /*
     * Clamp to +-JOY_MAX, not +-128. A 12-bit ADC cannot return 4096, so the
     * expression above already tops out at 127 and the +-128 clamp only ever
     * mattered as a theoretical guard -- but it is the wrong bound: the value
     * ends up in an int8_t on every receiving board, where 128 reads as -128.
     */
    mapped = (mapped > JOY_MAX) ? JOY_MAX : (mapped < -JOY_MAX) ? -JOY_MAX : mapped;

    /* Work on the magnitude so the rescale is symmetric about centre. */
    sign   = (mapped < 0) ? -1 : 1;
    mapped = mapped * sign;

    if (mapped <= JOY_DEADZONE) {
        return 0;
    }

    /*
     * Rescale the live range onto [0, JOY_MAX]. The denominator is
     * (JOY_MAX - JOY_DEADZONE), so full deflection maps to exactly JOY_MAX:
     * the old 128/(128-DZ) form topped out at 126, quietly throwing away the
     * last count of authority on every axis.
     */
    mapped = (mapped - JOY_DEADZONE) * JOY_MAX / (JOY_MAX - JOY_DEADZONE);

    return (int16_t)(mapped * sign);
}
/**
  * @brief  锟斤拷锟斤拷锟脚革拷锟斤拷锟斤拷转锟斤拷锟斤拷uint8_t锟斤拷锟斤拷围-327.68~327.67锟斤拷锟斤拷位小锟斤拷锟斤拷
  * @param  num: 锟斤拷锟诫浮锟斤拷锟斤拷锟斤拷锟斤拷围-327.68~327.67锟斤拷
  * @param  high_byte: 锟斤拷位锟街斤拷锟斤拷锟
  * @param  low_byte: 锟斤拷位锟街斤拷锟斤拷锟
  * @note   锟斤拷锟斤拷0.01锟斤拷锟斤拷效锟斤拷围-327.68~327.67
  */
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
    // 锟斤拷围锟斤拷椋拷锟窖★拷锟
    if(num > 327.67f) num = 327.67f;
    if(num < -327.68f) num = -327.68f;
    
    // 锟脚达拷100锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
    int16_t scaled = (int16_t)(round(num * 100.0f));
    
    // 锟斤拷int16_t锟斤拷锟轿拷锟斤拷锟斤拷纸锟
    *high_byte = (scaled >> 8) & 0xFF;
    *low_byte = scaled & 0xFF;
}

/**
  * @brief  锟斤拷锟斤拷uint8_t锟斤拷原为锟斤拷锟斤拷锟脚革拷锟斤拷锟斤拷
  * @param  high_byte: 锟斤拷位锟街斤拷
  * @param  low_byte: 锟斤拷位锟街斤拷
  * @retval 锟斤拷原锟侥革拷锟斤拷锟斤拷锟斤拷锟斤拷围-327.68~327.67锟斤拷
  */
float twoSint8ToFloat(uint8_t high_byte, uint8_t low_byte) 
{
    // 锟斤拷锟轿猧nt16_t锟斤拷注锟解：锟斤拷位锟斤拷要锟斤拷锟斤拷锟斤拷展锟斤拷
    int16_t scaled = (int16_t)((high_byte << 8) | low_byte);
    
    // 锟斤拷小100锟斤拷
    return (float)scaled / 100.0f;
}
// 全锟斤拷通锟斤拷 锟斤拷锟斤拷锟睫凤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷值锟斤拷Keil 锟斤拷锟斤拷锟斤拷锟斤拷
float limit_float(float val, float min, float max)
{
    val = (val < min) ? min : val;
    val = (val > max) ? max : val;
    return val;
}
/**
 * @brief  锟斤拷10位锟斤拷锟斤拷锟斤拷锟斤拷key_arr转锟斤拷为8位key_num锟斤拷映锟斤拷前8锟斤拷锟斤拷锟斤拷锟斤拷
 * @param  key_arr  锟斤拷锟诫：10位锟斤拷锟斤拷状态锟斤拷锟介（元锟斤拷为0/1锟斤拷0=锟斤拷锟铰ｏ拷1=未锟斤拷锟斤拷
 * @retval key_num  锟斤拷锟斤拷锟8位锟睫凤拷锟斤拷锟斤拷锟斤拷锟斤拷每一位锟斤拷应key_arr前8锟斤拷锟斤拷锟斤拷状态
 * @note   映锟斤拷锟斤拷锟絢ey_arr[0]锟斤拷key_num锟斤拷7位锟斤拷key_arr[1]锟斤拷锟斤拷6位...key_arr[7]锟斤拷锟斤拷0位
 */
uint8_t KeyArr_To_KeyNum(uint8_t key_arr[10])
{
    if(key_arr == NULL) return 0xFF; // 锟斤拷指锟诫保锟斤拷锟斤拷锟斤拷锟斤拷默锟斤拷值锟斤拷全1=锟睫帮拷锟斤拷锟斤拷
    
    uint8_t key_num = 0; // 锟斤拷始锟斤拷key_num为0
    
    // 锟斤拷锟斤拷key_arr前8位锟斤拷映锟戒到key_num锟斤拷8锟斤拷锟斤拷锟斤拷锟斤拷位
    for(uint8_t i = 0; i < 8; i++)
    {
        // 锟斤拷锟斤拷key_arr[i]为0锟斤拷锟斤拷锟斤拷锟斤拷锟铰ｏ拷时锟斤拷锟斤拷key_num锟斤拷应位锟斤拷0锟斤拷锟斤拷锟斤拷为1锟斤拷未锟斤拷锟斤拷
        // 锟竭硷拷锟斤拷key_num锟侥碉拷(7-i)位 = key_arr[i]锟斤拷1=未锟斤拷锟斤拷0=锟斤拷锟铰ｏ拷
        if(key_arr[i] == 1)
        {
            key_num |= (1 << (7 - i)); // 未锟斤拷锟斤拷锟接ξ伙拷锟1
        }
        // 锟斤拷锟斤拷锟斤拷锟接ξ伙拷锟斤拷锟0锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟絢ey_num锟斤拷始为0锟斤拷
    }
    
    return key_num;
}
// 锟斤拷锟斤拷锟斤拷锟斤拷8位key_num转为"11111111"锟斤拷式锟侥达拷锟斤拷锟斤拷锟斤拷锟街凤拷锟斤拷
void KeyNum_To_BinStr(uint8_t key_num, char *bin_str)
{
    if(bin_str == NULL) return; // 锟斤拷指锟诫保锟斤拷
    uint8_t i;
    // 锟斤拷位锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟轿伙拷锟斤拷锟7位锟斤拷锟斤拷锟斤拷锟轿伙拷锟斤拷锟0位锟斤拷
    for(i = 0; i < 8; i++)
    {
        // 锟斤拷位锟斤拷锟叫断碉拷前位锟斤拷1锟斤拷锟斤拷0锟斤拷转为锟街凤拷'1'/'0'
        bin_str[i] = (key_num & (1 << (7 - i))) ? '1' : '0';
    }
    bin_str[8] = '\0'; // 锟街凤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
}

// 锟斤拷锟斤拷锟斤拷锟斤拷OLED锟斤拷锟斤拷示key_num锟侥讹拷锟斤拷锟斤拷值
void oled_show_keynum_bin(uint8_t x, uint8_t y, uint8_t key_num, uint8_t size)
{
    char bin_str[9] = {0}; // 锟芥储8位锟斤拷锟斤拷锟斤拷锟街凤拷锟斤拷锟斤拷+锟斤拷锟斤拷锟斤拷锟斤拷
    KeyNum_To_BinStr(key_num, bin_str); // 转锟斤拷为锟斤拷锟斤拷锟斤拷锟街凤拷锟斤拷
    
    // 锟斤拷锟斤拷示锟斤拷签锟斤拷锟斤拷锟斤拷示锟斤拷锟斤拷锟斤拷值锟斤拷示锟斤拷锟斤拷锟斤拷签"Key:" + 锟斤拷锟斤拷锟斤拷值锟斤拷
    oled_show_string(x, y, "Key:", size);          // 锟斤拷示锟斤拷签
    oled_show_string(x + size*2, y, bin_str, size); // 锟斤拷示锟斤拷锟斤拷锟斤拷值锟斤拷偏锟狡憋拷签锟斤拷锟饺ｏ拷
}
