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
uint8_t Send_Out[32] = {0};
int16_t Joystick_value[4] = {0};
int16_t encoder_value[2] = {0};
float Joystick_Points_value[4];


/*
	֡��ʽ
	0x55 ���� enl enr lxh lxl lyh lyl
	rxh  rxl  ryh ryl 0   0   0   0
	0    0    0   0xff
*/
void data_change(void)
{
	
		// ��ȡ����״̬
		key_scan(); 
		
		// ��ȡ������ֵ
		encoder_value[0] = encoder_l();
		encoder_value[1] = encoder_r();	
	
		// ң����ֵ-128 �� +128
		Joystick_value[0] = map0_4096To128_128WithMaskOpt(adc_value[2]); // lx
		Joystick_value[1] = map0_4096To128_128WithMaskOpt(adc_value[3]); // ly
		Joystick_value[2] = map0_4096To128_128WithMaskOpt(adc_value[1]); // rl
		Joystick_value[3] = map0_4096To128_128WithMaskOpt(adc_value[0]); // ry
		
		// ң��������ֵ
		Joystick_Points_value[0] = Joystick_Points_value[0] + Joystick_value[0] * Joystick_Points_kp;// lx
		Joystick_Points_value[1] = Joystick_Points_value[1] + Joystick_value[1] * Joystick_Points_kp;// ly
	
		/*
		 * Wide enough that this integrator is never the travel limit.
		 * Real pitch ends are enforced on the motor board from the
		 * AS5600 absolute angle. ±1.6 rad (~92 deg) covers any start
		 * pose inside a ~50 deg mechanical window.
		 */
		Joystick_Points_value[1] = limit_float(Joystick_Points_value[1], -1.6f, 1.6f);
//		Joystick_Points_value[0] = limit_float(Joystick_Points_value[0],-1.0f ,1.0f);
	

		floatToTwoSint8(Joystick_Points_value[0],&Send_Out[4],&Send_Out[5]);// lx
		/* 0.001 rad LSB (0.057 deg). The old *100 scale stepped 0.57 deg. */
		floatToTwoSint8Milli(Joystick_Points_value[1],&Send_Out[6],&Send_Out[7]);// ly
		floatToTwoSint8(Joystick_value[2],&Send_Out[8],&Send_Out[9]);		// rl
		floatToTwoSint8(Joystick_value[3],&Send_Out[10],&Send_Out[11]);		// ry
		
		Send_Out[0]  = 0x55;
		Send_Out[1]  = key_num;
		Send_Out[2]  = encoder_value[0];
		Send_Out[3]  = encoder_value[1];
		Send_Out[12] = 0xff;
		
}

void data_print(void)
{
		for(int i=0;i<20;i++){
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
 * about 0.05 rad/s) and still reaches exactly +-128 at the mechanical stops, so
 * no top-end authority is given up. See JOY_DEADZONE in data.h for why the band
 * has to exist at all.
 */
int16_t map0_4096To128_128WithMaskOpt(uint16_t raw_value)
{
    int32_t mapped = (int32_t)raw_value * 256 / 4096 - 128;
    int32_t sign;

    mapped = (mapped > 128) ? 128 : (mapped < -128) ? -128 : mapped;

    /* Work on the magnitude so the rescale is symmetric about centre. */
    sign   = (mapped < 0) ? -1 : 1;
    mapped = mapped * sign;

    if (mapped <= JOY_DEADZONE) {
        return 0;
    }

    /* Integer arithmetic is exact enough here: the 128/(128-DZ) factor is
     * applied before the divide, so no intermediate rounding is lost. */
    mapped = (mapped - JOY_DEADZONE) * 128 / (128 - JOY_DEADZONE);

    return (int16_t)(mapped * sign);
}
/**
  * @brief  �����Ÿ�����ת����uint8_t����Χ-327.68~327.67����λС����
  * @param  num: ���븡��������Χ-327.68~327.67��
  * @param  high_byte: ��λ�ֽ����
  * @param  low_byte: ��λ�ֽ����
  * @note   ����0.01����Ч��Χ-327.68~327.67
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
    // ��Χ��飨��ѡ��
    if(num > 327.67f) num = 327.67f;
    if(num < -327.68f) num = -327.68f;
    
    // �Ŵ�100������������
    int16_t scaled = (int16_t)(round(num * 100.0f));
    
    // ��int16_t���Ϊ�����ֽ�
    *high_byte = (scaled >> 8) & 0xFF;
    *low_byte = scaled & 0xFF;
}

/**
  * @brief  ����uint8_t��ԭΪ�����Ÿ�����
  * @param  high_byte: ��λ�ֽ�
  * @param  low_byte: ��λ�ֽ�
  * @retval ��ԭ�ĸ���������Χ-327.68~327.67��
  */
float twoSint8ToFloat(uint8_t high_byte, uint8_t low_byte) 
{
    // ���Ϊint16_t��ע�⣺��λ��Ҫ������չ��
    int16_t scaled = (int16_t)((high_byte << 8) | low_byte);
    
    // ��С100��
    return (float)scaled / 100.0f;
}
// ȫ��ͨ�� �����޷�������������ֵ��Keil ��������
float limit_float(float val, float min, float max)
{
    val = (val < min) ? min : val;
    val = (val > max) ? max : val;
    return val;
}
/**
 * @brief  ��10λ��������key_arrת��Ϊ8λkey_num��ӳ��ǰ8��������
 * @param  key_arr  ���룺10λ����״̬���飨Ԫ��Ϊ0/1��0=���£�1=δ����
 * @retval key_num  �����8λ�޷���������ÿһλ��Ӧkey_arrǰ8������״̬
 * @note   ӳ�����key_arr[0]��key_num��7λ��key_arr[1]����6λ...key_arr[7]����0λ
 */
uint8_t KeyArr_To_KeyNum(uint8_t key_arr[10])
{
    if(key_arr == NULL) return 0xFF; // ��ָ�뱣��������Ĭ��ֵ��ȫ1=�ް�����
    
    uint8_t key_num = 0; // ��ʼ��key_numΪ0
    
    // ����key_arrǰ8λ��ӳ�䵽key_num��8��������λ
    for(uint8_t i = 0; i < 8; i++)
    {
        // ����key_arr[i]Ϊ0���������£�ʱ����key_num��Ӧλ��0������Ϊ1��δ����
        // �߼���key_num�ĵ�(7-i)λ = key_arr[i]��1=δ����0=���£�
        if(key_arr[i] == 1)
        {
            key_num |= (1 << (7 - i)); // δ�����Ӧλ��1
        }
        // �������Ӧλ����0�������������key_num��ʼΪ0��
    }
    
    return key_num;
}
// ��������8λkey_numתΪ"11111111"��ʽ�Ĵ��������ַ���
void KeyNum_To_BinStr(uint8_t key_num, char *bin_str)
{
    if(bin_str == NULL) return; // ��ָ�뱣��
    uint8_t i;
    // ��λ�����������λ����7λ�������λ����0λ��
    for(i = 0; i < 8; i++)
    {
        // ��λ���жϵ�ǰλ��1����0��תΪ�ַ�'1'/'0'
        bin_str[i] = (key_num & (1 << (7 - i))) ? '1' : '0';
    }
    bin_str[8] = '\0'; // �ַ�������������������
}

// ��������OLED����ʾkey_num�Ķ�����ֵ
void oled_show_keynum_bin(uint8_t x, uint8_t y, uint8_t key_num, uint8_t size)
{
    char bin_str[9] = {0}; // �洢8λ�������ַ�����+��������
    KeyNum_To_BinStr(key_num, bin_str); // ת��Ϊ�������ַ���
    
    // ����ʾ��ǩ������ʾ������ֵ��ʾ������ǩ"Key:" + ������ֵ��
    oled_show_string(x, y, "Key:", size);          // ��ʾ��ǩ
    oled_show_string(x + size*2, y, bin_str, size); // ��ʾ������ֵ��ƫ�Ʊ�ǩ���ȣ�
}
