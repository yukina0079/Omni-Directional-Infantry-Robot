#include "data.h"
#include "stdio.h"
#include "motor.h"
#include "nrf24L01.h"
#include "delay.h"
#include "led.h"
#include "math.h"
#include "ws2812b.h"
#include "Biu.h"

#define CONTROL_LINK_TIMEOUT_MS 200U

uint8_t Receive[33];

int speed[2];

static uint8_t control_link_active = 0U;
static uint32_t last_valid_frame_ms = 0U;

void laser_contral(void)
{
	if((control_link_active != 0U) && (Receive[0] == 0x55) && (Receive[12] == 0xff)){
		if((Receive[1] & 0x02) != 0x02){
			LASER(1);  /* 按下 KEY2 (0x02)，PB6 推挽输出高电平点亮激光 */
		}else{
			LASER(0);  /* PB6 输出低电平关闭激光 */
		}
	}else{
		LASER(0);      /* 无线断线超时切断激光 */
	}
}

void motor_contral(void)
{
	Biu_pwm_compare_set(speed[0]);
	laser_contral();
	
	if((control_link_active != 0U) && (Receive[0] == 0x55) && (Receive[12] == 0xff) && (speed[0] > 1024)){
		if((Receive[1] & 0x20) != 0x20){
			GO_forward();	
			pwm1_compare_set(300);
		}else if((Receive[1] & 0x10) != 0x10){
			GO_back();	
			pwm1_compare_set(500);
		}else{pwm1_compare_set(0);}
	}else{pwm1_compare_set(0);}
}
void light_contral(void)
{
	if((Receive[0] == 0x55) && (Receive[12] == 0xff) && (speed[0] > 1024)){
		if((speed[1] < 75) && ((Receive[1] & 0x20) != 0x20)){
			WS2812_FlowLight(0,0,255,speed[1],0);	
		}else{WS2812_FlowLight(0,0,0  ,speed[1],0);}
	}else{WS2812_FlowLight(0,0,0  ,speed[1],0);}
	LED2_TOGGLE();
}
void data_change(void)
{
	uint32_t now_ms = HAL_GetTick();

	if(NRF24L01_RxPacket(Receive) == 0U)
	{
		if((Receive[0] == 0x55) && (Receive[12] == 0xff))
		{
			speed[0] = LIMIT(1000 + Receive[3] * 3, 0, 1200);
			speed[1] = LIMIT(70 - Receive[3], 10, 70);
			last_valid_frame_ms = now_ms;
			control_link_active = 1U;
		}
	}

	if((control_link_active != 0U) &&
	   ((uint32_t)(now_ms - last_valid_frame_ms) >= CONTROL_LINK_TIMEOUT_MS))
	{
		control_link_active = 0U;
		speed[0] = 1000;
	}
}
void data_print(void)
{
//		printf("进入数据接收模式\n");
		for(int i=0;i<20;i++){
			printf("%2x ",Receive[i]);
		}printf("\n");

}
/**
  * @brief  带符号浮点数转两个uint8_t（范围-327.68~327.67，两位小数）
  * @param  num: 输入浮点数（范围-327.68~327.67）
  * @param  high_byte: 高位字节输出
  * @param  low_byte: 低位字节输出
  * @note   精度0.01，有效范围-327.68~327.67
  */
void float_to_two_uint8_signed(float num, uint8_t* high_byte, uint8_t* low_byte) 
{
    // 范围检查（可选）
    if(num > 327.67f) num = 327.67f;
    if(num < -327.68f) num = -327.68f;
    
    // 放大100倍并四舍五入
    int16_t scaled = (int16_t)(round(num * 100.0f));
    
    // 将int16_t拆分为两个字节
    *high_byte = (scaled >> 8) & 0xFF;
    *low_byte = scaled & 0xFF;
}

/**
  * @brief  两个uint8_t还原为带符号浮点数
  * @param  high_byte: 高位字节
  * @param  low_byte: 低位字节
  * @retval 还原的浮点数（范围-327.68~327.67）
  */
float two_uint8_to_float_signed(uint8_t high_byte, uint8_t low_byte) 
{
    // 组合为int16_t（注意：高位需要符号扩展）
    int16_t scaled = (int16_t)((high_byte << 8) | low_byte);
    
    // 缩小100倍
    return (float)scaled / 100.0f;
}


