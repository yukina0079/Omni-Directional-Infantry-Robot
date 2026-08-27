#include "key.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "oled.h"
#include "data.h"

void key_init(void)
{
		__HAL_RCC_AFIO_CLK_ENABLE();
		__HAL_AFIO_REMAP_SWJ_NOJTAG(); // 禁用JTAG，保留SWD
	
		GPIO_InitTypeDef gpio1_initstruct;//定义结构体参数
		GPIO_InitTypeDef gpio2_initstruct;//定义结构体参数
		GPIO_InitTypeDef gpio3_initstruct;//定义结构体参数
	
		__HAL_RCC_GPIOA_CLK_ENABLE();		//开启GPIOA组引脚
		__HAL_RCC_GPIOB_CLK_ENABLE();		//开启GPIOA组引脚
		__HAL_RCC_GPIOC_CLK_ENABLE();		//开启GPIOA组引脚
	
		gpio1_initstruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_15;//配置引脚号
		gpio1_initstruct.Mode = GPIO_MODE_INPUT;//配置工作模式 推挽输出
		gpio1_initstruct.Pull = GPIO_PULLUP;//配置上拉下拉
		gpio1_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;//配置输出速度
	
		gpio2_initstruct.Pin = GPIO_PIN_0|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;//配置引脚号
		gpio2_initstruct.Mode = GPIO_MODE_INPUT;//配置工作模式 推挽输出
		gpio2_initstruct.Pull = GPIO_PULLUP;//配置上拉下拉
		gpio2_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;//配置输出速度
	
		gpio3_initstruct.Pin = GPIO_PIN_13;//配置引脚号
		gpio3_initstruct.Mode = GPIO_MODE_INPUT;//配置工作模式 推挽输出
		gpio3_initstruct.Pull = GPIO_PULLUP;//配置上拉下拉
		gpio3_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;//配置输出速度	
		
		HAL_GPIO_Init(GPIOA,&gpio1_initstruct);
		HAL_GPIO_Init(GPIOB,&gpio2_initstruct);
		HAL_GPIO_Init(GPIOC,&gpio3_initstruct);
}


void key_scan(void)
{
    uint8_t curr_state; // 临时存储当前按键状态
    
//    if((curr_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) == GPIO_PIN_RESET && key_prev_state[0] == GPIO_PIN_SET) 
//	{key_arr[0] ^= 1;} key_prev_state[0] = curr_state; // 按键1：GPIOC_PIN_13
    if((curr_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)) == GPIO_PIN_RESET && key_prev_state[1] == GPIO_PIN_SET) 
	{key_arr[1] ^= 1;} key_prev_state[1] = curr_state; // 按键2：GPIOA_PIN_4
    if((curr_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)) == GPIO_PIN_RESET && key_prev_state[2] == GPIO_PIN_SET) 
	{key_arr[2] ^= 1;} key_prev_state[2] = curr_state; // 按键3：GPIOA_PIN_5
    if((curr_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)) == GPIO_PIN_RESET && key_prev_state[3] == GPIO_PIN_SET) 
	{key_arr[3] ^= 1;} key_prev_state[3] = curr_state; // 按键4：GPIOB_PIN_0
	if((curr_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)) == GPIO_PIN_RESET && key_prev_state[4] == GPIO_PIN_SET) 
	{key_arr[4] ^= 1;} key_prev_state[4] = curr_state; // 按键5：GPIOB_PIN_9
    if((curr_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)) == GPIO_PIN_RESET && key_prev_state[5] == GPIO_PIN_SET) 
	{key_arr[5] ^= 1;} key_prev_state[5] = curr_state; // 按键6：GPIOB_PIN_8
	if((curr_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)) == GPIO_PIN_RESET && key_prev_state[6] == GPIO_PIN_SET) 
	{key_arr[6] ^= 1;} key_prev_state[6] = curr_state; // 按键7：GPIOB_PIN_5
    if((curr_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)) == GPIO_PIN_RESET && key_prev_state[7] == GPIO_PIN_SET) 
	{key_arr[7] ^= 1;} key_prev_state[7] = curr_state; // 按键8：GPIOB_PIN_4
	key_num = KeyArr_To_KeyNum(key_arr);
}




