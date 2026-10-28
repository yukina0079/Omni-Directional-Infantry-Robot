#include "key.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "oled.h"
#include "data.h"

void key_init(void)
{
		__HAL_RCC_AFIO_CLK_ENABLE();
		__HAL_AFIO_REMAP_SWJ_NOJTAG();

		GPIO_InitTypeDef gpio1_initstruct;
		GPIO_InitTypeDef gpio2_initstruct;
		GPIO_InitTypeDef gpio3_initstruct;

		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		__HAL_RCC_GPIOC_CLK_ENABLE();

		gpio1_initstruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_15;
		gpio1_initstruct.Mode = GPIO_MODE_INPUT;
		gpio1_initstruct.Pull = GPIO_PULLUP;
		gpio1_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;

		gpio2_initstruct.Pin = GPIO_PIN_0|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
		gpio2_initstruct.Mode = GPIO_MODE_INPUT;
		gpio2_initstruct.Pull = GPIO_PULLUP;
		gpio2_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;

		gpio3_initstruct.Pin = GPIO_PIN_13;
		gpio3_initstruct.Mode = GPIO_MODE_INPUT;
		gpio3_initstruct.Pull = GPIO_PULLUP;
		gpio3_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;

		HAL_GPIO_Init(GPIOA,&gpio1_initstruct);
		HAL_GPIO_Init(GPIOB,&gpio2_initstruct);
		HAL_GPIO_Init(GPIOC,&gpio3_initstruct);
}

void key_scan(void)
{
    uint8_t curr_state;

    if((curr_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)) == GPIO_PIN_RESET && key_prev_state[1] == GPIO_PIN_SET)
	{key_arr[1] ^= 1;} key_prev_state[1] = curr_state;
    if((curr_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)) == GPIO_PIN_RESET && key_prev_state[2] == GPIO_PIN_SET)
	{key_arr[2] ^= 1;} key_prev_state[2] = curr_state;
    if((curr_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)) == GPIO_PIN_RESET && key_prev_state[3] == GPIO_PIN_SET)
	{key_arr[3] ^= 1;} key_prev_state[3] = curr_state;
	if((curr_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)) == GPIO_PIN_RESET && key_prev_state[4] == GPIO_PIN_SET)
	{key_arr[4] ^= 1;} key_prev_state[4] = curr_state;
    if((curr_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)) == GPIO_PIN_RESET && key_prev_state[5] == GPIO_PIN_SET)
	{key_arr[5] ^= 1;} key_prev_state[5] = curr_state;
	if((curr_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)) == GPIO_PIN_RESET && key_prev_state[6] == GPIO_PIN_SET)
	{key_arr[6] ^= 1;} key_prev_state[6] = curr_state;
    if((curr_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)) == GPIO_PIN_RESET && key_prev_state[7] == GPIO_PIN_SET)
	{key_arr[7] ^= 1;} key_prev_state[7] = curr_state;
	key_num = KeyArr_To_KeyNum(key_arr);
}
