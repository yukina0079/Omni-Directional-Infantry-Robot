#include "LED.h"
#include "sys.h"

//³õÊ¼»¯GPIO¿Ú
void lde_init(void)
{
	
		GPIO_InitTypeDef gpio_initstruct;
		__HAL_RCC_GPIOB_CLK_ENABLE();		
	
		gpio_initstruct.Pin = GPIO_PIN_0;
		gpio_initstruct.Mode = GPIO_MODE_OUTPUT_PP;
		gpio_initstruct.Pull = GPIO_PULLDOWN;
		gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB,&gpio_initstruct);
	

}


void lde1_open(void)	{HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);}
void lde1_close(void)	{HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);}
void lde2_open(void)	{HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);}
void lde2_close(void)	{HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);}
void lde3_open(void)	{HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);}
void lde3_close(void)	{HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);}

void lde1_toggl(void)		{HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);}
void lde2_toggl(void)		{HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);}
void lde3_toggl(void)		{HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_7);}

void lde_toggl(void)
{
	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);
	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_7);
	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);
}
