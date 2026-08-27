#include "LED.h"
#include "sys.h"

//³õÊ¼»¯GPIO¿Ú
void lde_init(void)
{
	
		GPIO_InitTypeDef gpio_initstruct;
		__HAL_RCC_GPIOB_CLK_ENABLE();		
	
		gpio_initstruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
		gpio_initstruct.Mode = GPIO_MODE_OUTPUT_PP;
		gpio_initstruct.Pull = GPIO_PULLUP;
		gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB,&gpio_initstruct);
	
		lde_close();
}

void lde1_on(void){HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);}
void lde1_off(void){HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);}
void lde1_toggle(void){HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_5);}
void lde2_on(void){HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);}
void lde2_off(void){HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);}
void lde2_toggle(void){HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_6);}
void lde3_on(void){HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);}
void lde3_off(void){HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);}
void lde3_toggle(void){HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_7);}
void lde_close(void)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
}


