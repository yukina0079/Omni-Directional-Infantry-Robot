#include "key.h"
#include "sys.h"
#include "delay.h"
//³õÊ¼»¯GPIO¿Ú
void key_init(void)
{
	
		GPIO_InitTypeDef gpio_initstruct;
		__HAL_RCC_GPIOC_CLK_ENABLE();		
	
		gpio_initstruct.Pin = GPIO_PIN_13;
		gpio_initstruct.Mode = GPIO_MODE_INPUT;
		gpio_initstruct.Pull = GPIO_PULLUP;
		gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;
	
		HAL_GPIO_Init(GPIOC,&gpio_initstruct);
}

//uint8_t key_scan(void)
//{
//		if( HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == GPIO_PIN_RESET){
//				delay_ms(10);
//			if( HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == GPIO_PIN_RESET){
//				while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == GPIO_PIN_RESET);
//				return 1;
//			}
//		}return 0;
//			
//	
//}
