#include "key.h"
#include "sys.h"
#include "delay.h"

#define Key_port GPIOB
#define Key1_pin GPIO_PIN_5
void key_init(void)
{

		GPIO_InitTypeDef gpio_initstruct;
		__HAL_RCC_GPIOB_CLK_ENABLE();

		gpio_initstruct.Pin = Key1_pin;
		gpio_initstruct.Mode = GPIO_MODE_INPUT;
		gpio_initstruct.Pull = GPIO_PULLUP;
		gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;

		HAL_GPIO_Init(Key_port,&gpio_initstruct);
}

uint8_t key_scan(void)
{
		if( HAL_GPIO_ReadPin(Key_port,Key1_pin) == GPIO_PIN_RESET){
				delay_ms(10);
			if( HAL_GPIO_ReadPin(Key_port,Key1_pin) == GPIO_PIN_RESET){
				while(HAL_GPIO_ReadPin(Key_port,Key1_pin) == GPIO_PIN_RESET);
				return 1;
			}
		}return 0;

}
