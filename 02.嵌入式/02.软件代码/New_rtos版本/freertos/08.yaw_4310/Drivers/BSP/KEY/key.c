#include "key.h"
#include "sys.h"
#include "delay.h"

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
