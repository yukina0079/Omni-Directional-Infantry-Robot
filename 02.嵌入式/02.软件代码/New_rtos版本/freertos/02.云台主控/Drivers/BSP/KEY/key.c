#include "key.h"
#include "sys.h"
#include "delay.h"

#define key1_port GPIOC
#define key1_pin  GPIO_PIN_2
#define key1_rcc  __HAL_RCC_GPIOC_CLK_ENABLE();	

#define key2_port GPIOC
#define key2_pin  GPIO_PIN_3
#define key2_rcc  __HAL_RCC_GPIOC_CLK_ENABLE();

void key_init(void)
{
	
		GPIO_InitTypeDef gpio_initstruct;//定义结构体参数
		key2_rcc;
	
		gpio_initstruct.Pin = key1_pin|key2_pin;//配置引脚号
		gpio_initstruct.Mode = GPIO_MODE_INPUT;//配置工作模式 推挽输出
		gpio_initstruct.Pull = GPIO_PULLUP;//配置上拉下拉
		gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;//配置输出速度
	
		HAL_GPIO_Init(key1_port,&gpio_initstruct);
}

uint8_t key_scan(void)
{
		if( HAL_GPIO_ReadPin(key1_port,key1_pin) == GPIO_PIN_RESET){
				delay_ms(10);
			if( HAL_GPIO_ReadPin(key1_port,key1_pin) == GPIO_PIN_RESET){
				while(HAL_GPIO_ReadPin(key1_port,key1_pin) == GPIO_PIN_RESET);
				return 1;
			}
		}return 0;
			
	
}
