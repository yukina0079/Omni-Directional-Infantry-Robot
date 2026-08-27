#include "key.h"
#include "sys.h"
#include "delay.h"
//初始化GPIO口
#define Key_port GPIOB
#define Key1_pin GPIO_PIN_5
void key_init(void)
{
	
		GPIO_InitTypeDef gpio_initstruct;//定义结构体参数
		__HAL_RCC_GPIOB_CLK_ENABLE();		//开启GPIOA组引脚
	
		gpio_initstruct.Pin = Key1_pin;//配置引脚号
		gpio_initstruct.Mode = GPIO_MODE_INPUT;//配置工作模式 推挽输出
		gpio_initstruct.Pull = GPIO_PULLUP;//配置上拉下拉
		gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;//配置输出速度
	
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
