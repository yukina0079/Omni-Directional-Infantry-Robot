#include "pwm.h"

//TIM_HandleTypeDef pwm_handle = {0};			//定义定时器参数结构体

//void pwm_init(uint16_t arr,uint16_t psc)
//{
//		TIM_OC_InitTypeDef pwm_config = {0};
//	
//		pwm_handle.Instance = TIM2;					
//		pwm_handle.Init.Prescaler = psc;
//		pwm_handle.Init.Period = arr;
//		pwm_handle.Init.CounterMode = TIM_COUNTERMODE_UP;//向上计数
//		
//		HAL_TIM_PWM_Init(&pwm_handle);
//		
//		pwm_config.OCMode = TIM_OCMODE_PWM1; //通用定时器
//		pwm_config.Pulse  = 0;
//		pwm_config.OCPolarity = TIM_OCPOLARITY_LOW;//计数方式
//		HAL_TIM_PWM_ConfigChannel(&pwm_handle,&pwm_config,TIM_CHANNEL_3);
//		
//		HAL_TIM_PWM_Start(&pwm_handle,TIM_CHANNEL_3); //开启定时器
//}
//void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)		//PWM硬件参数
//{
//	if(htim->Instance == TIM2){
//		GPIO_InitTypeDef gpio_initstruct;//定义结构体参数
//			
//		__HAL_RCC_TIM2_CLK_ENABLE();		//定时器4时钟使能	
//		__HAL_RCC_GPIOA_CLK_ENABLE();		//开启GPIOB组引脚
//	
//		gpio_initstruct.Pin = GPIO_PIN_3;//配置引脚号
//		gpio_initstruct.Mode = GPIO_MODE_AF_PP;//配置工作模式 复用推挽输出
//		gpio_initstruct.Pull = GPIO_PULLUP;//配置上拉下拉
//		gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;//配置输出速度
//	
//		HAL_GPIO_Init(GPIOA,&gpio_initstruct);		
//	}
//}
void pwm_compare_set(uint16_t val)
{
//		__HAL_TIM_SET_COMPARE(&pwm_handle,TIM_CHANNEL_3,val);
}
