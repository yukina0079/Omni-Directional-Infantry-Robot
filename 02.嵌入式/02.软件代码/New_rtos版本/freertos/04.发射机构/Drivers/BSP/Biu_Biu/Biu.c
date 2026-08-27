#include "Biu.h"

TIM_HandleTypeDef biu_pwm_handle = {0};	
TIM_HandleTypeDef biu2_pwm_handle = {0};

void Biu_pwm_init(uint16_t arr,uint16_t psc)
{
		TIM_OC_InitTypeDef pwm_config = {0};
		TIM_OC_InitTypeDef pwm2_config = {0};
		
		biu_pwm_handle.Instance = TIM2;					//定时器通道
		biu_pwm_handle.Init.Prescaler = psc;
		biu_pwm_handle.Init.Period = arr;
		biu_pwm_handle.Init.CounterMode = TIM_COUNTERMODE_UP;//向上计数
		
		HAL_TIM_PWM_Init(&biu_pwm_handle);
		
		pwm_config.OCMode = TIM_OCMODE_PWM1; //通用定时器
		pwm_config.Pulse  = 0;
		pwm_config.OCPolarity = TIM_OCPOLARITY_HIGH;//计数方式
		

	
		biu2_pwm_handle.Instance = TIM3;					//定时器通道
		biu2_pwm_handle.Init.Prescaler = psc;
		biu2_pwm_handle.Init.Period = arr;
		biu2_pwm_handle.Init.CounterMode = TIM_COUNTERMODE_UP;//向上计数
		
		HAL_TIM_PWM_Init(&biu2_pwm_handle);
		
		pwm2_config.OCMode = TIM_OCMODE_PWM1; //通用定时器
		pwm2_config.Pulse  = 0;
		pwm2_config.OCPolarity = TIM_OCPOLARITY_HIGH;//计数方式

		
		HAL_TIM_PWM_ConfigChannel(&biu_pwm_handle,&pwm_config,TIM_CHANNEL_3);
		HAL_TIM_PWM_ConfigChannel(&biu_pwm_handle,&pwm_config,TIM_CHANNEL_4);
		HAL_TIM_PWM_ConfigChannel(&biu2_pwm_handle,&pwm2_config,TIM_CHANNEL_3);
		HAL_TIM_PWM_ConfigChannel(&biu2_pwm_handle,&pwm2_config,TIM_CHANNEL_4);
		

		HAL_TIM_PWM_Start(&biu_pwm_handle,TIM_CHANNEL_3); 
		HAL_TIM_PWM_Start(&biu_pwm_handle,TIM_CHANNEL_4);		
		HAL_TIM_PWM_Start(&biu2_pwm_handle,TIM_CHANNEL_3); 
		HAL_TIM_PWM_Start(&biu2_pwm_handle,TIM_CHANNEL_4); 		

}

void Biu_pwm_compare_set(uint16_t val)
{
		__HAL_TIM_SET_COMPARE(&biu2_pwm_handle,TIM_CHANNEL_3,val);
		__HAL_TIM_SET_COMPARE(&biu2_pwm_handle,TIM_CHANNEL_4,val);
		__HAL_TIM_SET_COMPARE(&biu_pwm_handle,TIM_CHANNEL_3,val);
		__HAL_TIM_SET_COMPARE(&biu_pwm_handle,TIM_CHANNEL_4,val);
}



