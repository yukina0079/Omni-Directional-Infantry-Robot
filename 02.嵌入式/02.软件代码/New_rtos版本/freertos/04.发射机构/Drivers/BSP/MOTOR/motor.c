#include "motor.h"

/**
 * @file 电机引脚
 * motro:PB15 PB8  PWM:PA8 TIM1_CH1
 */
#define GPIO2_PORT GPIOB
#define GPIO_P1 GPIO_PIN_8
#define GPIO_P2 GPIO_PIN_15

#define FEEDER_DIRECTION_FORWARD 1U
#define FEEDER_DIRECTION_BACK    2U
#define FEEDER_REVERSE_DELAY_MS  10U

static uint8_t feeder_direction = 0U;


/*************************初始化**************************/
void motor_init(void)
{
		GPIO_InitTypeDef gpio_initstruct;
		
		__HAL_RCC_GPIOB_CLK_ENABLE();	
		gpio_initstruct.Pin = GPIO_P2|GPIO_P1;
		gpio_initstruct.Mode = GPIO_MODE_OUTPUT_PP;
		gpio_initstruct.Pull = GPIO_PULLUP;
		gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIO2_PORT,&gpio_initstruct);	
	
}
TIM_HandleTypeDef pwm_handle = {0};			//定义定时器参数结构体

void pwm_init(uint16_t arr,uint16_t psc)
{
		TIM_OC_InitTypeDef pwm_config = {0};
	
		pwm_handle.Instance = TIM1;					//定时器通道
		pwm_handle.Init.Prescaler = psc;
		pwm_handle.Init.Period = arr;
		pwm_handle.Init.CounterMode = TIM_COUNTERMODE_UP;//向上计数
		
		HAL_TIM_PWM_Init(&pwm_handle);
		
		pwm_config.OCMode = TIM_OCMODE_PWM1; //通用定时器
		pwm_config.Pulse  = 0;
		pwm_config.OCPolarity = TIM_OCPOLARITY_HIGH;//计数方式
		HAL_TIM_PWM_ConfigChannel(&pwm_handle,&pwm_config,TIM_CHANNEL_1);

		HAL_TIM_PWM_Start(&pwm_handle,TIM_CHANNEL_1); 

}
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)	
{
		if(htim->Instance == TIM1){
			
		GPIO_InitTypeDef Biu_initstruct;
			
		__HAL_RCC_TIM1_CLK_ENABLE();		
		__HAL_RCC_GPIOA_CLK_ENABLE();		
	
		Biu_initstruct.Pin = GPIO_PIN_8;//配置引脚号
		Biu_initstruct.Mode = GPIO_MODE_AF_PP;//配置工作模式 复用推挽输出
		Biu_initstruct.Pull = GPIO_PULLUP;//配置上拉下拉
		Biu_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;//配置输出速度
		
		HAL_GPIO_Init(GPIOA,&Biu_initstruct);
		}
		
		if(htim->Instance == TIM2){
			
		GPIO_InitTypeDef Biu_initstruct;
			
		__HAL_RCC_TIM2_CLK_ENABLE();		
		__HAL_RCC_GPIOA_CLK_ENABLE();		
	
		Biu_initstruct.Pin = GPIO_PIN_2|GPIO_PIN_3;//配置引脚号
		Biu_initstruct.Mode = GPIO_MODE_AF_PP;//配置工作模式 复用推挽输出
		Biu_initstruct.Pull = GPIO_PULLUP;//配置上拉下拉
		Biu_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;//配置输出速度
		
		HAL_GPIO_Init(GPIOA,&Biu_initstruct);
		}
		if(htim->Instance == TIM3){
			
		GPIO_InitTypeDef Biu_initstruct;
			
		__HAL_RCC_TIM3_CLK_ENABLE();		
		__HAL_RCC_GPIOB_CLK_ENABLE();		
	
		Biu_initstruct.Pin = GPIO_PIN_0|GPIO_PIN_1;//配置引脚号
		Biu_initstruct.Mode = GPIO_MODE_AF_PP;//配置工作模式 复用推挽输出
		Biu_initstruct.Pull = GPIO_PULLUP;//配置上拉下拉
		Biu_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;//配置输出速度
	
		HAL_GPIO_Init(GPIOB,&Biu_initstruct);	
	
		}

	if(htim->Instance == TIM4){
		GPIO_InitTypeDef gpio_initstruct;//定义结构体参数
			
		__HAL_RCC_TIM4_CLK_ENABLE();		//定时器4时钟使能	
		__HAL_RCC_GPIOB_CLK_ENABLE();		//开启GPIOB组引脚
	
		gpio_initstruct.Pin = GPIO_PIN_7;//配置引脚号
		gpio_initstruct.Mode = GPIO_MODE_AF_PP;//配置工作模式 复用推挽输出
		gpio_initstruct.Pull = GPIO_PULLUP;//配置上拉下拉
		gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;//配置输出速度
	
		HAL_GPIO_Init(GPIOB,&gpio_initstruct);		
	}

}

/*************************软件相关**************************/

void GO_forward(void)
{
		if(feeder_direction != FEEDER_DIRECTION_FORWARD)
		{
				pwm1_compare_set(0);
				HAL_Delay(FEEDER_REVERSE_DELAY_MS);
				HAL_GPIO_WritePin(GPIO2_PORT,GPIO_P1,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIO2_PORT,GPIO_P2,GPIO_PIN_SET);
				feeder_direction = FEEDER_DIRECTION_FORWARD;
		}

}
void GO_back(void)
{
		if(feeder_direction != FEEDER_DIRECTION_BACK)
		{
				pwm1_compare_set(0);
				HAL_Delay(FEEDER_REVERSE_DELAY_MS);
				HAL_GPIO_WritePin(GPIO2_PORT,GPIO_P1,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIO2_PORT,GPIO_P2,GPIO_PIN_RESET);
				feeder_direction = FEEDER_DIRECTION_BACK;
		}
}

void pwm1_compare_set(uint16_t val)
{
		__HAL_TIM_SET_COMPARE(&pwm_handle,TIM_CHANNEL_1,val);
}



