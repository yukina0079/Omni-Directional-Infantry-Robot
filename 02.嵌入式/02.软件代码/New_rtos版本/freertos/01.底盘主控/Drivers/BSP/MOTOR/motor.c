#include "motor.h"
#include "stdlib.h"

#define MOTOR_GPIO1_PORT GPIOE
#define MOTOR_GPIO2_PORT GPIOB

#define MOTOR1_GPIO_P1 GPIO_PIN_8
#define MOTOR1_GPIO_P2 GPIO_PIN_9

#define MOTOR2_GPIO_P1 GPIO_PIN_0
#define MOTOR2_GPIO_P2 GPIO_PIN_1

#define MOTOR3_GPIO_P1 GPIO_PIN_12
#define MOTOR3_GPIO_P2 GPIO_PIN_13

#define MOTOR4_GPIO_P1 GPIO_PIN_14
#define MOTOR4_GPIO_P2 GPIO_PIN_15

TIM_HandleTypeDef pwm_handle = {0};

void motor_init(void)
{
		GPIO_InitTypeDef gpio_initstruct;

		__HAL_RCC_GPIOE_CLK_ENABLE();
		gpio_initstruct.Pin = MOTOR3_GPIO_P1|MOTOR3_GPIO_P2|MOTOR2_GPIO_P1|MOTOR2_GPIO_P2|MOTOR4_GPIO_P1|MOTOR4_GPIO_P2;
		gpio_initstruct.Mode = GPIO_MODE_OUTPUT_PP;
		gpio_initstruct.Pull = GPIO_PULLUP;
		gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(MOTOR_GPIO1_PORT,&gpio_initstruct);

		__HAL_RCC_GPIOB_CLK_ENABLE();
		gpio_initstruct.Pin = MOTOR1_GPIO_P1|MOTOR1_GPIO_P2;
		gpio_initstruct.Mode = GPIO_MODE_OUTPUT_PP;
		gpio_initstruct.Pull = GPIO_PULLUP;
		gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(MOTOR_GPIO2_PORT,&gpio_initstruct);
}

void pwm_init(uint16_t arr,uint16_t psc)
{
		TIM_OC_InitTypeDef pwm_config = {0};

		pwm_handle.Instance = TIM8;
		pwm_handle.Init.Prescaler = psc;
		pwm_handle.Init.Period = arr;
		pwm_handle.Init.CounterMode = TIM_COUNTERMODE_UP;

		HAL_TIM_PWM_Init(&pwm_handle);

		pwm_config.OCMode = TIM_OCMODE_PWM1;
		pwm_config.Pulse  = 0;
		pwm_config.OCPolarity = TIM_OCPOLARITY_HIGH;

		HAL_TIM_PWM_ConfigChannel(&pwm_handle,&pwm_config,TIM_CHANNEL_1);
		HAL_TIM_PWM_ConfigChannel(&pwm_handle,&pwm_config,TIM_CHANNEL_2);
		HAL_TIM_PWM_ConfigChannel(&pwm_handle,&pwm_config,TIM_CHANNEL_3);
		HAL_TIM_PWM_ConfigChannel(&pwm_handle,&pwm_config,TIM_CHANNEL_4);

		HAL_TIM_PWM_Start(&pwm_handle,TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&pwm_handle,TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&pwm_handle,TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&pwm_handle,TIM_CHANNEL_4);

}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM8){
        GPIO_InitTypeDef gpio_initstruct;

        __HAL_RCC_TIM8_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        gpio_initstruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
        gpio_initstruct.Mode = GPIO_MODE_AF_PP;
        gpio_initstruct.Pull = GPIO_PULLUP;
        gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_initstruct.Alternate = GPIO_AF3_TIM8;

        HAL_GPIO_Init(GPIOC, &gpio_initstruct);
    }
}

void motor1(int i)
{
	if(i == 1){
		HAL_GPIO_WritePin(MOTOR_GPIO2_PORT,MOTOR1_GPIO_P1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_GPIO2_PORT,MOTOR1_GPIO_P2,GPIO_PIN_SET);
	}
	if(i == 0){
		HAL_GPIO_WritePin(MOTOR_GPIO2_PORT,MOTOR1_GPIO_P1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_GPIO2_PORT,MOTOR1_GPIO_P2,GPIO_PIN_RESET);
	}
}
void motor2(int i)
{
	if(i == 1){
		HAL_GPIO_WritePin(MOTOR_GPIO1_PORT,MOTOR2_GPIO_P1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_GPIO1_PORT,MOTOR2_GPIO_P2,GPIO_PIN_SET);
	}
	if(i == 0){
		HAL_GPIO_WritePin(MOTOR_GPIO1_PORT,MOTOR2_GPIO_P1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_GPIO1_PORT,MOTOR2_GPIO_P2,GPIO_PIN_RESET);
	}
}
void motor3(int i)
{
	if(i == 1){
		HAL_GPIO_WritePin(MOTOR_GPIO1_PORT,MOTOR3_GPIO_P1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_GPIO1_PORT,MOTOR3_GPIO_P2,GPIO_PIN_SET);
	}
	if(i == 0){
		HAL_GPIO_WritePin(MOTOR_GPIO1_PORT,MOTOR3_GPIO_P1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_GPIO1_PORT,MOTOR3_GPIO_P2,GPIO_PIN_RESET);
	}
}
void motor4(int i)
{
	if(i == 1){
		HAL_GPIO_WritePin(MOTOR_GPIO1_PORT,MOTOR4_GPIO_P1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_GPIO1_PORT,MOTOR4_GPIO_P2,GPIO_PIN_SET);
	}
	if(i == 0){
		HAL_GPIO_WritePin(MOTOR_GPIO1_PORT,MOTOR4_GPIO_P1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_GPIO1_PORT,MOTOR4_GPIO_P2,GPIO_PIN_RESET);
	}
}

void motor1_speed(uint16_t val)
{
			__HAL_TIM_SET_COMPARE(&pwm_handle,TIM_CHANNEL_4,val);
}
void motor2_speed(uint16_t val)
{
			__HAL_TIM_SET_COMPARE(&pwm_handle,TIM_CHANNEL_3,val);
}
void motor3_speed(uint16_t val)
{
			__HAL_TIM_SET_COMPARE(&pwm_handle,TIM_CHANNEL_2,val);
}
void motor4_speed(uint16_t val)
{
			__HAL_TIM_SET_COMPARE(&pwm_handle,TIM_CHANNEL_1,val);
}

void motor1_move(int val)
{
		if(val>0){motor1(1);}
		if(val<0){motor1(0);}
		motor1_speed(abs(val));
}
void motor2_move(int val)
{
		if(val>0){motor2(1);}
		if(val<0){motor2(0);}
		motor2_speed(abs(val));
}
void motor3_move(int val)
{
		if(val>0){motor3(1);}
		if(val<0){motor3(0);}
		motor3_speed(abs(val));
}
void motor4_move(int val)
{
		if(val>0){motor4(1);}
		if(val<0){motor4(0);}
		motor4_speed(abs(val));
}

void MOTOR_PWM_UPDATE(int val1,int val2,int val3,int val4)
{
	motor1_move(val1);

	motor2_move(val2);

	motor3_move(val3);

	motor4_move(val4);
}
