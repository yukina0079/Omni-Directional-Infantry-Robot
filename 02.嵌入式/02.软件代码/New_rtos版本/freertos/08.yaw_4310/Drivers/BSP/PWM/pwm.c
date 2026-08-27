#include "pwm.h"

TIM_HandleTypeDef pwm1_handle = {0};

/*
 * Only TIM1 is initialised now.
 *
 * The original code also set up TIM4 on PB6/PB7/PB8 for a second motor, but
 * this board (new_foc, DRV8313) drives a single motor, and PB8 is wired to
 * CAN_RX. Configuring PB8 as a push-pull alternate-function output for TIM4
 * would fight the CAN peripheral the moment CAN is enabled. The second motor
 * was never used, so TIM4 is gone.
 */
void pwm_init(uint16_t arr, uint16_t psc)
{
	TIM_OC_InitTypeDef pwm1_config = {0};

	pwm1_handle.Instance = TIM1;
	pwm1_handle.Init.Prescaler = psc;
	pwm1_handle.Init.Period = arr;
	pwm1_handle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;

	HAL_TIM_PWM_Init(&pwm1_handle);

	pwm1_config.OCMode = TIM_OCMODE_PWM1;
	/*
	 * Start at 0% duty, not 50%. The DRV8313's EN pins are tied active on this
	 * board, so the gate driver is live from power-up; parking the compare
	 * registers mid-scale would put all three half-bridges at 50% for the
	 * window between pwm_init() and the first modulator update.
	 */
	pwm1_config.Pulse  = 0;
	pwm1_config.OCPolarity = TIM_OCPOLARITY_HIGH;

	HAL_TIM_PWM_ConfigChannel(&pwm1_handle,&pwm1_config,TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&pwm1_handle,&pwm1_config,TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel(&pwm1_handle,&pwm1_config,TIM_CHANNEL_3);

	/* Outputs stay off until motor1_pwm_start(). */
	motor1_pwm_stop();
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1){
		GPIO_InitTypeDef gpio_initstruct;

		__HAL_RCC_TIM1_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();

		gpio_initstruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
		gpio_initstruct.Mode = GPIO_MODE_AF_PP;
		gpio_initstruct.Pull = GPIO_NOPULL;
		gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;

		HAL_GPIO_Init(GPIOA,&gpio_initstruct);
	}
}

void motor1_pwm_start(void)
{
	HAL_TIM_PWM_Start(&pwm1_handle,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&pwm1_handle,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&pwm1_handle,TIM_CHANNEL_3);
}

/*
 * Hot path: three register writes, nothing else.
 *
 * The old version called HAL_TIM_PWM_Start() on all three channels here, on
 * every single modulator update. Each call re-checks driver state, does a
 * read-modify-write on CCER, re-enables MOE and re-enables the counter --
 * hundreds of wasted cycles per control cycle.
 */
void motor1_pwm_set(uint16_t val1, uint16_t val2, uint16_t val3)
{
	__HAL_TIM_SET_COMPARE(&pwm1_handle,TIM_CHANNEL_1,val1);
	__HAL_TIM_SET_COMPARE(&pwm1_handle,TIM_CHANNEL_2,val2);
	__HAL_TIM_SET_COMPARE(&pwm1_handle,TIM_CHANNEL_3,val3);
}

void motor1_pwm_stop(void)
{
	/* Clear all three compare values before disabling the outputs. */
	__HAL_TIM_SET_COMPARE(&pwm1_handle,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&pwm1_handle,TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&pwm1_handle,TIM_CHANNEL_3,0);

	HAL_TIM_PWM_Stop(&pwm1_handle,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&pwm1_handle,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&pwm1_handle,TIM_CHANNEL_3);
}
