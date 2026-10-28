#include "dma.h"
#include "stdio.h"

DMA_HandleTypeDef dam_handle = {0};
TIM_HandleTypeDef pwm_handle = {0};

void pwm_init(uint16_t arr,uint16_t psc)
{
		TIM_OC_InitTypeDef pwm_config = {0};

		pwm_handle.Instance = TIM2;
		pwm_handle.Init.Prescaler = psc;
		pwm_handle.Init.Period = arr;
		pwm_handle.Init.CounterMode = TIM_COUNTERMODE_UP;

		HAL_TIM_PWM_Init(&pwm_handle);

		pwm_config.OCMode = TIM_OCMODE_PWM1;
		pwm_config.Pulse  = 0;
		pwm_config.OCPolarity = TIM_OCPOLARITY_LOW;
		HAL_TIM_PWM_ConfigChannel(&pwm_handle,&pwm_config,TIM_CHANNEL_3);

		HAL_TIM_PWM_Start(&pwm_handle,TIM_CHANNEL_3);
}
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2){
		GPIO_InitTypeDef gpio_initstruct;

		__HAL_RCC_TIM2_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();

		gpio_initstruct.Pin = GPIO_PIN_3;
		gpio_initstruct.Mode = GPIO_MODE_AF_PP;
		gpio_initstruct.Pull = GPIO_PULLUP;
		gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;

		HAL_GPIO_Init(GPIOA,&gpio_initstruct);
	}
}
void dma_init(void)
{
		__HAL_RCC_DMA1_CLK_ENABLE();

		dam_handle.Instance = DMA1_Channel1;
		dam_handle.Init.Direction = DMA_MEMORY_TO_PERIPH;
		dam_handle.Init.PeriphInc = DMA_PINC_DISABLE;
		dam_handle.Init.MemInc = DMA_MINC_ENABLE;
		dam_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
		dam_handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
		dam_handle.Init.Mode = DMA_NORMAL;
		dam_handle.Init.Priority = DMA_PRIORITY_LOW;

		HAL_DMA_Init(&dam_handle);

		__HAL_LINKDMA(&pwm_handle,hdma[TIM_DMA_ID_CC3],dam_handle);
}
