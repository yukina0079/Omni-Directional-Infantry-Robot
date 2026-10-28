#include "timer.h"
#include "usart.h"
#include "imu.h"
#include "nrf24l01.h"
#include "data.h"

TIM_HandleTypeDef htim2;

void TIM2_Init(uint16_t arr, uint16_t psc)
{
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = psc;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = arr;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {

    }

    HAL_TIM_Base_Start_IT(&htim2);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {

        __HAL_RCC_TIM2_CLK_ENABLE();

        HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
    }
}

void TIM2_IRQHandler(void)
{

    HAL_TIM_IRQHandler(&htim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {

		imu_updata();

		pid_calculate();

    }
}
