#include "./BSP/LED/led.h"
#include "./BSP/TIMER/btim.h"

TIM_HandleTypeDef timx_handler;

void BTIM_TIMX_INT_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&timx_handler);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&timx_handler))
    {
        LED1_TOGGLE();
    }
}

void btim_timx_int_init(uint16_t arr, uint16_t psc)
{
    timx_handler.Instance = BTIM_TIMX_INT;
    timx_handler.Init.Prescaler = psc;
    timx_handler.Init.CounterMode = TIM_COUNTERMODE_UP;
    timx_handler.Init.Period = arr;
    timx_handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&timx_handler);

    HAL_TIM_Base_Start_IT(&timx_handler);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == BTIM_TIMX_INT)
    {
        BTIM_TIMX_INT_CLK_ENABLE();
        HAL_NVIC_SetPriority(BTIM_TIMX_INT_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(BTIM_TIMX_INT_IRQn);
    }
}
