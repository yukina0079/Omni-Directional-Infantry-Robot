#include "./BSP/TIMER/gtim.h"
#include "./BSP/LED/led.h"

TIM_HandleTypeDef g_timx_handle;

TIM_HandleTypeDef g_timx_pwm_chy_handle;
TIM_OC_InitTypeDef g_timx_oc_pwm_chy_handle;
uint8_t g_timxchy_cap_sta = 0;
uint16_t g_timxchy_cap_val = 0;

TIM_HandleTypeDef g_timx_cap_chy_handler;
TIM_IC_InitTypeDef g_timx_ic_cap_chy_handler;

TIM_HandleTypeDef g_timx_cnt_chy_handler;

uint32_t g_timxchy_cnt_ofcnt = 0;

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == GTIM_TIMX_CAP)
    {
        GPIO_InitTypeDef gpio_init_struct;
        GTIM_TIMX_CAP_CHY_CLK_ENABLE();
        GTIM_TIMX_CAP_CHY_GPIO_CLK_ENABLE();

        gpio_init_struct.Pin = GTIM_TIMX_CAP_CHY_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_PULLDOWN;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct.Alternate = GTIM_TIMX_CAP_CHY_GPIO_AF;
        HAL_GPIO_Init(GTIM_TIMX_CAP_CHY_GPIO_PORT, &gpio_init_struct);

        HAL_NVIC_SetPriority(GTIM_TIMX_CAP_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(GTIM_TIMX_CAP_IRQn);
    }
    else if (htim->Instance == GTIM_TIMX_CNT)
    {
        GPIO_InitTypeDef gpio_init_struct;
        TIM_SlaveConfigTypeDef tim_slave_config_handle = {0};
        TIM_IC_InitTypeDef timx_ic_cnt_chy_handler = {0};
        GTIM_TIMX_CNT_CHY_CLK_ENABLE();
        GTIM_TIMX_CNT_CHY_GPIO_CLK_ENABLE();

        gpio_init_struct.Pin = GTIM_TIMX_CNT_CHY_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_PULLDOWN;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GTIM_TIMX_CNT_CHY_GPIO_PORT, &gpio_init_struct);

        tim_slave_config_handle.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
        tim_slave_config_handle.InputTrigger = TIM_TS_TI1FP1;
        tim_slave_config_handle.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
        tim_slave_config_handle.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
        tim_slave_config_handle.TriggerFilter = 0;
        HAL_TIM_SlaveConfigSynchro(&g_timx_cnt_chy_handler, &tim_slave_config_handle);

        timx_ic_cnt_chy_handler.ICPolarity = TIM_ICPOLARITY_RISING;
        timx_ic_cnt_chy_handler.ICSelection = TIM_ICSELECTION_DIRECTTI;
        timx_ic_cnt_chy_handler.ICPrescaler = TIM_ICPSC_DIV1;
        timx_ic_cnt_chy_handler.ICFilter = 0;
        HAL_TIM_IC_ConfigChannel(&g_timx_cnt_chy_handler, &timx_ic_cnt_chy_handler, GTIM_TIMX_CNT_CHY);
        HAL_TIM_IC_Start(&g_timx_cnt_chy_handler, GTIM_TIMX_CNT_CHY);

        HAL_NVIC_SetPriority(GTIM_TIMX_CNT_IRQn, 1, 3);
        HAL_NVIC_EnableIRQ(GTIM_TIMX_CNT_IRQn);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&g_timx_handle))
    {
        LED1_TOGGLE();
    }

    if (htim == (&g_timx_cap_chy_handler))
    {
        if ((g_timxchy_cap_sta & 0X80) == 0)
        {
            if (g_timxchy_cap_sta & 0X40)
            {
                if ((g_timxchy_cap_sta & 0X3F) == 0X3F)
                {
                    g_timxchy_cap_sta |= 0X80;
                    g_timxchy_cap_val = 0XFFFF;
                }
                else
                {
                    g_timxchy_cap_sta++;
                }
            }
        }
    }

    if (htim == (&g_timx_cnt_chy_handler))
    {
        g_timxchy_cnt_ofcnt++;
    }
}

void gtim_timx_int_init(uint16_t arr, uint16_t psc)
{
    GTIM_TIMX_INT_CLK_ENABLE();

    g_timx_handle.Instance = GTIM_TIMX_INT;
    g_timx_handle.Init.Prescaler = psc;
    g_timx_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_timx_handle.Init.Period = arr;
    g_timx_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&g_timx_handle);

    HAL_NVIC_SetPriority(GTIM_TIMX_INT_IRQn, 1, 3);
    HAL_NVIC_EnableIRQ(GTIM_TIMX_INT_IRQn);

    HAL_TIM_Base_Start_IT(&g_timx_handle);
}

void GTIM_TIMX_INT_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_timx_handle);
}

void gtim_timx_pwm_chy_init(uint16_t arr, uint16_t psc)
{

    g_timx_pwm_chy_handle.Instance = GTIM_TIMX_PWM;
    g_timx_pwm_chy_handle.Init.Prescaler = psc;
    g_timx_pwm_chy_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_timx_pwm_chy_handle.Init.Period = arr;
    g_timx_pwm_chy_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&g_timx_pwm_chy_handle);

    g_timx_oc_pwm_chy_handle.OCMode = TIM_OCMODE_PWM1;
    g_timx_oc_pwm_chy_handle.Pulse = arr / 2;

    g_timx_oc_pwm_chy_handle.OCPolarity = TIM_OCPOLARITY_LOW;
    HAL_TIM_PWM_ConfigChannel(&g_timx_pwm_chy_handle, &g_timx_oc_pwm_chy_handle, GTIM_TIMX_PWM_CHY);
    HAL_TIM_PWM_Start(&g_timx_pwm_chy_handle, GTIM_TIMX_PWM_CHY);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == GTIM_TIMX_PWM)
    {
        GPIO_InitTypeDef gpio_init_struct;
        GTIM_TIMX_PWM_CHY_GPIO_CLK_ENABLE();
        GTIM_TIMX_PWM_CHY_CLK_ENABLE();

        gpio_init_struct.Pin = GTIM_TIMX_PWM_CHY_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_PULLUP;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct.Alternate = GTIM_TIMX_PWM_CHY_GPIO_AF;
        HAL_GPIO_Init(GTIM_TIMX_PWM_CHY_GPIO_PORT, &gpio_init_struct);
    }
}

void gtim_timx_pwm_setcompare(uint32_t compare)
{
    GTIM_TIMX_PWM_CHY_CCRX = compare;
}

uint32_t get_timx_capture(void)
{
    return HAL_TIM_ReadCapturedValue(&g_timx_pwm_chy_handle, GTIM_TIMX_PWM_CHY);
}

void gtim_timx_cap_chy_init(uint32_t arr, uint16_t psc)
{
    g_timx_cap_chy_handler.Instance = GTIM_TIMX_CAP;
    g_timx_cap_chy_handler.Init.Prescaler = psc;
    g_timx_cap_chy_handler.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_timx_cap_chy_handler.Init.Period = arr;
    g_timx_cap_chy_handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_IC_Init(&g_timx_cap_chy_handler);

    g_timx_ic_cap_chy_handler.ICPolarity = TIM_ICPOLARITY_RISING;
    g_timx_ic_cap_chy_handler.ICSelection = TIM_ICSELECTION_DIRECTTI;
    g_timx_ic_cap_chy_handler.ICPrescaler = TIM_ICPSC_DIV1;
    g_timx_ic_cap_chy_handler.ICFilter = 0;
    HAL_TIM_IC_ConfigChannel(&g_timx_cap_chy_handler, &g_timx_ic_cap_chy_handler, GTIM_TIMX_CAP_CHY);

    HAL_TIM_IC_Start_IT(&g_timx_cap_chy_handler, GTIM_TIMX_CAP_CHY);
    __HAL_TIM_ENABLE_IT(&g_timx_cap_chy_handler, TIM_IT_UPDATE);
}

void GTIM_TIMX_CAP_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_timx_cap_chy_handler);
}

void gtim_timx_cnt_chy_init(uint16_t psc)
{
    g_timx_cnt_chy_handler.Instance = GTIM_TIMX_CNT;
    g_timx_cnt_chy_handler.Init.Prescaler = psc;
    g_timx_cnt_chy_handler.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_timx_cnt_chy_handler.Init.Period = 65535;
    g_timx_cnt_chy_handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    g_timx_cnt_chy_handler.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_IC_Init(&g_timx_cnt_chy_handler);
}

uint32_t gtim_timx_cnt_chy_get_count(void)
{
    uint32_t count = 0;
    count = g_timxchy_cnt_ofcnt * 65536;
    count += __HAL_TIM_GET_COUNTER(&g_timx_cnt_chy_handler);
    return count;
}

void gtim_timx_cnt_chy_restart(void)
{
    __HAL_TIM_DISABLE(&g_timx_cnt_chy_handler);
    g_timxchy_cnt_ofcnt = 0;
    __HAL_TIM_SET_COUNTER(&g_timx_cnt_chy_handler, 0);
    __HAL_TIM_ENABLE(&g_timx_cnt_chy_handler);
}

void GTIM_TIMX_CNT_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_timx_cnt_chy_handler);
}
