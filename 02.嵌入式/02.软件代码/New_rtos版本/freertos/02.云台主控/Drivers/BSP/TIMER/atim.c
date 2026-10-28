#include "./BSP/TIMER/atim.h"
#include "./BSP/LED/led.h"

TIM_HandleTypeDef g_atimx_pwm_chy_handle;
TIM_OC_InitTypeDef g_atimx_oc_pwm_chy_handle;

TIM_HandleTypeDef g_atimx_comp_pwm_handle;

TIM_HandleTypeDef g_atimx_cplm_pwm_handle;
TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

uint8_t g_atimxchy_pwmin_sta  = 0;
uint16_t g_atimxchy_pwmin_psc  = 0;
uint32_t g_atimxchy_pwmin_hval = 0;
uint32_t g_atimxchy_pwmin_cval = 0;
static TIM_HandleTypeDef g_timx_pwmin_chy_handle;

void atim_timx_comp_pwm_init(uint16_t arr, uint16_t psc)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    g_atimx_comp_pwm_handle.Instance = ATIM_TIMX_COMP;
    g_atimx_comp_pwm_handle.Init.Prescaler = psc  ;
    g_atimx_comp_pwm_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_atimx_comp_pwm_handle.Init.Period = arr;
    g_atimx_comp_pwm_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    g_atimx_comp_pwm_handle.Init.RepetitionCounter = 0;
    g_atimx_comp_pwm_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_OC_Init(&g_atimx_comp_pwm_handle);

    sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
    sConfigOC.Pulse = 250;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    HAL_TIM_OC_ConfigChannel(&g_atimx_comp_pwm_handle, &sConfigOC,  TIM_CHANNEL_1);
    __HAL_TIM_ENABLE_OCxPRELOAD(&g_atimx_comp_pwm_handle, TIM_CHANNEL_1);

    sConfigOC.Pulse = 500;
    HAL_TIM_OC_ConfigChannel(&g_atimx_comp_pwm_handle, &sConfigOC, TIM_CHANNEL_2);
    __HAL_TIM_ENABLE_OCxPRELOAD(&g_atimx_comp_pwm_handle, TIM_CHANNEL_2);

    sConfigOC.Pulse = 750;
    HAL_TIM_OC_ConfigChannel(&g_atimx_comp_pwm_handle, &sConfigOC,  TIM_CHANNEL_3);
    __HAL_TIM_ENABLE_OCxPRELOAD(&g_atimx_comp_pwm_handle, TIM_CHANNEL_3);

    sConfigOC.Pulse = 1000;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    HAL_TIM_OC_ConfigChannel(&g_atimx_comp_pwm_handle, &sConfigOC,  TIM_CHANNEL_4);
    __HAL_TIM_ENABLE_OCxPRELOAD(&g_atimx_comp_pwm_handle, TIM_CHANNEL_4);

    HAL_TIM_OC_Start(&g_atimx_comp_pwm_handle,TIM_CHANNEL_1);
    HAL_TIM_OC_Start(&g_atimx_comp_pwm_handle,TIM_CHANNEL_2);
    HAL_TIM_OC_Start(&g_atimx_comp_pwm_handle,TIM_CHANNEL_3);
    HAL_TIM_OC_Start(&g_atimx_comp_pwm_handle,TIM_CHANNEL_4);
}

void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == ATIM_TIMX_COMP)
    {
        ATIM_TIMX_COMP_CLK_ENABLE();
        GPIO_InitTypeDef gpio_init_struct;
        ATIM_TIMX_COMP_CH1_GPIO_CLK_ENABLE();
        ATIM_TIMX_COMP_CH2_GPIO_CLK_ENABLE();
        ATIM_TIMX_COMP_CH3_GPIO_CLK_ENABLE();
        ATIM_TIMX_COMP_CH4_GPIO_CLK_ENABLE();

        gpio_init_struct.Pin = ATIM_TIMX_COMP_CH1_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct.Alternate = ATIM_TIMX_COMP_GPIO_AF;
        HAL_GPIO_Init(ATIM_TIMX_COMP_CH1_GPIO_PORT, &gpio_init_struct);

        gpio_init_struct.Pin = ATIM_TIMX_COMP_CH2_GPIO_PIN;
        HAL_GPIO_Init(ATIM_TIMX_COMP_CH2_GPIO_PORT, &gpio_init_struct);

        gpio_init_struct.Pin = ATIM_TIMX_COMP_CH3_GPIO_PIN;
        HAL_GPIO_Init(ATIM_TIMX_COMP_CH3_GPIO_PORT, &gpio_init_struct);

        gpio_init_struct.Pin = ATIM_TIMX_COMP_CH4_GPIO_PIN;
        HAL_GPIO_Init(ATIM_TIMX_COMP_CH4_GPIO_PORT, &gpio_init_struct);
    }
}

void atim_timx_cplm_pwm_init(uint16_t arr, uint16_t psc)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    GPIO_InitTypeDef gpio_init_struct = {0};

    ATIM_TIMX_CPLM_CHY_GPIO_CLK_ENABLE();
    ATIM_TIMX_CPLM_CHYN_GPIO_CLK_ENABLE();
    ATIM_TIMX_CPLM_BKIN_GPIO_CLK_ENABLE();

    gpio_init_struct.Pin = ATIM_TIMX_CPLM_BKIN_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH ;
    gpio_init_struct.Alternate = ATIM_TIMX_CPLM_CHY_GPIO_AF;
    HAL_GPIO_Init(ATIM_TIMX_CPLM_BKIN_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = ATIM_TIMX_CPLM_CHY_GPIO_PIN;
    HAL_GPIO_Init(ATIM_TIMX_CPLM_CHY_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = ATIM_TIMX_CPLM_CHYN_GPIO_PIN;
    HAL_GPIO_Init(ATIM_TIMX_CPLM_CHYN_GPIO_PORT, &gpio_init_struct);

    ATIM_TIMX_CPLM_CLK_ENABLE();

    g_atimx_cplm_pwm_handle.Instance = ATIM_TIMX_CPLM;
    g_atimx_cplm_pwm_handle.Init.Prescaler = psc;
    g_atimx_cplm_pwm_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_atimx_cplm_pwm_handle.Init.Period = arr;
    g_atimx_cplm_pwm_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    g_atimx_cplm_pwm_handle.Init.RepetitionCounter = 0;
    g_atimx_cplm_pwm_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&g_atimx_cplm_pwm_handle) ;

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
    HAL_TIM_PWM_ConfigChannel(&g_atimx_cplm_pwm_handle, &sConfigOC, ATIM_TIMX_CPLM_CHY);

    sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&g_atimx_cplm_pwm_handle, &sBreakDeadTimeConfig);

    HAL_TIM_PWM_Start(&g_atimx_cplm_pwm_handle, ATIM_TIMX_CPLM_CHY);
    HAL_TIMEx_PWMN_Start(&g_atimx_cplm_pwm_handle, ATIM_TIMX_CPLM_CHY);

}

void atim_timx_cplm_pwm_set(uint16_t ccr, uint8_t dtg)
{
    sBreakDeadTimeConfig.DeadTime = dtg;
    HAL_TIMEx_ConfigBreakDeadTime(&g_atimx_cplm_pwm_handle, &sBreakDeadTimeConfig);
    __HAL_TIM_MOE_ENABLE(&g_atimx_cplm_pwm_handle);
    ATIM_TIMX_CPLM_CHY_CCRY = ccr;

}

void atim_timx_pwmin_chy_init(void)
{
    ATIM_TIMX_PWMIN_CHY_GPIO_CLK_ENABLE();

    GPIO_InitTypeDef gpio_init_struct = {0};

    gpio_init_struct.Pin = ATIM_TIMX_PWMIN_CHY_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate = ATIM_TIMX_PWMIN_CHY_GPIO_AF;

    HAL_GPIO_Init(ATIM_TIMX_PWMIN_CHY_GPIO_PORT, &gpio_init_struct);

    TIM_SlaveConfigTypeDef sSlaveConfig = {0};
    TIM_IC_InitTypeDef sConfigIC = {0};

    ATIM_TIMX_PWMIN_CHY_CLK_ENABLE();

    g_timx_pwmin_chy_handle.Instance = ATIM_TIMX_PWMIN;
    g_timx_pwmin_chy_handle.Init.Prescaler = 0;
    g_timx_pwmin_chy_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_timx_pwmin_chy_handle.Init.Period = 65535;
    g_timx_pwmin_chy_handle.Init.ClockDivision =TIM_CLOCKDIVISION_DIV1;
    g_timx_pwmin_chy_handle.Init.RepetitionCounter = 0;
    g_timx_pwmin_chy_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_IC_Init(&g_timx_pwmin_chy_handle);

    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
    sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
    sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sSlaveConfig.TriggerFilter = 0;
    HAL_TIM_SlaveConfigSynchro(&g_timx_pwmin_chy_handle, &sSlaveConfig);

    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    HAL_TIM_IC_ConfigChannel( &g_timx_pwmin_chy_handle, &sConfigIC, TIM_CHANNEL_1 );

    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
    sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
    HAL_TIM_IC_ConfigChannel(&g_timx_pwmin_chy_handle,&sConfigIC,TIM_CHANNEL_2);

    HAL_NVIC_SetPriority(ATIM_TIMX_PWMIN_IRQn, 1, 3);
    HAL_NVIC_EnableIRQ( ATIM_TIMX_PWMIN_IRQn );

    if ( ATIM_TIMX_PWMIN == TIM1 || ATIM_TIMX_PWMIN == TIM8)
    {
        HAL_NVIC_SetPriority(ATIM_TIMX_PWMIN_CC_IRQn, 1, 3);
        HAL_NVIC_EnableIRQ(ATIM_TIMX_PWMIN_CC_IRQn);
    }

    HAL_TIM_IC_Start_IT(&g_timx_pwmin_chy_handle, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&g_timx_pwmin_chy_handle, TIM_CHANNEL_2);
}

void ATIM_TIMX_PWMIN_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_timx_pwmin_chy_handle);
}

void ATIM_TIMX_PWMIN_CC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_timx_pwmin_chy_handle);
}

void atim_timx_pwmin_chy_restart(void)
{
    sys_intx_disable();

    g_atimxchy_pwmin_sta = 0;
    g_atimxchy_pwmin_hval=0;
    g_atimxchy_pwmin_cval=0;

    sys_intx_enable();
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

    if (g_atimxchy_pwmin_sta == 0)
    {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        {
            g_atimxchy_pwmin_hval = HAL_TIM_ReadCapturedValue(&g_timx_pwmin_chy_handle,TIM_CHANNEL_2)+2;
            g_atimxchy_pwmin_cval = HAL_TIM_ReadCapturedValue(&g_timx_pwmin_chy_handle,TIM_CHANNEL_1)+2;
            g_atimxchy_pwmin_sta = 1;
        }
    }
}
