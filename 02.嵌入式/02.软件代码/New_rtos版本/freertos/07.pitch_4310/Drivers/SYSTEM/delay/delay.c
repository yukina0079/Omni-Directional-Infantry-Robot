#include "delay.h"
#include "systime.h"

/* TIM2 is a free-running 1 MHz counter. It lives in this already-built
 * translation unit so the Keil project file does not need an extra source. */
static volatile uint32_t s_micros_high = 0;
static TIM_HandleTypeDef s_tim2_handle = {0};

void systime_init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    s_tim2_handle.Instance = TIM2;
    s_tim2_handle.Init.Prescaler = 71;     /* 72 MHz / 72 = 1 MHz */
    s_tim2_handle.Init.Period = 0xFFFF;
    s_tim2_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    s_tim2_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    s_tim2_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&s_tim2_handle);

    HAL_NVIC_SetPriority(TIM2_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    __HAL_TIM_CLEAR_FLAG(&s_tim2_handle, TIM_FLAG_UPDATE);
    HAL_TIM_Base_Start_IT(&s_tim2_handle);
}

void TIM2_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&s_tim2_handle, TIM_FLAG_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_IT(&s_tim2_handle, TIM_IT_UPDATE);
        s_micros_high += 0x10000u;
    }
}

uint32_t micros(void)
{
    uint32_t high;
    uint16_t count;

    do {
        high = s_micros_high;
        count = (uint16_t)TIM2->CNT;
    } while (high != s_micros_high);

    return high + count;
}

uint32_t millis(void)
{
    return micros() / 1000u;
}

uint32_t micros_since(uint32_t prev_us)
{
    return micros() - prev_us;
}

float systime_delta_s(uint32_t *prev_us)
{
    uint32_t now = micros();
    uint32_t dt = now - *prev_us;

    *prev_us = now;
    if (dt == 0u || dt > 100000u) {
        return 1e-3f;
    }
    return (float)dt * 1e-6f;
}

/*
 * Busy-wait delays built on the TIM2 microsecond time base.
 *
 * The previous implementation hijacked SysTick: it rewrote LOAD/VAL and, on
 * exit, cleared the ENABLE bit. That left SysTick stopped for the rest of the
 * program, which froze SysTick->VAL (breaking every control-loop timestep) and
 * froze uwTick (breaking HAL_GetTick and therefore every HAL timeout).
 *
 * SysTick now belongs entirely to HAL, and HAL_Delay is no longer overridden.
 */

void delay_us(uint32_t nus)
{
    uint32_t start = micros();

    while (micros_since(start) < nus) {
        /* busy wait */
    }
}

void delay_ms(uint32_t nms)
{
    /*
     * Delay in 1 ms steps rather than one big delay_us(nms * 1000): the
     * multiplication would overflow a uint32_t for nms > ~4295, and stepping
     * keeps each individual wait well inside the counter range.
     */
    while (nms--) {
        delay_us(1000);
    }
}

void delay_s(uint32_t ns)
{
    while (ns--) {
        delay_ms(1000);
    }
}
