#include "systime.h"

/*
 * High word of the 32-bit microsecond counter, advanced by 65536 on every
 * TIM2 update event. Declared volatile because it is written from the ISR and
 * read from thread context.
 */
static volatile uint32_t s_micros_high = 0;

static TIM_HandleTypeDef s_tim2_handle = {0};

void systime_init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    s_tim2_handle.Instance           = TIM2;
    s_tim2_handle.Init.Prescaler     = 71;      /* 72 MHz / (71+1) = 1 MHz */
    s_tim2_handle.Init.Period        = 0xFFFF;  /* wrap every 65.536 ms    */
    s_tim2_handle.Init.CounterMode   = TIM_COUNTERMODE_UP;
    s_tim2_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    s_tim2_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&s_tim2_handle);

    /*
     * The update interrupt only bumps a counter, so it can sit at the lowest
     * priority without hurting timing accuracy: micros() re-reads the high word
     * to detect a wrap that happened mid-read, so a late ISR is harmless as
     * long as it runs within one 65 ms wrap period.
     */
    HAL_NVIC_SetPriority(TIM2_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    __HAL_TIM_CLEAR_FLAG(&s_tim2_handle, TIM_FLAG_UPDATE);
    HAL_TIM_Base_Start_IT(&s_tim2_handle);
}

void TIM2_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&s_tim2_handle, TIM_FLAG_UPDATE) != RESET)
    {
        __HAL_TIM_CLEAR_IT(&s_tim2_handle, TIM_IT_UPDATE);
        s_micros_high += 0x10000u;
    }
}

uint32_t micros(void)
{
    uint32_t high;
    uint16_t cnt;

    /*
     * Read high word, then counter, then re-read the high word. If the timer
     * wrapped between the two reads the high word changed and we retry, so the
     * returned pair is always consistent.
     */
    do {
        high = s_micros_high;
        cnt  = (uint16_t)TIM2->CNT;
    } while (high != s_micros_high);

    return high + cnt;
}

uint32_t millis(void)
{
    return micros() / 1000u;
}

uint32_t micros_since(uint32_t prev_us)
{
    /* Unsigned subtraction wraps correctly, so no explicit overflow test. */
    return micros() - prev_us;
}

float systime_delta_s(uint32_t *prev_us)
{
    uint32_t now = micros();
    uint32_t dt  = now - *prev_us;

    *prev_us = now;

    /*
     * Guard against the very first call (prev_us == 0 -> dt is whatever the
     * timer happens to read) and against a stalled loop. 100 ms is far longer
     * than any healthy control period here, so anything above it means the
     * measurement is meaningless; fall back to a nominal 1 ms rather than
     * feeding a huge Ts into the integrator.
     */
    if (dt == 0u || dt > 100000u) {
        return 1e-3f;
    }

    return (float)dt * 1e-6f;
}
