#include "delay.h"
#include "systime.h"

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
