#ifndef __SYSTIME_H__
#define __SYSTIME_H__

#include "sys.h"

/*
 * Free-running microsecond time base built on TIM2.
 *
 * Why a dedicated timer instead of SysTick:
 * The previous implementation derived every control-loop timestep from
 * SysTick->VAL. That was broken in two independent ways:
 *   1. delay_us() reprogrammed SysTick->LOAD and then DISABLED the counter on
 *      exit, so SysTick->VAL froze permanently after the first delay call.
 *   2. Even when running, SysTick is a 1 ms down-counter driven by HCLK
 *      (not HCLK/8), so it can neither measure intervals longer than 1 ms nor
 *      be converted with the /9 factor the old code used.
 * Every PID / low-pass / velocity Ts therefore collapsed onto its 1e-3 fallback.
 *
 * TIM2 is a 16-bit general-purpose timer on APB1. With APB1 at 36 MHz its
 * timer clock is doubled to 72 MHz (RCC rule: when the APB prescaler is not 1,
 * the timer clock is 2x PCLK), so PSC = 71 yields exactly 1 MHz -> 1 us/tick.
 * ARR = 0xFFFF makes it wrap every 65.536 ms; the update interrupt extends the
 * count into a full 32-bit microsecond counter good for ~71 minutes.
 */

void systime_init(void);

/* Microseconds since systime_init(). Safe to call from any context. */
uint32_t micros(void);

/* Milliseconds since systime_init(). */
uint32_t millis(void);

/*
 * Elapsed microseconds from a previous micros() stamp to now.
 * Handles 32-bit wraparound correctly via unsigned arithmetic.
 */
uint32_t micros_since(uint32_t prev_us);

/*
 * Elapsed seconds since *prev_us, then updates *prev_us to now.
 * This is the helper every control loop should use to obtain its Ts:
 * it clamps implausible values (first call, long stalls) to a sane default
 * instead of letting a garbage Ts reach the integrator or differentiator.
 */
float systime_delta_s(uint32_t *prev_us);

#endif
