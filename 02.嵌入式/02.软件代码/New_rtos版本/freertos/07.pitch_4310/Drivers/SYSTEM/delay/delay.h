#ifndef __DELAY_H__
#define __DELAY_H__

#include "sys.h"

/*
 * Busy-wait delays backed by the TIM2 microsecond time base (systime.c).
 * systime_init() must be called before any of these.
 *
 * HAL_Delay is deliberately NOT overridden any more: SysTick belongs to HAL,
 * so the stock SysTick-based HAL_Delay works and HAL_GetTick stays valid.
 */

void delay_ms(uint32_t nms);
void delay_us(uint32_t nus);
void delay_s(uint32_t ns);

#endif
