#ifndef  __PWM_H__
#define  __PWM_H__

#include "sys.h"

/*
 * TIM1 ARR. Centre-aligned mode counts up then down, so the actual PWM
 * frequency is 72 MHz / (2 * 1440) = 25 kHz -- above audible range and a
 * sensible carrier for this motor.
 *
 * This used to be hardcoded as a bare 1440 in three places in FOC.c; the
 * modulator now derives its scaling from this single definition.
 */
#define PWM_PERIOD   1440u

void pwm_init(uint16_t arr,uint16_t psc);

/* Starts the three TIM1 channels. Call once, before the control loop. */
void motor1_pwm_start(void);

/* Updates the three compare registers. Assumes motor1_pwm_start() ran. */
void motor1_pwm_set(uint16_t val1,uint16_t val2,uint16_t val3);

/* Zeroes the duty cycles and disables the outputs. */
void motor1_pwm_stop(void);

#endif
