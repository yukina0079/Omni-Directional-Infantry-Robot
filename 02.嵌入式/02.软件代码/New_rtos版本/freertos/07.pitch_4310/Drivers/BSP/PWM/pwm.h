#ifndef  __PWM_H__
#define  __PWM_H__

#include "sys.h"

#define PWM_PERIOD   1440u

void pwm_init(uint16_t arr,uint16_t psc);

void motor1_pwm_start(void);

void motor1_pwm_set(uint16_t val1,uint16_t val2,uint16_t val3);

void motor1_pwm_stop(void);

#endif
