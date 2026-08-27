#ifndef  __PWM_H__
#define  __PWM_H__
#include "sys.h"
void pwm_init(uint16_t arr,uint16_t psc);
void pwm_compare_set(uint16_t val);

#endif
