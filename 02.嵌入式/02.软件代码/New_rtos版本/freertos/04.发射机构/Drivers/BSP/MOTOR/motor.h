#ifndef  __MOTOR_H__
#define  __MOTOR_H__
#include "sys.h"
void motor_init(void);
void pwm_init(uint16_t arr,uint16_t psc);
void pwm1_compare_set(uint16_t val);
void pwm2_compare_set(uint16_t val);
void GO_forward(void);
void GO_back(void);
void stop(void);
void EN_init(void);
void EN_close(void);
void EN_open(void);

#endif
