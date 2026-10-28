#ifndef  __MOTOR_H__
#define  __MOTOR_H__
#include "sys.h"

void motor_init(void);
void pwm_init(uint16_t arr,uint16_t psc);

void EN_init(void);
void EN_close(void);
void EN_open(void);
void MOTOR_PWM_UPDATE(int val1,int val2,int val3,int val4);

#endif
