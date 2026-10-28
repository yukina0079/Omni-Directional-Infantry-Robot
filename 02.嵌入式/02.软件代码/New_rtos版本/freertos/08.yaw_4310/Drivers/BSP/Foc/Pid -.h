#ifndef PID_H
#define PID_H

#include "stm32f10x.h"
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));

typedef struct{
    float P;
    float I;
    float D;
    float output;
    float limit;
    float output_ramp;
    float error_prev;
    float output_prev;
    float integral_prev;
    unsigned long timestamp_prev;
}PID;

float PID_velocity(float error);
void PID_init(PID *pid);

#endif
