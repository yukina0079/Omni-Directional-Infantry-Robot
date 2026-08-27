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
    float error_prev;//最后跟踪的误差
    float output_prev;//最后的pid输出值
    float integral_prev;//最后的积分输出值
    unsigned long timestamp_prev;//最后的时间戳
}PID;

float PID_velocity(float error);
void PID_init(PID *pid);

#endif 
