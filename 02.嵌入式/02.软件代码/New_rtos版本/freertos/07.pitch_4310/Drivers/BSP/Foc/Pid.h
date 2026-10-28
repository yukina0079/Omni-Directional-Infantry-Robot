#ifndef PID_H
#define PID_H

#include "sys.h"

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

typedef struct{
    float P;
    float I;
    float D;
    float output;
    float limit;

    float output_ramp;

    float D_lpf_Tf;

    float error_prev;
    float derivative_prev;
    float output_prev;
    float integral_prev;
    uint32_t timestamp_prev;

} PID;

void  PID_Reset(PID *pid);
float PID_operator(PID *pid, float error);

#endif
