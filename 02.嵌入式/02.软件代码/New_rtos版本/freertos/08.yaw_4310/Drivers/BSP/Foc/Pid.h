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
    /*
     * Maximum rate of change of the output, in output-units per second.
     * Set to 0 to disable rate limiting. Without it the loop steps straight to
     * the saturation voltage after a large error, which is a hard mechanical
     * slam on a gimbal carrying a barrel.
     */
    float output_ramp;

    /*
     * Time constant for a first-order filter on the D term alone, in seconds.
     * Set to 0 to disable.
     *
     * The derivative divides a quantised error by Ts, which amplifies the
     * encoder's 1 LSB (2*PI/4096 = 1.53 mrad = 0.088 deg) by 1/Ts. At the
     * ~470 us loop period that is 187 deg/s of noise per LSB, so an unfiltered
     * D term chatters even when the shaft is still. Filtering only D leaves the
     * P path -- which carries the actual position information -- untouched.
     */
    float D_lpf_Tf;

    // 原有变量
    float error_prev;
    float derivative_prev;    /* filtered D term, carried between calls */
    float output_prev;
    float integral_prev;
    uint32_t timestamp_prev;   /* micros() stamp, see systime.h */

} PID;

void  PID_Reset(PID *pid);
float PID_operator(PID *pid, float error);

#endif
