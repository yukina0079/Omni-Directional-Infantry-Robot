#include "Pid.h"
#include "systime.h"

void PID_Reset(PID *pid)
{
    pid->error_prev      = 0.0f;
    pid->output_prev     = 0.0f;
    pid->integral_prev   = 0.0f;
    pid->derivative_prev = 0.0f;
    pid->timestamp_prev  = micros();
}

float PID_operator(PID *pid, float error)
{

    float Ts = systime_delta_s(&pid->timestamp_prev);

    float proportional = pid->P * error;

    float integral = pid->integral_prev + pid->I * Ts * 0.5f * (error + pid->error_prev);
    integral = _constrain(integral, -pid->limit, pid->limit);

    float derivative = pid->D * (error - pid->error_prev) / Ts;
    if (pid->D_lpf_Tf > 0.0f) {
        float alpha = pid->D_lpf_Tf / (pid->D_lpf_Tf + Ts);
        derivative = alpha * pid->derivative_prev + (1.0f - alpha) * derivative;
    }

    float output = proportional + integral + derivative;
    output = _constrain(output, -pid->limit, pid->limit);

    if (pid->output_ramp > 0.0f) {
        float delta = (output - pid->output_prev) / Ts;
        if (delta > pid->output_ramp) {
            output = pid->output_prev + pid->output_ramp * Ts;
        } else if (delta < -pid->output_ramp) {
            output = pid->output_prev - pid->output_ramp * Ts;
        }
    }

    pid->integral_prev   = integral;
    pid->output_prev     = output;
    pid->error_prev      = error;
    pid->derivative_prev = derivative;
    return output;
}
