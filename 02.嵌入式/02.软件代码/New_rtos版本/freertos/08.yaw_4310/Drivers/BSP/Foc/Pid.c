#include "Pid.h"
#include "systime.h"

/*
 * Clears the accumulated state so the next PID_operator() call starts fresh.
 * Call this whenever the loop has been disengaged for a while (motor stopped,
 * comms lost); otherwise a stale integral and a stale error_prev produce a
 * violent kick on re-engagement.
 */
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
    /*
     * Ts now comes from the free-running TIM2 microsecond counter. The old
     * code did (SysTick->VAL - prev) * 1e-6f, which was wrong three ways:
     * SysTick counts DOWN (so the unsigned subtraction underflowed), the
     * result was never divided by the timer frequency, and SysTick had been
     * disabled by delay_us() anyway. Every call fell through to the 1e-3
     * fallback, which is why the I and D terms never behaved as expected.
     */
    float Ts = systime_delta_s(&pid->timestamp_prev);

    // P环
    float proportional = pid->P * error;

    // I环 (trapezoidal integration, clamped to prevent windup)
    float integral = pid->integral_prev + pid->I * Ts * 0.5f * (error + pid->error_prev);
    integral = _constrain(integral, -pid->limit, pid->limit);

    // D环
    /*
     * Filtered, because the raw derivative divides a quantised error by Ts and
     * so amplifies one encoder LSB into a large spurious rate -- see D_lpf_Tf
     * in Pid.h. The filter acts on the D term alone, so the P path, which
     * carries the real position information, keeps full bandwidth.
     */
    float derivative = pid->D * (error - pid->error_prev) / Ts;
    if (pid->D_lpf_Tf > 0.0f) {
        float alpha = pid->D_lpf_Tf / (pid->D_lpf_Tf + Ts);
        derivative = alpha * pid->derivative_prev + (1.0f - alpha) * derivative;
    }

    // 将P,I,D三环的计算值加起来
    float output = proportional + integral + derivative;
    output = _constrain(output, -pid->limit, pid->limit);

    /*
     * Slew-rate limit. output_ramp is expressed in units/second, so the most
     * the output may move this tick is output_ramp * Ts.
     */
    if (pid->output_ramp > 0.0f) {
        float delta = (output - pid->output_prev) / Ts;   /* units per second */
        if (delta > pid->output_ramp) {
            output = pid->output_prev + pid->output_ramp * Ts;
        } else if (delta < -pid->output_ramp) {
            output = pid->output_prev - pid->output_ramp * Ts;
        }
    }

    // 保存值（为了下一次循环）
    pid->integral_prev   = integral;
    pid->output_prev     = output;
    pid->error_prev      = error;
    pid->derivative_prev = derivative;
    return output;
}
