#include "pid.h"
/*
 * Scalar PID, position and incremental forms. Derived from the DJI RoboMaster
 * reference implementation, unchanged in substance.
 *
 * No sample period anywhere. There is no dt term in either branch: the
 * integrator adds Ki*error once per call and the derivative is a bare
 * difference of successive errors. Consequently
 *
 *   - Ki carries units of [output]/[error] PER CALL, not per second, and
 *   - Kd carries units of [output]/[error] PER CALL as well,
 *
 * so every gain in this project is implicitly tied to the rate at which its
 * loop is invoked. Both chassis loops are driven from data_get_task at a fixed
 * 1 ms (my_task.c, vTaskDelayUntil(1 ms)), which is what makes the omission
 * safe. Change that period and Ki/Kd change meaning by the same factor -- a
 * loop moved from 1 ms to 2 ms integrates half as fast at identical gains.
 * Anything that could call PID_calc() irregularly must not use this file.
 */


/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
void PID_init(pid_type_def *pid, uint8_t mode, const float PID[3], float max_out, float max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
float PID_calc(pid_type_def *pid, float ref, float set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    /*
     * error = set - ref. See the ARGUMENT ORDER note in pid.h: `ref` is the
     * feedback and `set` is the target, which is the reverse of how the two
     * names read. The history shifts above are what let the delta branch below
     * form a second difference; the position branch only needs error[0..1].
     */
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        /*
         * Derivative on error, as an unfiltered first difference -- no division
         * by dt and no low-pass. Two consequences worth knowing before touching
         * Kd:
         *
         *   - One LSB of sensor noise appearing on a single sample produces a
         *     full Kd-sized kick in the output, because the difference of two
         *     adjacent samples is exactly that noise. At the 1 ms loop rate this
         *     is a 1 kHz impulse train straight onto the actuator. The FOC
         *     boards solve the same problem with an explicit D_lpf_Tf
         *     (08.yaw_4310, Pid.c); this implementation has no equivalent, which
         *     is why the chassis loops keep Kd small (0.01) or zero.
         *   - Dbuf[2]/Dbuf[1] are maintained but never read in this branch. They
         *     exist for the delta form below and as debugger history.
         *
         * Derivative-on-error also means a step change in the TARGET produces a
         * derivative kick (the classic problem that derivative-on-measurement
         * avoids). Relevant here because the targets are stick-driven.
         */
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        /*
         * Two clamps, in this order, and the order is the anti-windup:
         *
         *   1. Iout is bounded by max_iout BEFORE it is summed. Because Iout is
         *      the only term that carries state across calls, bounding it here
         *      is what stops a sustained error from accumulating an integral so
         *      large that the loop keeps commanding full output long after the
         *      error has reversed. Clamping only the total (step 2) would not
         *      do this -- the total would look bounded while Iout kept growing
         *      underneath it.
         *   2. out is then bounded by max_out, which is the actuator limit.
         *
         * This is classic clamping anti-windup, not back-calculation: the
         * integrator is limited unconditionally rather than only while the
         * output is saturated, so it also caps authority during normal
         * operation. Both chassis loops set max_out == max_iout == 300
         * (data.c, data_init), which means the integrator alone may command the
         * full output range.
         *
         * Note Ki is 0.0 in both chassis loops today, so Iout is dead code
         * there -- but yaw_gimble_pos[] keeps a non-zero Kd, so the derivative
         * path below is live.
         */
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    /*
     * Incremental form. Unused by the chassis (both loops are PID_POSITION),
     * documented because the asymmetry with the branch above is a trap:
     *
     *   - pid->out ACCUMULATES (`out +=`), so out is the integrator here. There
     *     is no separate Iout state, and max_iout is therefore never applied in
     *     this branch at all -- the only bound is max_out on the total.
     *   - Pout uses the difference of successive errors, and Dout uses the
     *     second difference, which is what makes the accumulated result
     *     equivalent to the position form. Iout uses the plain error.
     *   - Because the accumulator is the output itself, PID_clear() is the only
     *     way to reset the integral state, and switching a live pid_type_def
     *     between the two modes carries the old accumulator over.
     */
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
/*
 * Clears all runtime state: error history, derivative history, every output
 * term, and the stored set/fdb.
 *
 * Deliberately does NOT touch mode, Kp/Ki/Kd, max_out or max_iout, so a loop
 * can be reset mid-flight without being reconfigured. Call this whenever a loop
 * is re-enabled after being disabled -- otherwise Iout (position form) or out
 * (delta form) resumes from a value accumulated against a stale error, which on
 * the yaw follow loop means the chassis lurches the instant the loop comes back.
 *
 * Nothing in the chassis firmware calls this today; the loops run continuously
 * from boot. It becomes necessary the moment a mode switch gates one of them.
 */
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

