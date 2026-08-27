#ifndef PID_H
#define PID_H
#include "sys.h"

/*
 * Symmetric clamp: forces `input` into [-max, +max] in place.
 *
 * Three properties of this macro that call sites depend on, and that are easy
 * to break:
 *
 *   - It is a STATEMENT, not an expression, and it is a bare braced block with
 *     no do{...}while(0) wrapper. `if (c) LimitMax(a, b); else ...` therefore
 *     fails to compile: the `;` after the block becomes an empty statement and
 *     orphans the `else`. Every use in this project is an unconditional
 *     statement, which is why it has never bitten.
 *   - Neither parameter is parenthesised. `LimitMax(a + b, m)` expands to
 *     `a + b > m` and then assigns to `a + b`, which will not compile; and a
 *     `max` passed as `x - 1` expands the negative branch to `input < -x - 1`,
 *     a different bound than intended. Pass plain lvalues and plain values.
 *   - `input` is evaluated up to three times and `max` twice, so neither may
 *     have side effects (no `LimitMax(*p++, ...)`).
 *
 * The bound is deliberately symmetric; an asymmetric limit needs a different
 * macro, and there is none in this project.
 */
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }
	
/*
 * Which of the two textbook PID forms PID_calc() evaluates.
 *
 * PID_POSITION ("full position form"): out = Kp*e + Ki*sum(e) + Kd*de. The
 *   integrator lives in pid->Iout and is clamped on its own by max_iout, so
 *   integral windup is bounded independently of the total-output clamp. The
 *   result is an ABSOLUTE command -- handing it to an actuator that itself
 *   integrates would give a double integration.
 *
 * PID_DELTA ("incremental form"): the same expression is applied to the
 *   DIFFERENCE of successive errors and accumulated into pid->out, so the
 *   caller still receives an absolute value but built up by increments. Note
 *   the asymmetry in this implementation: in delta mode max_iout is never
 *   applied (see PID_calc), so the only bound is max_out on the running total.
 *
 * Both chassis loops -- yaw_gimble_pid and yaw_chassis_pid in data.c,
 * data_init() -- use PID_POSITION.
 */
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出

    /*
     * Last target and last feedback, stored purely for inspection -- nothing in
     * PID_calc() reads them back. On a board whose only serial port is the FOC
     * link they are the cheapest way to see what a loop was actually told
     * versus what it actually saw, read live over SWD.
     */
    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;
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
extern void PID_init(pid_type_def *pid, uint8_t mode, const float PID[3], float max_out, float max_iout);

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
/*
 * ARGUMENT ORDER -- the single easiest thing to get wrong in this file.
 *
 *     PID_calc(pid, ref, set)   computes   error = set - ref
 *
 * so the parameter NAMES read backwards: `ref` is the MEASUREMENT (feedback)
 * and `set` is the TARGET, while `ref` reads like "reference" -- i.e. target --
 * to most people. Both names come from the DJI reference implementation this
 * file is derived from.
 *
 * Every call site in this project passes the target FIRST and the measurement
 * second, so what the loops actually compute is
 *
 *     error = measurement - target
 *
 * the NEGATIVE of the textbook definition. This is a house convention, not a
 * mistake, and it is consistent across boards: the chassis does it in data.c
 * (pid_calculate), and so does the gimbal board, whose pid_calculate() also
 * computes the same error by hand and arrives at the identical sign. The
 * compensating inversion lives downstream in the actuator direction -- for the
 * chassis follow loop that is the global -omega in OmniKinematics() together
 * with the motor wiring.
 *
 * The practical consequence is that a positive Kp here drives the measurement
 * AWAY from the target unless something downstream inverts it. So never reason
 * about one sign in isolation, and never "correct" this on one board only: loop
 * polarity is a property of the whole chain, and the only honest way to settle
 * it is by measurement, with the robot on blocks and the wheels free.
 */
extern float PID_calc(pid_type_def *pid, float ref, float set);

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
extern void PID_clear(pid_type_def *pid);

#endif
