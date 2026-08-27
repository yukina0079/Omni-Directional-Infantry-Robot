#ifndef Motor_H
#define Motor_H

#include "FOC.h"
#include "Pid.h"
#include "Data.h"

/*
 * Alignment drive voltage, in volts. It is capped by the same voltage-vector
 * limit as normal closed-loop operation.
 */
#define MOTOR_ALIGN_VOLTAGE   PITCH_TORQUE_LIMIT_V

/*
 * Coulomb assist covers dry friction only. Gravity is a separate
 * feedforward (PITCH_GRAVITY_HOLD_V) so hold no longer needs a
 * following error to generate the volts that keep the barrel up.
 * Live only when |error| is past the encoder-noise band.
 */
#define PITCH_FRICTION_COMP_VOLTAGE  1.2f
#define PITCH_FRICTION_CMD_EPS       0.08f
#define PITCH_FRICTION_ERR_ON_DEG    0.8f

/*
 * Horizon gravity hold, volts of Uq. Positive Uq lowers the barrel
 * (sensor_direction = -1), so the feedforward is negative and peaks
 * at PITCH_ZERO_RAD. Faded in PositionCloseloop() when error_deg > 0
 * so a down-stick can overpower the hold. Flip the sign if it climbs
 * unbidden with the stick centred.
 */
#define PITCH_GRAVITY_HOLD_V         2.0f

//初始化
void Motor_init(void);

/*
 * Determines sensor_direction by open-loop sweeping the electrical angle and
 * measuring which way the shaft actually turns; validates pole_pairs from the
 * magnitude of the same measurement. Writes the result into sensor_direction.
 *
 * Must run BEFORE the position loop is ever closed: a wrong sensor_direction
 * makes the position loop positive feedback. Rotor turns ~0.3 rev slowly.
 * Barrel off, supply current-limited. Takes ~3 s.
 */
void motor_direction_test(void);
/* Safe stop: zero duty, outputs disabled, PID state cleared. */
void Motor_stop(void);
/* Re-enables the outputs after a Motor_stop(). */
void Motor_start(void);

//开环速度控制
float velocityOpenloop(float target_velocity);

//闭环位置控制(单闭环无电流环)
void PID_Pos_Set(float P, float I, float D, float limit);
/*
 * Position error wrapped to the shortest path, in radians.
 *
 * Exposed so telemetry reports the same error the loop is acting on. Reporting
 * the raw difference instead would show a multi-turn value that the loop never
 * sees, which is exactly the sort of mismatch that sends a debugging session
 * chasing the wrong thing.
 */
float position_error_rad(float target);

void PositionCloseloop(float position);//位置
/*
 * Voltage-mode torque command. Uq is already in volts and is clipped to
 * PITCH_TORQUE_LIMIT_V. Commutation still uses the encoder; there is no
 * position or velocity loop.
 */
void TorqueCommand(float uq_volts);
//速度闭环(单闭环无电流环)
void PID_Vel_Set(float P, float I, float D, float limit);
void VelocityCloseloop(float Velocity);
//电流环
void PID_Cur_Set(float P, float I, float D, float limit);
void CurrentCloseloop(float current);
//位置速度环
void Position_VelocityCloseloop(float position);
//速度电流环
void Velocity_CurrentCloseloop(float velocity);
//位置电流环
void Position_CurrentCloseloop(float position);

extern PID PID_Pos;
extern PID PID_Vel;
extern PID PID_Cur;

#endif
