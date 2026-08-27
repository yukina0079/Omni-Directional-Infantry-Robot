#ifndef Motor_H
#define Motor_H

#include "FOC.h"
#include "Pid.h"
#include "Data.h"

/*
 * Alignment drive voltage, in volts. Applied to the q axis while sweeping the
 * electrical angle to find the encoder's electrical zero. Keep it modest: this
 * torque physically drags the rotor, and on an assembled gimbal that means
 * swinging the barrel.
 */
#define MOTOR_ALIGN_VOLTAGE   2.0f

/*
 * Smooth Coulomb feedforward for the yaw axis, in volts.
 *
 * The old implementation was a relay: add +/-2 V whenever |error| > 0.5 deg.
 * That is a classic limit-cycle source on an inertia load (gimbal + barrel):
 * the shaft is kicked through the deadband, overshoots, and the 2 V flips.
 * SimpleFOC's hold-position thread and Andrew's inertia-vs-friction note
 * both describe this exact chatter.
 *
 * Replacement: Fc * tanh((P * error) / EPS).  The argument is commanded
 * effort, not the error sign, so the term lives on the feedforward path
 * (same idea as ODrive anticogging looking up the setpoint).  At hold,
 * P*error -> 0 and the extra voltage disappears; on a real move it
 * saturates quickly so the axis still breaks stiction and stays snappy.
 *
 * COMP kept modest so a hunting following-error cannot slam ±1.4 V
 * across the turret. ERR_ON stretched so the term stays off near
 * hold and only fills in on a real move.
 */
#define YAW_FRICTION_COMP_VOLTAGE  0.8f
#define YAW_FRICTION_CMD_EPS       0.20f
#define YAW_FRICTION_ERR_ON_DEG    1.5f

/*
 * When following error eats most of the 8.6 deg lead window, blend
 * Uq toward the 5.5 V rail. P*8.6 + Coulomb is only ~2.3 V -- the
 * left side stalled there at 0.23 A (takes 1-3, 3). Below 5 deg the
 * blend is off, so hold and slow tracking keep the soft 0.18 gain.
 */
#define YAW_RAIL_ERR_ON_DEG    5.0f
#define YAW_RAIL_ERR_FULL_DEG  8.2f

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
