#ifndef Motor_H
#define Motor_H

#include "FOC.h"
#include "Pid.h"
#include "Data.h"

#define MOTOR_ALIGN_VOLTAGE   PITCH_TORQUE_LIMIT_V

#define PITCH_FRICTION_COMP_VOLTAGE  1.2f
#define PITCH_FRICTION_CMD_EPS       0.08f
#define PITCH_FRICTION_ERR_ON_DEG    0.8f

#define PITCH_GRAVITY_HOLD_V         2.0f

void Motor_init(void);

void motor_direction_test(void);

void Motor_stop(void);

void Motor_start(void);

float velocityOpenloop(float target_velocity);

void PID_Pos_Set(float P, float I, float D, float limit);

float position_error_rad(float target);

void PositionCloseloop(float position);

void TorqueCommand(float uq_volts);

void PID_Vel_Set(float P, float I, float D, float limit);
void VelocityCloseloop(float Velocity);

void PID_Cur_Set(float P, float I, float D, float limit);
void CurrentCloseloop(float current);

void Position_VelocityCloseloop(float position);

void Velocity_CurrentCloseloop(float velocity);

void Position_CurrentCloseloop(float position);

extern PID PID_Pos;
extern PID PID_Vel;
extern PID PID_Cur;

#endif
