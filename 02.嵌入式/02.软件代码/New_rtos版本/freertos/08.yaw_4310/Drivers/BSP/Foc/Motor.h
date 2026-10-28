#ifndef Motor_H
#define Motor_H

#include "FOC.h"
#include "Pid.h"
#include "Data.h"

#define MOTOR_ALIGN_VOLTAGE   2.0f

#define YAW_FRICTION_COMP_VOLTAGE  0.8f
#define YAW_FRICTION_CMD_EPS       0.20f
#define YAW_FRICTION_ERR_ON_DEG    1.5f

#define YAW_RAIL_ERR_ON_DEG    5.0f
#define YAW_RAIL_ERR_FULL_DEG  8.2f

void Motor_init(void);

void motor_direction_test(void);

void Motor_stop(void);

void Motor_start(void);

float velocityOpenloop(float target_velocity);

void PID_Pos_Set(float P, float I, float D, float limit);

float position_error_rad(float target);

void PositionCloseloop(float position);

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
