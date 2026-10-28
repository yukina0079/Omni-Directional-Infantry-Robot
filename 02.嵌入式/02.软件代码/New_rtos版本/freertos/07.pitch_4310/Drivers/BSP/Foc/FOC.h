#ifndef FOC_H
#define FOC_H

#include "AS5600.h"
#include "Data.h"

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define FOC_MAX_MODULATION   0.577f

extern float sensor_direction;
extern float pole_pairs;
extern float voltage_power_supply;
extern float zero_electric_angle;

float FOC_voltage_limit(void);

void  setPhaseVoltage(float Uq, float Ud, float angle_el);
float _normalizeAngle(float angle);
float _electricalAngle(void);

float _sin(float a);
float _cos(float a);

#endif
