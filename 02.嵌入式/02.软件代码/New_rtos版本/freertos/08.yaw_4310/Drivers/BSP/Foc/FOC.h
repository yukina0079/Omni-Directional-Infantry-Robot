#ifndef FOC_H
#define FOC_H

#include "AS5600.h"
#include "Data.h"

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

/*
 * SVPWM modulation ceiling. In space-vector modulation the maximum realisable
 * phase voltage is Vbus/sqrt(3) = 0.577*Vbus; beyond that the duty cycles
 * would clip and distort the waveform.
 */
#define FOC_MAX_MODULATION   0.577f

extern float sensor_direction;
extern float pole_pairs;
extern float voltage_power_supply;
extern float zero_electric_angle;

/* Largest phase voltage the modulator can actually produce, in volts.
 * Use this to size PID output limits -- a limit above it is meaningless. */
float FOC_voltage_limit(void);

void  setPhaseVoltage(float Uq, float Ud, float angle_el);
float _normalizeAngle(float angle);
float _electricalAngle(void);

/* Fast trig, see FOC.c. */
float _sin(float a);
float _cos(float a);

#endif
