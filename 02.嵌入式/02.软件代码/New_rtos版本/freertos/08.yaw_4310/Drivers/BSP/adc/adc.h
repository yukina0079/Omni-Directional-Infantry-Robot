#ifndef __ADC_H__
#define __ADC_H__

#include "sys.h"

void adc_dma_init(uint32_t *mar);

float _readADCVoltage(uint16_t ch);

/* Clarke transform + q-axis projection. Takes phase A and phase C currents
 * (that is what the two INA240 shunt amplifiers measure on this board). */
float cal_Iq_Id(float i_a, float i_c, float angle_el);

void Current_calibrateOffsets(void);
void CurrSense_init(void);
void CurrSense_getPhaseCurrents(void);

/* Filtered q-axis current in amps. */
float Get_Current(void);

/*
 * Static DC-injection test for the two INA240 channels' sign convention.
 * Parks a voltage vector on the phase A axis, then on the phase C axis, and
 * reports the measured currents plus their ratio (expected -0.5).
 *
 * Nothing spins, but the rotor WILL snap to the commanded angle. Barrel off,
 * supply current-limited to ~1 A. u_test = 2.0 V is a reasonable starting
 * point. Takes about 3 s.
 */
void current_polarity_test(float u_test);

#endif
