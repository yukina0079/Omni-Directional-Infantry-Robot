#ifndef  __AS5600_H__
#define  __AS5600_H__
#include "sys.h"
#include "Data.h"

typedef struct
{
	int32_t  cpr;                   /* encoder resolution, AS5600 = 12bit (4096) */
	int32_t  angle_data_prev;       /* previous raw count, for wrap detection */
	float    angle_prev;            /* previous continuous angle, for velocity */
	float    full_rotation_offset;  /* accumulated multi-turn offset, radians */
	uint32_t velocity_calc_timestamp; /* micros() stamp, see systime.h */
} ENCODER_TypeDef;

/*
 * Encoder health. Set when an I2C transaction to the AS5600 fails (NACK, i.e.
 * unplugged / unpowered / wedged sensor). Once set, the cached angle is stale
 * and MUST NOT be used to commutate: driving PWM from a frozen angle is how a
 * gimbal runs away. The main loop checks this and stops the motor.
 */
extern volatile uint8_t encoder_fault;

void as5600_init(void);

/*
 * Samples the encoder ONCE and refreshes the cached raw/mechanical/electrical
 * angles. Call this exactly once at the top of each control cycle; everything
 * below then reads the cache instead of hitting the I2C bus again.
 *
 * Returns 1 on success, 0 on I2C failure (and sets encoder_fault).
 *
 * This is the fix for the old design, where _electricalAngle() and Get_Angel()
 * each triggered their own bit-banged transaction -- a cascaded position/
 * velocity loop hit the bus four times per cycle.
 */
uint8_t as5600_update(void);

/* Raw 0..4095 count from the most recent as5600_update(). */
float as5600_read_angal(void);

/* Continuous (multi-turn) mechanical angle in radians, from the cache. */
float Get_Angel(void);

/* Single-turn mechanical angle in radians (0..2PI), from the cache. */
float Get_Angel_Notrack(void);

/* Filtered mechanical velocity in rad/s, from the cache. */
float Get_Velocity(void);

/*
 * Measures the I2C half-bit period and the cost of one encoder sample, then
 * scans the clock rate downward to find where this bus actually fails.
 * Replaces what a scope would have been used for. Results go to the serial
 * port. Motor must be off and the shaft stationary; takes about 15 s.
 */
void as5600_selftest(void);

#endif
