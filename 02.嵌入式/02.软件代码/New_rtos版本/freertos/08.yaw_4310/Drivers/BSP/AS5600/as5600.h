#ifndef  __AS5600_H__
#define  __AS5600_H__
#include "sys.h"
#include "Data.h"

typedef struct
{
	int32_t  cpr;
	int32_t  angle_data_prev;
	float    angle_prev;
	float    full_rotation_offset;
	uint32_t velocity_calc_timestamp;
} ENCODER_TypeDef;

extern volatile uint8_t encoder_fault;

void as5600_init(void);

uint8_t as5600_update(void);

float as5600_read_angal(void);

float Get_Angel(void);

float Get_Angel_Notrack(void);

float Get_Velocity(void);

void as5600_selftest(void);

#endif
