#ifndef LOWPASS_H
#define LOWPASS_H

#include "FOC.h"

typedef struct
{
	float Tf;
	float y_prev;
	uint32_t timestamp_prev;
} LowPassFilter;

extern LowPassFilter  LPF_current,LPF_velocity;

void LOWPass_Init(void);
float LowPassFilter_operator(LowPassFilter *Lfi,float x);

#endif
