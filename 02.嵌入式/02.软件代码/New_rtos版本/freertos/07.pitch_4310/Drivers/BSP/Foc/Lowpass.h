#ifndef LOWPASS_H
#define LOWPASS_H

#include "FOC.h"

typedef struct
{
	float Tf;                      /* filter time constant, seconds */
	float y_prev;                  /* previous output */
	uint32_t timestamp_prev;       /* micros() stamp, see systime.h */
} LowPassFilter;

extern LowPassFilter  LPF_current,LPF_velocity;


void LOWPass_Init(void);
float LowPassFilter_operator(LowPassFilter *Lfi,float x);


#endif
