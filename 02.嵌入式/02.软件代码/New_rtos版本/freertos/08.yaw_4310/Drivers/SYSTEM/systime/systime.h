#ifndef __SYSTIME_H__
#define __SYSTIME_H__

#include "sys.h"

void systime_init(void);

uint32_t micros(void);

uint32_t millis(void);

uint32_t micros_since(uint32_t prev_us);

float systime_delta_s(uint32_t *prev_us);

#endif
