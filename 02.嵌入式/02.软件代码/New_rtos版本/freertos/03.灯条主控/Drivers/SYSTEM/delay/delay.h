#ifndef __DELAY_H__
#define __DELAY_H__

#include "sys.h"

void delay_ms(uint32_t nms);
void delay_us(uint32_t nus);
void delay_s(uint32_t ns);
void HAL_Delay(uint32_t nms);

#endif
