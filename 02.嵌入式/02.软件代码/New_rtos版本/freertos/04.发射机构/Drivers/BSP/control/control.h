#ifndef  __CONTROL_H__
#define  __CONTROL_H__
#include "sys.h"

void NRF24L01_find(void);
void NRF24L01_recive(void);
float two_uint8_to_float_signed(uint8_t high_byte, uint8_t low_byte);
void float_to_two_uint8_signed(float num, uint8_t* high_byte, uint8_t* low_byte);

#endif
