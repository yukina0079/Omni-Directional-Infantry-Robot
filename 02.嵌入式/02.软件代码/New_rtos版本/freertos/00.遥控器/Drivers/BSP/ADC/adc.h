#ifndef  __ADC_H__
#define __ADC_H__
#include "sys.h"
void adc_init(void);
uint32_t adc_get_result(uint32_t ch);
void adc_dma_init(uint32_t *adc_value);

#endif
