#ifndef  __WS2812B_H__
#define  __WS2812B_H__
#include "sys.h"

void dma_init(void);
void pwm_init(void);
void WS2812_Set(uint8_t index,uint8_t r,uint8_t g,uint8_t b);
void WS2812_SetALL(uint8_t r,uint8_t g,uint8_t b);
void WS2812_Updata(void);
void WS2812_FlowLight_Task(uint8_t r, uint8_t g, uint8_t b, uint16_t delay_ms, uint8_t direction);

#endif


