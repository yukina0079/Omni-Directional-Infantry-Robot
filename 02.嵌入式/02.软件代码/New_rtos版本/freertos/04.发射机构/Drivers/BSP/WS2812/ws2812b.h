#ifndef  __WS2812B_H__
#define  __WS2812B_H__
#include "sys.h"


void dma_init(void);
void ws2812_init(uint16_t arr,uint16_t psc);
void WS2812_Set(uint8_t index,uint8_t r,uint8_t g,uint8_t b);
void WS2812_SetALL(uint8_t r,uint8_t g,uint8_t b);
void WS2812_Updata(void);
void WS2812_FlowLight(uint8_t r, uint8_t g, uint8_t b, uint16_t delay_ms, uint8_t direction);

#endif


