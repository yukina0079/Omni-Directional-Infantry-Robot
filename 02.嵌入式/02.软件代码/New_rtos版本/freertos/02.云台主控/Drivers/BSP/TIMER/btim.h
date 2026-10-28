#ifndef __BTIM_H
#define __BTIM_H

#include "./SYSTEM/sys/sys.h"

#define BTIM_TIMX_INT                       TIM3
#define BTIM_TIMX_INT_IRQn                  TIM3_IRQn
#define BTIM_TIMX_INT_IRQHandler            TIM3_IRQHandler
#define BTIM_TIMX_INT_CLK_ENABLE()          do{ __HAL_RCC_TIM3_CLK_ENABLE(); }while(0)

void btim_timx_int_init(uint16_t arr, uint16_t psc);

#endif
