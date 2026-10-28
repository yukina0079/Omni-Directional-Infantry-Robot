#ifndef __SYS_H
#define __SYS_H

#include "stm32f1xx.h"

#define SYS_SUPPORT_OS          1

void sys_nvic_set_vector_table(uint32_t baseaddr, uint32_t offset);
void sys_standby(void);
void sys_soft_reset(void);
uint8_t sys_clock_set(uint32_t plln);
void sys_stm32_clock_init(uint32_t plln);

void sys_wfi_set(void);
void sys_intx_disable(void);
void sys_intx_enable(void);
void sys_msr_msp(uint32_t addr);

#endif
