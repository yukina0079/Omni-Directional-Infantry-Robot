#include "stm32f1xx_hal.h"
#include "stm32f1xx_it.h"

#define FAULT_MAGIC   0x464C5431u

typedef struct {
    uint32_t magic;
    uint32_t kind;
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t mmfar;
    uint32_t bfar;
    uint32_t pc;
    uint32_t lr;
    uint32_t psr;
} fault_record_t;

volatile fault_record_t g_fault_record = {0};

static void fault_kill_pwm(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
}

static void fault_record(uint32_t kind, uint32_t *sp)
{
    g_fault_record.magic = FAULT_MAGIC;
    g_fault_record.kind  = kind;
    g_fault_record.cfsr  = SCB->CFSR;
    g_fault_record.hfsr  = SCB->HFSR;
    g_fault_record.mmfar = SCB->MMFAR;
    g_fault_record.bfar  = SCB->BFAR;
    g_fault_record.pc    = sp[6];
    g_fault_record.lr    = sp[5];
    g_fault_record.psr   = sp[7];
}

__asm void fault_entry(void)
{
    IMPORT  fault_common
    TST     LR, #4
    ITE     EQ
    MRSEQ   R1, MSP
    MRSNE   R1, PSP
    B       fault_common
}

void fault_common(uint32_t kind, uint32_t *sp)
{
    fault_kill_pwm();
    fault_record(kind, sp);

    while (1)
    {
    }
}

void NMI_Handler(void)
{
}

__asm void HardFault_Handler(void)
{
    MOVS    R0, #1
    B       fault_entry
}

__asm void MemManage_Handler(void)
{
    MOVS    R0, #2
    B       fault_entry
}

__asm void BusFault_Handler(void)
{
    MOVS    R0, #3
    B       fault_entry
}

__asm void UsageFault_Handler(void)
{
    MOVS    R0, #4
    B       fault_entry
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}
