/**
  ******************************************************************************
  * @file    Templates/Src/stm32f1xx.c
  * @author  MCD Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx_it.h"

/** @addtogroup STM32F1xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*
 * Fault record, readable over SWD after the fact.
 *
 * The handlers below used to be bare `while(1)` loops. On this board that is
 * close to the worst possible behaviour for two reasons:
 *
 *   1. TIM1 keeps running. The CPU stops, but the PWM peripheral does not, so
 *      the last commanded duty cycles stay on the gate driver -- a DC voltage
 *      vector parked on the winding, drawing stall current indefinitely. That
 *      is the one condition capable of actually cooking the motor.
 *   2. Nothing is recorded. Halting the target afterwards shows a PC somewhere
 *      inside the infinite loop, which says nothing about where the fault came
 *      from.
 *
 * So each handler now kills the PWM outputs first, then stores enough state to
 * diagnose the fault, then loops. `magic` lets yaw_monitor.py tell "a fault was
 * recorded" apart from "this RAM has never been written".
 */
#define FAULT_MAGIC   0x464C5431u   /* "FLT1" */

typedef struct {
    uint32_t magic;
    uint32_t kind;      /* 1 HardFault, 2 MemManage, 3 BusFault, 4 UsageFault */
    uint32_t cfsr;      /* Configurable Fault Status Register  */
    uint32_t hfsr;      /* HardFault Status Register           */
    uint32_t mmfar;     /* MemManage Fault Address Register    */
    uint32_t bfar;      /* BusFault Address Register           */
    uint32_t pc;        /* stacked return address = faulting instruction */
    uint32_t lr;        /* stacked link register                        */
    uint32_t psr;       /* stacked program status register              */
} fault_record_t;

volatile fault_record_t g_fault_record = {0};

/*
 * Disables the three TIM1 PWM outputs by clearing MOE (main output enable) in
 * BDTR, writing the register directly rather than going through HAL.
 *
 * Direct register access is deliberate here: HAL_TIM_PWM_Stop() walks handle
 * state and can call back into other HAL code, and a fault handler must not
 * depend on any of that still being consistent -- the fault may well have been
 * caused by corrupt state. Clearing one bit in one register cannot fail.
 */
static void fault_kill_pwm(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
}

/*
 * Records the fault. `sp` points at the exception stack frame that the CPU
 * pushed on entry, whose layout is fixed by the architecture:
 * R0 R1 R2 R3 R12 LR PC xPSR. The PC slot is the instruction that faulted,
 * which is the single most useful number for finding the bug in the .map file.
 */
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

/*
 * Common tail for every fault handler.
 *
 * Naked so the compiler emits no prologue: the prologue would move SP before we
 * can work out which stack the exception frame is on. Bit 2 of EXC_RETURN (in
 * LR on entry) selects MSP or PSP; this firmware is bare-metal single-stack so
 * it is always MSP, but reading it properly costs nothing and keeps the handler
 * correct if an RTOS is ever added -- the directory is called `freertos`, after
 * all.
 */
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

    /*
     * Park here. A debugger attached over SWD can now read g_fault_record and
     * see exactly what happened and where.
     *
     * Deliberately NOT auto-resetting: a fault that reset the board would make
     * the gimbal twitch back to life repeatedly, and the evidence would be gone
     * every time. A dead axis is safer and far easier to diagnose than one that
     * silently reboots in a loop.
     */
    while (1)
    {
    }
}

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/*
 * The four fault handlers below are identical apart from the kind code, so
 * they all tail-call the shared entry stub. `IMPORT` is deliberately absent
 * here: fault_entry is defined earlier in this same translation unit, and the
 * inline assembler rejects a repeated IMPORT of a name it already knows
 * (A1129E). Declaring it once, at the definition, is enough.
 */

/**
  * @brief  This function handles Hard Fault exception.
  */
__asm void HardFault_Handler(void)
{
    MOVS    R0, #1
    B       fault_entry
}

/**
  * @brief  This function handles Memory Manage exception.
  */
__asm void MemManage_Handler(void)
{
    MOVS    R0, #2
    B       fault_entry
}

/**
  * @brief  This function handles Bus Fault exception.
  */
__asm void BusFault_Handler(void)
{
    MOVS    R0, #3
    B       fault_entry
}

/**
  * @brief  This function handles Usage Fault exception.
  */
__asm void UsageFault_Handler(void)
{
    MOVS    R0, #4
    B       fault_entry
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/*                 STM32F1xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f1xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/


/**
  * @}
  */ 

/**
  * @}
  */
