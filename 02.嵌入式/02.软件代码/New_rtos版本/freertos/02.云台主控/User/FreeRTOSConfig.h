#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include <stdint.h>

extern uint32_t SystemCoreClock;

#define configENABLE_FPU                         1

#define configUSE_PREEMPTION                            1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION         1
#define configUSE_TICKLESS_IDLE                         0
#define configCPU_CLOCK_HZ                              SystemCoreClock

#define configTICK_RATE_HZ                              1000
#define configMAX_PRIORITIES                            32
#define configMINIMAL_STACK_SIZE                        128
#define configMAX_TASK_NAME_LEN                         16
#define configUSE_16_BIT_TICKS                          0
#define configIDLE_SHOULD_YIELD                         1
#define configUSE_TASK_NOTIFICATIONS                    1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES           1
#define configUSE_MUTEXES                               1
#define configUSE_RECURSIVE_MUTEXES                     1
#define configUSE_COUNTING_SEMAPHORES                   1
#define configUSE_ALTERNATIVE_API                       0
#define configQUEUE_REGISTRY_SIZE                       8
#define configUSE_QUEUE_SETS                            1
#define configUSE_TIME_SLICING                          1
#define configUSE_NEWLIB_REENTRANT                      0
#define configENABLE_BACKWARD_COMPATIBILITY             0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS         0
#define configSTACK_DEPTH_TYPE                          uint16_t
#define configMESSAGE_BUFFER_LENGTH_TYPE                size_t

#define configSUPPORT_STATIC_ALLOCATION                 0
#define configSUPPORT_DYNAMIC_ALLOCATION                1
#define configTOTAL_HEAP_SIZE                           ((size_t)(10 * 1024))
#define configAPPLICATION_ALLOCATED_HEAP                0
#define configSTACK_ALLOCATION_FROM_SEPARATE_HEAP       0

#define configUSE_IDLE_HOOK                             0
#define configUSE_TICK_HOOK                             0
#define configCHECK_FOR_STACK_OVERFLOW                  0
#define configUSE_MALLOC_FAILED_HOOK                    0
#define configUSE_DAEMON_TASK_STARTUP_HOOK              0

#define configGENERATE_RUN_TIME_STATS                   0
#if configGENERATE_RUN_TIME_STATS
#include "./BSP/TIMER/btim.h"
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()        ConfigureTimeForRunTimeStats()
extern uint32_t FreeRTOSRunTimeTicks;
#define portGET_RUN_TIME_COUNTER_VALUE()                FreeRTOSRunTimeTicks
#endif
#define configUSE_TRACE_FACILITY                        1
#define configUSE_STATS_FORMATTING_FUNCTIONS            1

#define configUSE_CO_ROUTINES                           0
#define configMAX_CO_ROUTINE_PRIORITIES                 2

#define configUSE_TIMERS                                1
#define configTIMER_TASK_PRIORITY                       ( configMAX_PRIORITIES - 1 )
#define configTIMER_QUEUE_LENGTH                        5
#define configTIMER_TASK_STACK_DEPTH                    ( configMINIMAL_STACK_SIZE * 2)

#define INCLUDE_vTaskPrioritySet                        1
#define INCLUDE_uxTaskPriorityGet                       1
#define INCLUDE_vTaskDelete                             1
#define INCLUDE_vTaskSuspend                            1
#define INCLUDE_xResumeFromISR                          1
#define INCLUDE_vTaskDelayUntil                         1
#define INCLUDE_vTaskDelay                              1
#define INCLUDE_xTaskGetSchedulerState                  1
#define INCLUDE_xTaskGetCurrentTaskHandle               1
#define INCLUDE_uxTaskGetStackHighWaterMark             1
#define INCLUDE_xTaskGetIdleTaskHandle                  1
#define INCLUDE_eTaskGetState                           1
#define INCLUDE_xEventGroupSetBitFromISR                1
#define INCLUDE_xTimerPendFunctionCall                  1
#define INCLUDE_xTaskAbortDelay                         1
#define INCLUDE_xTaskGetHandle                          1
#define INCLUDE_xTaskResumeFromISR                      1

#ifdef __NVIC_PRIO_BITS
    #define configPRIO_BITS __NVIC_PRIO_BITS
#else
    #define configPRIO_BITS 4
#endif

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    5
#define configKERNEL_INTERRUPT_PRIORITY                 ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY            ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
#define configMAX_API_CALL_INTERRUPT_PRIORITY           configMAX_SYSCALL_INTERRUPT_PRIORITY

#define xPortPendSVHandler                              PendSV_Handler
#define vPortSVCHandler                                 SVC_Handler

#define vAssertCalled(char, int) printf("Error: %s, %d\r\n", char, int)
#define configASSERT( x ) if( ( x ) == 0 ) vAssertCalled( __FILE__, __LINE__ )

#endif
