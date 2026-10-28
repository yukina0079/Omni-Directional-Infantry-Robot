#ifndef __MY_TASK_H
#define __MY_TASK_H

#include "sys.h"

#define CHASSIS_IMU_ENABLE          1U
#define CHASSIS_MOTOR_OUTPUT_ENABLE 1U

#define CHASSIS_NRF_HOLD_MS         200U

#define YAW_REPLY_HOLD_MS           100U

#define IMU_RETRY_PERIOD_MS         100U

#define CHASSIS_UART3_DEBUG_ENABLE  0U

#define CHASSIS_IWDG_ENABLE         1U
#define CHASSIS_IWDG_PERIOD_MS      500U

#define CHASSIS_IWDG_DEBUG_FREEZE   1U

#define IWDG_TASK_IMU        (1U << 0)
#define IWDG_TASK_NRF        (1U << 1)
#define IWDG_TASK_EXCHANGE   (1U << 2)
#define IWDG_TASK_UART       (1U << 3)
#define IWDG_TASK_ALL        (IWDG_TASK_IMU | IWDG_TASK_NRF | \
                              IWDG_TASK_EXCHANGE | IWDG_TASK_UART)

#define START_TASK_PRIO    10
#define DATA_GET_PRIO      9
#define DATA_COMM_PRIO     8
#define OLED_SHOW_PRIO     2
#define PRINT_PRIO         1
#define DATA_EXCHANGE_PRIO 7
#define UART_Trans_PRIO	   6

#define START_STK_SIZE     128
#define DATA_GET_STK_SIZE  512
#define DATA_COMM_SIZE     256
#define OLED_SHOW_STK_SIZE 256
#define PRINT_STK_SIZE     256
#define DATA_EXCHANGE_SIZE 128
#define UART_Trans_SIZE	   256

void create_task(void);
void start_task(void *pvParameters);
void print_task(void *pvParameters);
void data_get_task(void *pvParameters);
void DATA_COMM_task(void *pvParameters);
void oled_show_task(void *pvParameters);
void data_exchange_task(void *pvParameters);
void UART_Trans_task(void *pvParameters);
void create_task(void);

#if CHASSIS_IWDG_ENABLE
void iwdg_init(void);
void iwdg_task_checkin(uint32_t task_bit);
void iwdg_kick(void);

extern volatile uint32_t g_reset_csr;
extern volatile uint32_t g_iwdg_kicks;
extern volatile uint32_t g_iwdg_gap_max_ms;
#else
#define iwdg_init()             do {} while (0)
#define iwdg_task_checkin(bit)  do {} while (0)
#define iwdg_kick()             do {} while (0)
#endif

#endif
