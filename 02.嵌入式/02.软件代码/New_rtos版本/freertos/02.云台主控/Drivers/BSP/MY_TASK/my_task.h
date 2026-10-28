#ifndef __MY_TASK_H
#define __MY_TASK_H

#include "sys.h"

#define GIMBAL_IMU_ENABLE          0U

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
#define PRINT_STK_SIZE     768
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

#endif
