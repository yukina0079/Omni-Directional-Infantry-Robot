#ifndef __MY_TASK_H__
#define __MY_TASK_H__

#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"
#include "led.h"

#define START_TASK_PRIO 10
#define START_STK_SIZE  128
extern TaskHandle_t StartTask_Handler;
void start_task(void *pvParameters);

#define print_PRIO      2
#define print_STK_SIZE  128
extern TaskHandle_t printTask_Handler;
void print_task(void *pvParameters);

#define DATA_GET_PRIO      8
#define DATA_GET_STK_SIZE  128
extern TaskHandle_t DATA_GET_Task_Handler;
void data_get_task(void *pvParameters);

#define DATA_SEND          7
#define DATA_SEND_SIZE  128
extern TaskHandle_t DATA_SEND_Handler;
void DATA_SEND_task(void *pvParameters);

#define light_TASK_PRIO    5
#define light_TASK_SIZE  128
extern TaskHandle_t light_TASK_Handler;
void light_task(void *pvParameters);

#define motor_TASK_PRIO    6
#define motor_TASK_SIZE  128
extern TaskHandle_t motor_TASK_Handler;
void motor_task(void *pvParameters);

#define OLED_SHOW_PRIO      3
#define OLED_SHOW_STK_SIZE  128
extern TaskHandle_t OLED_SHOW_Task_Handler;
void oled_show_task(void *pvParameters);

void create_task(void);

#endif
