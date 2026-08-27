#ifndef __MY_TASK_H__  
#define __MY_TASK_H__

#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"
#include "led.h"

/* START_TASK任务配置
 * 注意：句柄只声明（extern），不定义！
 */
#define START_TASK_PRIO 10               /* 任务优先级 */
#define START_STK_SIZE  128             /* 任务堆栈大小 */
extern TaskHandle_t StartTask_Handler;  /* 任务句柄：用extern声明，告诉编译器在其他.c文件中定义 */
void start_task(void *pvParameters);    /* 任务函数声明 */


/* print任务配置 */
#define print_PRIO      2                /* 任务优先级 */
#define print_STK_SIZE  128              /* 任务堆栈大小 */
extern TaskHandle_t printTask_Handler;   /* 任务句柄：仅声明 */
void print_task(void *pvParameters);     /* 任务函数声明 */


/* DATA_GET_任务配置 */
#define DATA_GET_PRIO      8                /* 任务优先级 */
#define DATA_GET_STK_SIZE  128              /* 任务堆栈大小 */
extern TaskHandle_t DATA_GET_Task_Handler;   /* 任务句柄：仅声明 */
void data_get_task(void *pvParameters);     /* 任务函数声明 */

/*DATA_SEND任务配置 */
#define DATA_SEND          9             /* 任务优先级 */
#define DATA_SEND_SIZE  128              /* 任务堆栈大小 */
extern TaskHandle_t DATA_SEND_Handler;   /* 任务句柄：仅声明 */
void DATA_SEND_task(void *pvParameters);     /* 任务函数声明 */

/* oled_show_task任务配置 */
#define OLED_SHOW_PRIO      3                /* 任务优先级 */
#define OLED_SHOW_STK_SIZE  128              /* 任务堆栈大小 */
extern TaskHandle_t OLED_SHOW_Task_Handler;   /* 任务句柄：仅声明 */
void oled_show_task(void *pvParameters);     /* 任务函数声明 */

void create_task(void);  /* 任务创建函数声明 */


#endif
