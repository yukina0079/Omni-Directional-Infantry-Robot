#include "freertos_demo.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/LED/led.h"

#include "FreeRTOS.h"
#include "task.h"

#define START_TASK_PRIO 1
#define START_STK_SIZE  128
TaskHandle_t            StartTask_Handler;
void start_task(void *pvParameters);

#define TASK1_PRIO      2
#define TASK1_STK_SIZE  128
TaskHandle_t            Task1Task_Handler;
void task1(void *pvParameters);

#define TASK2_PRIO      3
#define TASK2_STK_SIZE  128
TaskHandle_t            Task2Task_Handler;
void task2(void *pvParameters);

void freertos_demo(void)
{

    xTaskCreate((TaskFunction_t )start_task,
                (const char*    )"start_task",
                (uint16_t       )START_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )START_TASK_PRIO,
                (TaskHandle_t*  )&StartTask_Handler);
    vTaskStartScheduler();
}

void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();

    xTaskCreate((TaskFunction_t )task1,
                (const char*    )"task1",
                (uint16_t       )TASK1_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )TASK1_PRIO,
                (TaskHandle_t*  )&Task1Task_Handler);

    xTaskCreate((TaskFunction_t )task2,
                (const char*    )"task2",
                (uint16_t       )TASK2_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )TASK2_PRIO,
                (TaskHandle_t*  )&Task2Task_Handler);
    vTaskDelete(StartTask_Handler);
    taskEXIT_CRITICAL();
}

void task1(void *pvParameters)
{

    while(1)
    {

        LED0_TOGGLE();
        vTaskDelay(1000);
    }
}

void task2(void *pvParameters)
{
    float float_num = 0.0;

    while(1)
    {
        float_num += 0.01f;
        printf("float_num: %0.4f\r\n", float_num);
        vTaskDelay(1000);
    }
}
