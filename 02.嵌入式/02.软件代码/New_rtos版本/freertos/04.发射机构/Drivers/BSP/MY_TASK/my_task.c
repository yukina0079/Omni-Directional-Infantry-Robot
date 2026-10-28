#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "my_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "data.h"
#include "key.h"
#include "nrf24L01.h"

TaskHandle_t StartTask_Handler;
TaskHandle_t DATA_GET_Task_Handler;
TaskHandle_t DATA_SEND_Handler;
TaskHandle_t printTask_Handler;
TaskHandle_t motor_Task_Handler;
TaskHandle_t light_Task_Handler;
TaskHandle_t OLED_SHOW_Task_Handler;

void create_task(void)
{
    xTaskCreate((TaskFunction_t )start_task,
                (const char*    )"start_task",
                (uint16_t       )START_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )START_TASK_PRIO,
                (TaskHandle_t*  )&StartTask_Handler);
}

void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();

    xTaskCreate((TaskFunction_t )data_get_task,
                (const char*    )"data_get_task",
                (uint16_t       )DATA_GET_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )DATA_GET_PRIO,
                (TaskHandle_t*  )&DATA_GET_Task_Handler);

    xTaskCreate((TaskFunction_t )DATA_SEND_task,
                (const char*    )"DATA_SEND_task",
                (uint16_t       )DATA_SEND_SIZE,
                (void*          )NULL,
                (UBaseType_t    )DATA_SEND,
                (TaskHandle_t*  )&DATA_SEND_Handler);

    xTaskCreate((TaskFunction_t )light_task,
                (const char*    )"light_task",
                (uint16_t       )light_TASK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )light_TASK_PRIO,
                (TaskHandle_t*  )&light_Task_Handler);

    xTaskCreate((TaskFunction_t )motor_task,
                (const char*    )"motor_task",
                (uint16_t       )motor_TASK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )motor_TASK_PRIO,
                (TaskHandle_t*  )&motor_Task_Handler);

    vTaskDelete(StartTask_Handler);
    taskEXIT_CRITICAL();
}
void light_task(void *pvParameters)
{
    while(1)
    {
		light_contral();
		vTaskDelay(5);
    }

}
void motor_task(void *pvParameters)
{
    while(1)
    {
		motor_contral();
		vTaskDelay(2);
    }

}

void print_task(void *pvParameters)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
		data_print();
        vTaskDelay(20);
    }
}

void data_get_task(void *pvParameters)
{
    while(1)
    {
		data_change();
		vTaskDelay(2);
    }
}

void DATA_SEND_task(void *pvParameters)
{
    while(1)
    {

		vTaskDelay(5);
	}
}
void oled_show_task(void *pvParameters)
{

    while(1)
    {

		vTaskDelay(100);
    }
}
