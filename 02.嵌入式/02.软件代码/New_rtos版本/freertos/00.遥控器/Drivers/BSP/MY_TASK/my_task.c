#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "my_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "data.h"
#include "adc.h"
#include "encoder.h"
#include "oled.h"
#include "key.h"
#include "nrf24L01.h"

TaskHandle_t StartTask_Handler;
TaskHandle_t DATA_GET_Task_Handler;
TaskHandle_t DATA_SEND_Handler;
TaskHandle_t OLED_SHOW_Task_Handler;
TaskHandle_t printTask_Handler;

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

    xTaskCreate((TaskFunction_t )print_task,
                (const char*    )"print_task",
                (uint16_t       )print_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )print_PRIO,
                (TaskHandle_t*  )&printTask_Handler);

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

    xTaskCreate((TaskFunction_t )oled_show_task,
                (const char*    )"oled_show_task",
                (uint16_t       )OLED_SHOW_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )OLED_SHOW_PRIO,
                (TaskHandle_t*  )&OLED_SHOW_Task_Handler);

    vTaskDelete(StartTask_Handler);
    taskEXIT_CRITICAL();
}

void print_task(void *pvParameters)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
		data_print();
        vTaskDelay(10);
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
		if(NRF24L01_TxPacket(Send_Out)==TX_OK){}
		vTaskDelay(3);
	}
}
void oled_show_task(void *pvParameters)
{

	oled_fill(0x00);

    while(1)
    {
		snprintf(key_num_buffer, sizeof(key_num_buffer),"key:%2X",key_num);
		oled_show_string(0, 0,key_num_buffer,12);

		vTaskDelay(100);
    }
}
