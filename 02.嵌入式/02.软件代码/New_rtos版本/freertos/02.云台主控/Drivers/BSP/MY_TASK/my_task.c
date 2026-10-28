#include "my_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"
#include "stdio.h"

#include "imu.h"
#include "nrf24L01.h"
#include "oled.h"
#include "led.h"
#include "data.h"
#include "usart.h"
#include "ws2812b.h"

TaskHandle_t StartTask_Handler;
TaskHandle_t DATA_GET_Task_Handler;
TaskHandle_t DATA_COMM_Handler;
TaskHandle_t OLED_SHOW_Task_Handler;
TaskHandle_t PRINT_Task_Handler;
TaskHandle_t DATA_Exchange_Handler;

void create_task(void)
{
    xTaskCreate((TaskFunction_t )start_task, "start_task", START_STK_SIZE, NULL, START_TASK_PRIO, &StartTask_Handler);
}

void start_task(void *pvParameters)
{
    xTaskCreate(print_task,    "print",    PRINT_STK_SIZE,    NULL, PRINT_PRIO,      &PRINT_Task_Handler);
    xTaskCreate(data_get_task, "imu_get",  DATA_GET_STK_SIZE, NULL, DATA_GET_PRIO,   &DATA_GET_Task_Handler);
	xTaskCreate(data_exchange_task, "data_exchange",  DATA_EXCHANGE_SIZE, NULL, DATA_EXCHANGE_PRIO,   &DATA_Exchange_Handler);
    xTaskCreate(DATA_COMM_task,"nrf_comm", DATA_COMM_SIZE,    NULL, DATA_COMM_PRIO,  &DATA_COMM_Handler);

    vTaskDelete(NULL);
}

void data_get_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint16_t print_div = 0;
    while(1)
    {
        imu_updata();
        pid_calculate();
        if (++print_div >= 100u)
        {
            print_div = 0;
            data_print();
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}

void data_exchange_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        data_change();

        uart_sand();

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}

void DATA_COMM_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
		if(nrf24l01_rx_packet(B_rx_A_buf) == 0)
		{
			nrf_mark_rx();
		}
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2));
    }
}

void oled_show_task(void *pvParameters)
{

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1)
    {

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}

void print_task(void *pvParameters)
{
		TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1)
    {

		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}
