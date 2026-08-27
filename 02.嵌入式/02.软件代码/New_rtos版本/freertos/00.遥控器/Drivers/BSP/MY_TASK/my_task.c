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
    xTaskCreate((TaskFunction_t )start_task,            /* 任务函数 */
                (const char*    )"start_task",          /* 任务名称 */
                (uint16_t       )START_STK_SIZE,        /* 任务堆栈大小 */
                (void*          )NULL,                  /* 传入给任务函数的参数 */
                (UBaseType_t    )START_TASK_PRIO,       /* 任务优先级 */
                (TaskHandle_t*  )&StartTask_Handler);   /* 任务句柄 */
}
/**
 * @brief   start_task
 * @param   pvParameters: 传入参数(未用到)
 * @retval  无
 */
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           /* 进入临界区 */
	
    /* 创建data_print任务 */
    xTaskCreate((TaskFunction_t )print_task,
                (const char*    )"print_task",
                (uint16_t       )print_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )print_PRIO,
                (TaskHandle_t*  )&printTask_Handler);
				
    /* 创建data_get任务 */
    xTaskCreate((TaskFunction_t )data_get_task,
                (const char*    )"data_get_task",
                (uint16_t       )DATA_GET_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )DATA_GET_PRIO,
                (TaskHandle_t*  )&DATA_GET_Task_Handler);
                
    /* 创建DATA_SEND任务 */
    xTaskCreate((TaskFunction_t )DATA_SEND_task,
                (const char*    )"DATA_SEND_task",
                (uint16_t       )DATA_SEND_SIZE,
                (void*          )NULL,
                (UBaseType_t    )DATA_SEND,
                (TaskHandle_t*  )&DATA_SEND_Handler);
 
    /* 创建oled_show任务 */
    xTaskCreate((TaskFunction_t )oled_show_task,
                (const char*    )"oled_show_task",
                (uint16_t       )OLED_SHOW_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )OLED_SHOW_PRIO,
                (TaskHandle_t*  )&OLED_SHOW_Task_Handler);
				
    vTaskDelete(StartTask_Handler); /* 删除开始任务 */
    taskEXIT_CRITICAL();            /* 退出临界区 */
}
/**
 * @brief   打印任务
 * @param   pvParameters: 传入参数(未用到)
 * @retval  无
 */
void print_task(void *pvParameters)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
		data_print();
        vTaskDelay(10);
    }
}
/**
 * @brief   data_get任务
 * @param   pvParameters: 传入参数(未用到)
 * @retval  无
 */
void data_get_task(void *pvParameters)
{
    while(1)
    {
		data_change();
		vTaskDelay(2);
    }
}

/**
 * @brief   DATA_SEND任务
 * @param   pvParameters: 传入参数(未用到)
 * @retval  无
 */
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
//	oled_show_string(0  , 0,"chassis:", 12);
//	oled_show_string(0  , 2,"color:", 12);
    while(1)
    {
		snprintf(key_num_buffer, sizeof(key_num_buffer),"key:%2X",key_num);
		oled_show_string(0, 0,key_num_buffer,12);	

//			if(key_num == 0x3f){
//				key_flag = 1;
//				if(key_flag == 1){
//				oled_show_string(48  , 0,"gyroscope", 12);
//				oled_show_string(40  , 2,"blue", 12);
//				}
//				key_flag = 0;
//			}
//			if(key_num == 0x6f){
//				oled_show_string(48  , 0,"fllow    ", 12);
//				oled_show_string(40  , 2,"red ", 12);
//			}
			

		vTaskDelay(100);
    }
}

