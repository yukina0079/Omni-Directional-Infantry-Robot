#include "my_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"
#include "stdio.h"

/* Ӳ������ͷ�ļ� */
#include "imu.h"
#include "nrf24L01.h"
#include "oled.h"
#include "led.h"
#include "data.h"
#include "usart.h"
#include "ws2812b.h"

/* ������ */
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

/**
 * @brief 获取传感器数据 -- IMU, pitch loop, and telemetry.
 *
 * Highest priority, 1 ms period. Unlike the chassis version this task does three
 * jobs: imu_updata(), pid_calculate(), and a 10 Hz data_print(). Folding the
 * print in here is only safe because printf goes to USART2, a port nothing else
 * uses; the ~90-character line takes about 7.8 ms of wire time at 115200, which
 * would be catastrophic on a shared FOC link and is merely idle time here.
 *
 * Note pid_calculate() runs unconditionally, without checking whether the IMU
 * read succeeded. It does not need to: the health check is the first thing
 * pid_calculate() itself does, and its fault path ramps the output safely to
 * zero. The chassis makes the opposite choice -- gate the call from outside -- for
 * the same net effect.
 */void data_get_task(void *pvParameters)
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
/**
 * @brief 数据交换任务 -- rebuild the FOC frame and push it out.
 *
 * data_change() then uart_sand(), back to back in one task at 1 ms. Keeping them
 * in the same task is what makes the frame copy inside uart_sand() atomic without
 * a critical section: nothing can run between building the frame and copying it.
 * Splitting them into two tasks, as the chassis does, requires adding that
 * critical section -- see the note in uart_sand().
 */void data_exchange_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        data_change();
        /* Reuse this task so TX does not need another heap allocation. */
        uart_sand();

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}
/**
 * @brief 无线通讯任务 -- receive the operator frame.
 *
 * Polled at 2 ms against a remote transmitting every 3 ms.
 *
 * Note what is MISSING compared with the chassis: there is no header/tail check
 * before nrf_mark_rx(). Four receivers share one radio address, so a collided or
 * foreign payload can satisfy the module's own CRC and still refresh the link
 * timer here. The consequence is bounded -- data_change() re-stamps the header and
 * tail itself and only relays bytes 1..11, so a corrupt payload produces one bad
 * pitch command rather than a lost frame lock -- but the 200 ms failsafe can be
 * held alive by noise, which is precisely the hole the chassis closed.
 */void DATA_COMM_task(void *pvParameters)
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

/**
 * @brief OLED显示任务 (50ms周期) -- body empty, and NOT created and never was.
 *
 * start_task() does not call xTaskCreate for this one. Left in place because the
 * OLED header exists on the PCB; wiring a display back up means filling in the
 * body and adding the create call.
 */void oled_show_task(void *pvParameters)
{


    TickType_t xLastWakeTime = xTaskGetTickCount(); // ���þ�׼��ʱ
    while(1)
    {
	

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50)); 
    }
}

/**
 * @brief 调试打印任务 -- deliberately empty.
 *
 * Created (unlike oled_show_task) but its loop only sleeps: telemetry moved into
 * data_get_task so that the print is phase-locked to the control cycle it
 * reports on. Kept as slack -- a live task at the lowest priority is a convenient
 * place to hang a new diagnostic without disturbing the timing of anything else.
 */void print_task(void *pvParameters)
{
		TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1) 
    {
		/* data_get_task already prints MOTOR at 10 Hz; keep this task as slack. */
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}
