#include "my_task.h"
#include "frame13.h"
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
#include "motor.h"

#define NRF_UART_DEBUG_ENABLE    0U
#define NRF_UART_DEBUG_PERIOD_MS 100U
#define NRF_PACKET_SIZE          FRAME13_LEN

#if NRF_UART_DEBUG_ENABLE
static uint8_t nrf_debug_frame[NRF_PACKET_SIZE];
static uint8_t nrf_debug_pending;
static const char nrf_debug_hex[] = "0123456789ABCDEF";

static void nrf_debug_store(const uint8_t *frame)
{
    taskENTER_CRITICAL();
    memcpy(nrf_debug_frame, frame, NRF_PACKET_SIZE);
    nrf_debug_pending = 1U;
    taskEXIT_CRITICAL();
}

static uint8_t nrf_debug_take(uint8_t *frame)
{
    uint8_t pending;

    taskENTER_CRITICAL();
    pending = nrf_debug_pending;
    if (pending != 0U) {
        memcpy(frame, nrf_debug_frame, NRF_PACKET_SIZE);
        nrf_debug_pending = 0U;
    }
    taskEXIT_CRITICAL();

    return pending;
}

static void nrf_debug_send(const uint8_t *frame)
{
    uint8_t line[110];
    uint16_t pos = 0U;
    uint16_t i;

    line[pos++] = 'R';
    line[pos++] = 'F';
    line[pos++] = '_';
    line[pos++] = 'R';
    line[pos++] = 'X';
    line[pos++] = ' ';

    for (i = 0U; i < NRF_PACKET_SIZE; i++) {
        line[pos++] = (uint8_t)nrf_debug_hex[(frame[i] >> 4) & 0x0FU];
        line[pos++] = (uint8_t)nrf_debug_hex[frame[i] & 0x0FU];
        if (i + 1U < NRF_PACKET_SIZE) {
            line[pos++] = ' ';
        }
    }

    line[pos++] = '\r';
    line[pos++] = '\n';
    HAL_UART_Transmit(&g_uart1_handle, line, pos, 1000U);
}
#endif

TaskHandle_t StartTask_Handler;
TaskHandle_t DATA_GET_Task_Handler;
TaskHandle_t DATA_COMM_Handler;
TaskHandle_t OLED_SHOW_Task_Handler;
TaskHandle_t PRINT_Task_Handler;
TaskHandle_t DATA_Exchange_Handler;
TaskHandle_t UART_Trans_Handler;

void create_task(void)
{
    xTaskCreate((TaskFunction_t )start_task, "start_task", START_STK_SIZE, NULL, START_TASK_PRIO, &StartTask_Handler);
}

void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();

    xTaskCreate(data_get_task, "imu_get",  DATA_GET_STK_SIZE, NULL, DATA_GET_PRIO,   &DATA_GET_Task_Handler);

	xTaskCreate(data_exchange_task, "data_exchange",  DATA_EXCHANGE_SIZE, NULL, DATA_EXCHANGE_PRIO,   &DATA_Exchange_Handler);
	xTaskCreate(DATA_COMM_task,"nrf_comm", DATA_COMM_SIZE,    NULL, DATA_COMM_PRIO,  &DATA_COMM_Handler);
	xTaskCreate(UART_Trans_task,"UART_Trans", UART_Trans_SIZE,    NULL, UART_Trans_PRIO,  &UART_Trans_Handler);

    vTaskDelete(NULL);
    taskEXIT_CRITICAL();
}

void data_get_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
#if CHASSIS_IMU_ENABLE
    uint16_t retry_div = 0;
#endif
    while(1)
    {
#if CHASSIS_IMU_ENABLE

        if ((imu_healthy != 0U) || (++retry_div >= IMU_RETRY_PERIOD_MS))
        {
            retry_div = 0;
            if (imu_updata() != 0U)
            {
                pid_calculate();
            }
            else
            {
                imu_fault_reset();
            }
        }
#endif

        iwdg_task_checkin(IWDG_TASK_IMU);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}

void UART_Trans_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
#if CHASSIS_UART3_DEBUG_ENABLE
    TickType_t xLastAttTime = xTaskGetTickCount();
#endif
#if NRF_UART_DEBUG_ENABLE
    TickType_t xLastNrfDebugTime = xTaskGetTickCount();
    uint8_t debug_frame[NRF_PACKET_SIZE];
#endif

    while(1)
    {
		uart_sand();

#if CHASSIS_UART3_DEBUG_ENABLE
        if ((xTaskGetTickCount() - xLastAttTime) >= pdMS_TO_TICKS(100U))
        {
            xLastAttTime = xTaskGetTickCount();
            printf("ATT ypr=%.2f %.2f %.2f acc=%.2f %.2f %.2f gyr=%.1f %.1f %.1f dt=%.4f ok=%u\r\n",
                   INS_angle[0], INS_angle[1], INS_angle[2],
                   accel[0], accel[1], accel[2],
                   gyro[0], gyro[1], gyro[2],
                   imu_dt, (unsigned)imu_healthy);
        }
#endif

#if NRF_UART_DEBUG_ENABLE
        if ((xTaskGetTickCount() - xLastNrfDebugTime) >= pdMS_TO_TICKS(NRF_UART_DEBUG_PERIOD_MS)) {
            xLastNrfDebugTime = xTaskGetTickCount();
            if (nrf_debug_take(debug_frame) != 0U) {
                nrf_debug_send(debug_frame);
            }
        }
#endif

        iwdg_task_checkin(IWDG_TASK_UART);
        iwdg_kick();

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}

void data_exchange_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        data_change();

        iwdg_task_checkin(IWDG_TASK_EXCHANGE);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}

void DATA_COMM_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
		if(nrf24l01_rx_packet(C_rx_buf) == 0){

            if ((C_rx_buf[0] == 0x55u) && (C_rx_buf[12] == 0xFFu)) {
                nrf_mark_rx();
            }
#if NRF_UART_DEBUG_ENABLE
            nrf_debug_store(C_rx_buf);
#endif
        }
        iwdg_task_checkin(IWDG_TASK_NRF);

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
		data_print();
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;

    taskDISABLE_INTERRUPTS();
    MOTOR_PWM_UPDATE(0, 0, 0, 0);
    for (;;)
    {
    }
}

void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    MOTOR_PWM_UPDATE(0, 0, 0, 0);
    for (;;)
    {
    }
}

#if CHASSIS_IWDG_ENABLE

#define IWDG_KEY_RELOAD     0x0000AAAAu
#define IWDG_KEY_ENABLE     0x0000CCCCu
#define IWDG_KEY_WRITE      0x00005555u

#define IWDG_PRESCALE_CODE  4u
#define IWDG_TICK_MS        2u
#define IWDG_RELOAD_COUNTS  (CHASSIS_IWDG_PERIOD_MS / IWDG_TICK_MS)

volatile uint32_t g_reset_csr       = 0u;
volatile uint32_t g_iwdg_kicks      = 0u;
volatile uint32_t g_iwdg_gap_max_ms = 0u;

static volatile uint32_t s_iwdg_checkins = 0u;
static uint32_t          s_iwdg_last_ms  = 0u;

void iwdg_init(void)
{

    g_reset_csr = RCC->CSR;
    __HAL_RCC_CLEAR_RESET_FLAGS();

#if CHASSIS_IWDG_DEBUG_FREEZE
    __HAL_DBGMCU_FREEZE_IWDG();
#endif

    IWDG->KR  = IWDG_KEY_ENABLE;
    IWDG->KR  = IWDG_KEY_WRITE;
    IWDG->PR  = IWDG_PRESCALE_CODE;
    IWDG->RLR = IWDG_RELOAD_COUNTS;

    while (IWDG->SR != 0u)
    {
    }

    IWDG->KR = IWDG_KEY_RELOAD;

    s_iwdg_checkins = 0u;
    s_iwdg_last_ms  = HAL_GetTick();
}

void iwdg_task_checkin(uint32_t task_bit)
{
    taskENTER_CRITICAL();
    s_iwdg_checkins |= task_bit;
    taskEXIT_CRITICAL();
}

void iwdg_kick(void)
{
    uint32_t now;
    uint32_t gap;
    uint8_t  complete;

    taskENTER_CRITICAL();
    complete = ((s_iwdg_checkins & IWDG_TASK_ALL) == IWDG_TASK_ALL) ? 1u : 0u;
    if (complete != 0u)
    {
        s_iwdg_checkins = 0u;
    }
    taskEXIT_CRITICAL();

    if (complete == 0u)
    {

        return;
    }

    IWDG->KR = IWDG_KEY_RELOAD;

    now = HAL_GetTick();
    gap = now - s_iwdg_last_ms;
    s_iwdg_last_ms = now;
    if (gap > g_iwdg_gap_max_ms)
    {
        g_iwdg_gap_max_ms = gap;
    }
    g_iwdg_kicks++;
}
#endif
