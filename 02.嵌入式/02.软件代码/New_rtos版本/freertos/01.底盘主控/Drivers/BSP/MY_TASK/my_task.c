#include "my_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"
#include "stdio.h"

/* Hardware driver headers */
#include "imu.h"
#include "nrf24L01.h"
#include "oled.h"
#include "led.h"
#include "data.h"
#include "usart.h"
#include "motor.h"    /* MOTOR_PWM_UPDATE(), used by the FreeRTOS fault hooks */

/*
 * Must stay 0 while USART1 is the yaw FOC link. The dump is ASCII hex on
 * the same wire as the 0x55..0xFF binary frames; it does not form a valid
 * frame itself, but it breaks frame_sync() for ~10 ms every period and
 * can trip the yaw board's 100 ms comms interlock if a binary burst is
 * delayed behind it.
 */
#define NRF_UART_DEBUG_ENABLE    0U
#define NRF_UART_DEBUG_PERIOD_MS 100U
#define NRF_PACKET_SIZE          32U

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

/* Task handles */
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

/*
 * THE FOUR TASKS, and why the work is split this way at all.
 *
 * Everything this board does could fit in one 1 ms loop. It is split into four
 * because the pieces have different failure modes and different tolerances for
 * being late, and separating them lets the priority ordering express that (see
 * my_task.h). What each one owns:
 *
 *   data_get_task       reads the IMU and runs both yaw PIDs. Owns attitude.
 *   DATA_COMM_task      polls the radio, validates header/tail, stamps the
 *                       failsafe timer. Owns "is the operator still there".
 *   data_exchange_task  runs data_change(): decode the frame, build the outgoing
 *                       one, mix the wheels. Owns control.
 *   UART_Trans_task     pushes the frame out by DMA and refreshes the watchdog.
 *                       Owns the downstream link.
 *
 * All four are created inside a critical section and start_task then deletes
 * itself, so no partially-created task set is ever schedulable.
 *
 * Two tasks are deliberately NOT created: print_task and oled_show_task. printf
 * lands on USART3, which is the FOC link -- see CHASSIS_UART3_DEBUG_ENABLE in
 * my_task.h for why those cannot coexist.
 *
 * Every task uses vTaskDelayUntil() rather than vTaskDelay(). The difference
 * matters here: vTaskDelay(1) sleeps one tick AFTER the body finishes, so the
 * period drifts by however long the body took and the control rate becomes
 * load-dependent. vTaskDelayUntil() holds an absolute wake time, so the loops
 * stay on a fixed 1 ms grid -- which is exactly what the PID gains assume, since
 * pid.c has no dt term at all.
 */
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();

//    xTaskCreate(print_task,    "print",    PRINT_STK_SIZE,    NULL, PRINT_PRIO,      &PRINT_Task_Handler);
    xTaskCreate(data_get_task, "imu_get",  DATA_GET_STK_SIZE, NULL, DATA_GET_PRIO,   &DATA_GET_Task_Handler);
	/*
	 * data_exchange_task runs data_change(): frame sync + reply decode + the
	 * serial transmit buffer. It is NOT the motor output. Motor output is the
	 * OmniKinematics() call inside data_change(), which the motor-output gate
	 * in data.c handles separately. Gating THIS task behind
	 * CHASSIS_MOTOR_OUTPUT_ENABLE froze the whole serial link -- the chassis
	 * never sent a frame -- while still letting the robot drive, which is the
	 * opposite of what the gate is for.
	 */
	xTaskCreate(data_exchange_task, "data_exchange",  DATA_EXCHANGE_SIZE, NULL, DATA_EXCHANGE_PRIO,   &DATA_Exchange_Handler);
	xTaskCreate(DATA_COMM_task,"nrf_comm", DATA_COMM_SIZE,    NULL, DATA_COMM_PRIO,  &DATA_COMM_Handler);
	xTaskCreate(UART_Trans_task,"UART_Trans", UART_Trans_SIZE,    NULL, UART_Trans_PRIO,  &UART_Trans_Handler);
//  xTaskCreate(oled_show_task,"oled",     OLED_SHOW_STK_SIZE,NULL, OLED_SHOW_PRIO,  &OLED_SHOW_Task_Handler);

    vTaskDelete(NULL);
    taskEXIT_CRITICAL();
}

/**
 * @brief 获取�螺仪数据 -- read the IMU and advance both yaw loops.
 *
 * Highest priority on the board (9). Runs every 1 ms via vTaskDelayUntil, and
 * that regularity is load-bearing twice over: pid.c integrates per call rather
 * than per second, and process_continuous_angle() needs consecutive samples
 * close enough together that a wrap is never mistaken for real motion.
 *
 * pid_calculate() is called only on a SUCCESSFUL read. A failed read takes the
 * imu_fault_reset() path instead, which zeroes yaw_output[] -- so a dead sensor
 * makes the follow loop command zero rotation rather than acting on a stale
 * angle. Translation is unaffected; it never needed attitude.
 *
 * The whole body is compiled out when CHASSIS_IMU_ENABLE is 0, leaving only the
 * watchdog check-in. That is intentional: the task must keep checking in even
 * with no sensor fitted, or the watchdog would reset a board that is working
 * exactly as configured.
 */
void data_get_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
#if CHASSIS_IMU_ENABLE
    uint16_t retry_div = 0;
#endif
    while(1)
    {
#if CHASSIS_IMU_ENABLE
        /*
         * While the IMU answers, read it every millisecond as before. Once it
         * has been declared dead, back off to IMU_RETRY_PERIOD_MS.
         *
         * A failed read blocks for the whole IIC_HW_TIMEOUT_FAST, and this is
         * the highest-priority task in the system: hammering a dead bus every
         * millisecond would consume the entire CPU inside a timeout loop and
         * starve the two things that matter when a sensor dies -- the radio
         * failsafe that stops the wheels, and the frame stream that keeps the
         * gimbal energised. Retrying at 100 ms costs about 5% and still
         * reconnects promptly if the sensor comes back.
         *
         * imu_fault_reset() has already zeroed yaw_output[] by the time the
         * back-off engages, so the follow rotation rate stays 0 while the
         * sensor is down. Translation is unaffected: it never needed attitude.
         */
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
/**
 * @brief 串口通讯数据 -- push the outgoing frame, and refresh the watchdog.
 *
 * Lowest of the four supervised tasks (6), which is what makes the watchdog
 * refresh meaningful; see IWDG_TASK_ALL in my_task.h.
 *
 * uart_sand() is non-blocking by design: it skips the cycle if the TX DMA is
 * still busy rather than waiting. At 1 ms period and 1.74 ms per 20-byte frame
 * that means roughly every other cycle is a skip, and the effective frame rate
 * settles near 570 Hz -- comfortably inside the yaw board's 100 ms interlock.
 */
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

        /*
         * This task issues the refresh as well as its own check-in, and it
         * is the lowest priority of the four supervised tasks (6, against
         * 7/8/9). A higher-priority task that spins without yielding
         * therefore starves the refresh too, and the watchdog fires -- which
         * a refresh sited in the highest-priority task would not do.
         */
        iwdg_task_checkin(IWDG_TASK_UART);
        iwdg_kick();

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}
/**
 * @brief 数据交换任务 -- the control loop proper.
 *
 * data_change() does the whole chain: decode the received frame, apply the radio
 * failsafe, build the outgoing frame, decode the yaw board's reply, rotate the
 * stick velocity into the chassis frame, and command the wheels.
 *
 * Note this task is NOT gated behind CHASSIS_MOTOR_OUTPUT_ENABLE. Gating it
 * would freeze the serial link as well, so the chassis would stop talking to the
 * gimbal while still being perfectly able to drive -- the exact opposite of what
 * a motor-output gate is for. The gate lives inside data_change(), around the
 * OmniKinematics() call only.
 */
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
/**
 * @brief 无线通讯任务 -- receive from the remote.
 *
 * Polled at 2 ms against a remote that transmits every 3 ms, so the poll is
 * comfortably faster than the source. This is the only place the radio failsafe
 * timer is refreshed, and only for a payload whose header and tail check out:
 * four receivers share one address on this link, so a collided or foreign
 * payload can still satisfy the radio's own CRC, and noise must not be able to
 * keep the wheels alive after the remote is switched off.
 */
void DATA_COMM_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while(1)
    {
		if(nrf24l01_rx_packet(C_rx_buf) == 0){
            /*
             * Validate before trusting it. Four receivers share one address on
             * this radio and all of them ACK, so a collided or foreign payload
             * can still satisfy the NRF's own CRC. Only a frame with the right
             * header and tail is allowed to refresh the failsafe timer --
             * otherwise noise would keep the wheels alive after the remote is
             * already off.
             */
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

/**
 * @brief OLED显示任务 (50ms周期) -- body empty, and the task is not created.
 */
void oled_show_task(void *pvParameters)
{


    TickType_t xLastWakeTime = xTaskGetTickCount(); /* absolute-deadline timing */
    while(1)
    {
	

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50)); 
    }
}

/**
 * @brief 调试打印任务 (10ms周期) -- not created; printf shares the FOC wire.
 */
void print_task(void *pvParameters)
{
		TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1) 
    {
		data_print();
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

/*
 * Required once configCHECK_FOR_STACK_OVERFLOW and
 * configUSE_MALLOC_FAILED_HOOK are enabled; without them the link fails.
 *
 * Deliberately not a bare while(1). A silent lockup leaves TIM8's compare
 * registers and the direction pins exactly as they were, so the wheels keep
 * turning at whatever they were last commanded -- the very runaway the rest of
 * this change set exists to prevent. Kill the output stage first, then stop.
 *
 * Interrupts go off before touching the motors so nothing can re-command them
 * between the stop and the halt.
 */
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
/*
 * ---------------------------------------------------------------------------
 * Independent watchdog
 * ---------------------------------------------------------------------------
 *
 * Driven at register level on purpose. stm32f4xx_hal_iwdg.c is not in
 * FreeRTOS.uvprojx -- only its header is reachable, through
 * stm32f4xx_hal_conf.h:57 -- so calling HAL_IWDG_Init() would mean editing the
 * Keil project to add a translation unit, i.e. a build-configuration change for
 * the sake of four register writes. The sequence below is the one
 * HAL_IWDG_Init() performs, in the same order.
 *
 * The IWDG cannot be stopped once started, and its reset is a full system
 * reset: TIM8 goes back to its reset state, so the compare registers become 0
 * and the wheels stop. That is the point. It is also why the two FreeRTOS fault
 * hooks above are now self-clearing -- their for(;;) no longer parks the board
 * forever, it parks it with the output stage already killed until the watchdog
 * reboots it. A stack overflow that reproduces will reset-loop, which is loud
 * and visible on LED3, and every pass through main() re-runs pwm_init(16799, 0)
 * before any task can command a wheel.
 */

#define IWDG_KEY_RELOAD     0x0000AAAAu   /* refresh the down-counter        */
#define IWDG_KEY_ENABLE     0x0000CCCCu   /* start it; also forces LSI on    */
#define IWDG_KEY_WRITE      0x00005555u   /* unprotect PR and RLR            */

/*
 * PR = 4 selects LSI/64 -> 500 Hz nominal -> 2 ms per count. RLR is 12 bits,
 * so this prescale reaches about 8.2 s; 500 ms needs 250 counts, well inside it.
 */
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
    /* Latch the reset cause before RMVF wipes it; see g_reset_csr in my_task.h. */
    g_reset_csr = RCC->CSR;
    __HAL_RCC_CLEAR_RESET_FLAGS();

#if CHASSIS_IWDG_DEBUG_FREEZE
    __HAL_DBGMCU_FREEZE_IWDG();
#endif

    IWDG->KR  = IWDG_KEY_ENABLE;
    IWDG->KR  = IWDG_KEY_WRITE;
    IWDG->PR  = IWDG_PRESCALE_CODE;
    IWDG->RLR = IWDG_RELOAD_COUNTS;

    /*
     * PR and RLR are written from the APB domain into the LSI domain. SR's PVU
     * and RVU stay set until the transfer lands -- up to 5 LSI cycles, near
     * 300 us at the 17 kHz corner -- and a reload key issued before then is
     * ignored, which would leave the counter running on the reset default
     * (LSI/4, 0xFFF: about 512 ms nominal, but 128 ms at 47 kHz) instead of the
     * configured window.
     */
    while (IWDG->SR != 0u)
    {
    }

    IWDG->KR = IWDG_KEY_RELOAD;

    s_iwdg_checkins = 0u;
    s_iwdg_last_ms  = HAL_GetTick();
}

/*
 * Called once per loop by each supervised task. The read-modify-write of the
 * shared mask is not atomic on Cortex-M4, and the callers span priorities 6..9,
 * so it needs the critical section however trivial it looks.
 */
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
        /*
         * Not everyone has run yet. Normal: DATA_COMM_task checks in every 2 ms
         * while this runs every 1 ms, so roughly every other call lands here.
         * Let the counter keep ageing -- that is the whole mechanism.
         */
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
#endif /* CHASSIS_IWDG_ENABLE */
