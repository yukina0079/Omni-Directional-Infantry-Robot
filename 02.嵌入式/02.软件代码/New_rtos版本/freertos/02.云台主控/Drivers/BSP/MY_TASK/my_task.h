#ifndef __MY_TASK_H
#define __MY_TASK_H

#include "sys.h"

/*
 * Gimbal MCU is not mounted on the moving pitch assembly, so BMI055
 * does not rotate with the barrel. The pitch FOC board already closes
 * on its AS5600; pass the remote's integrated ly angle through and
 * skip the outer attitude PID. Set back to 1 once the board rides
 * with the axis and BMI055 is a valid pitch reference.
 */
#define GIMBAL_IMU_ENABLE          0U

/*
 * Task priorities, same numbering scheme as the chassis (higher = more urgent).
 *
 *   9  data_get_task       IMU + pitch PID + 10 Hz telemetry
 *   8  DATA_COMM_task      radio receive
 *   7  data_exchange_task  frame rebuild AND serial transmit
 *   1  print_task          created, but its body only sleeps
 *  10  start_task          creates the others, deletes itself
 *   2  oled_show_task      NOT created
 *
 * Only FOUR tasks exist here against the chassis's five. There is no separate
 * UART_Trans_task: uart_sand() is called from data_exchange_task, immediately
 * after data_change(). That is a deliberate trade -- it saves a task's stack and
 * makes the frame copy atomic without a critical section (see uart_sand() in
 * data.c) -- but it also couples transmit timing to control timing, so a slow
 * control cycle delays the frame.
 *
 * NO WATCHDOG on this board. The chassis arms the IWDG and requires all four of
 * its tasks to check in; nothing equivalent runs here. The reasoning is that a
 * hung gimbal board stops sending FOC frames, and the pitch 4310 board notices
 * within its own 100 ms comms interlock and de-energises -- so the failure is
 * caught downstream. Worth knowing that this leaves the barrel limp rather than
 * held, which is the opposite of the chassis failsafe philosophy.
 */
#define START_TASK_PRIO    10  /* startup task */
#define DATA_GET_PRIO      9   /* IMU + pitch PID + telemetry (highest) */
#define DATA_COMM_PRIO     8   /* NRF receive (second highest) */
#define OLED_SHOW_PRIO     2   /* OLED display -- task never created */
#define PRINT_PRIO         1   /* debug print -- created, body only sleeps */
#define DATA_EXCHANGE_PRIO 7   /* frame rebuild + serial transmit */
#define UART_Trans_PRIO	   6   /* unused: no UART_Trans_task on this board */

/*
 * Stack sizes in WORDS (x4 bytes on Cortex-M4). Four tasks are actually
 * created: 128 + 512 + 128 + 256 = 1024 words = 4 KB of configTOTAL_HEAP_SIZE.
 *
 * DATA_GET gets 512 because it runs the Mahony AHRS, which is float-heavy and,
 * at -O0, spills every intermediate to the stack. PRINT_STK_SIZE is 768 -- three
 * times the chassis value -- because printf with %f pulls in the formatting
 * routines; that allowance is now mostly unused, since the telemetry moved into
 * data_get_task and print_task only sleeps.
 */
#define START_STK_SIZE     128
#define DATA_GET_STK_SIZE  512
#define DATA_COMM_SIZE     256
#define OLED_SHOW_STK_SIZE 256
#define PRINT_STK_SIZE     768
#define DATA_EXCHANGE_SIZE 128
#define UART_Trans_SIZE	   256
void create_task(void);
void start_task(void *pvParameters);
void print_task(void *pvParameters);
void data_get_task(void *pvParameters);
void DATA_COMM_task(void *pvParameters);
void oled_show_task(void *pvParameters);
void data_exchange_task(void *pvParameters);
void UART_Trans_task(void *pvParameters);
void create_task(void);

#endif


