#ifndef __MY_TASK_H
#define __MY_TASK_H

#include "sys.h"

/* Chassis IMU660RA is fitted; enable attitude update and heading PID. */
#define CHASSIS_IMU_ENABLE          1U
#define CHASSIS_MOTOR_OUTPUT_ENABLE 1U

/*
 * Radio failsafe window, milliseconds.
 *
 * 200 ms is about 66 missed remote frames (the remote transmits every 3 ms):
 * long enough that an ordinary NRF retry burst never trips it, short enough
 * that the robot stops within its own length at full speed. The gimbal,
 * lightbar and shooter boards all already use 200 ms. The chassis was the only
 * board without a failsafe, and the only board that drives.
 */
#define CHASSIS_NRF_HOLD_MS         200U

/*
 * Validity window for the yaw board's reply frame, milliseconds.
 *
 * 100 ms deliberately equals that board's own COMMS_TIMEOUT_US
 * (08.yaw_4310/Drivers/BSP/Foc/Data.h:49) so both ends of the link give up at
 * the same instant, instead of this end steering on an angle the other end has
 * already disowned.
 */
#define YAW_REPLY_HOLD_MS           100U

/*
 * Retry period once the IMU has been declared unhealthy, milliseconds.
 *
 * A failed read costs the full IIC_HW_TIMEOUT_FAST, and data_get_task is the
 * highest-priority task in the system: retrying every millisecond would spend
 * essentially all the CPU failing, starving both the wheel-stop path and the
 * FOC link. Backing off to 100 ms costs about 5% instead, so an unplugged IMU
 * degrades to "no heading hold" rather than taking the chassis down with it.
 */
#define IMU_RETRY_PERIOD_MS         100U

/*
 * Must stay 0 on any build wired to the yaw FOC board.
 *
 * printf() lands on USART3 (usart.c:39, fputc writes USART3->DR directly) and
 * USART3 *is* the FOC link. Two independent reasons they cannot share it:
 *
 *   - The ASCII is not a 0x55..0xFF frame, so it desynchronises the yaw
 *     board's frame_sync() for the length of the burst. The 110-byte ATT line
 *     was about 9.5 ms out of every 100 ms: a 10% hole in the command stream.
 *   - uart_sand() now drives the link by DMA, and usart3_dma_send() leaves
 *     USART_CR3_DMAT set permanently. A CPU store to USART3->DR then races the
 *     DMA controller for the same register and corrupts both streams. Under
 *     the old blocking HAL_UART_Transmit() both writers were the CPU, so this
 *     was merely untidy; with DMA it is a hardware conflict.
 *
 * No observability is lost. Attitude is already published every cycle to the
 * volatile imu_telem struct (imu.h, IMU_TELEM_MAGIC), which a debugger reads
 * over SWD without halting the CPU -- the same technique 08.yaw_4310 uses for
 * g_yaw_monitor, and for the same reason: the moment the board is in its real
 * wiring is the moment the serial console stops being available.
 * Host side: tools/attitude_view.py.
 */
#define CHASSIS_UART3_DEBUG_ENABLE  0U

/*
 * Independent watchdog (IWDG).
 *
 * Armed immediately before vTaskStartScheduler(), never earlier. Boot spends up
 * to about 900 ms inside imu_gyro_calibrate() (GYRO_CALIB_DISCARD +
 * GYRO_CALIB_MAX_TRY samples at 1 ms each, imu.c:18-20) plus a 128-address
 * iic_scan(); arming before that would make a slow-but-correct boot
 * indistinguishable from a hang and reset-loop the board forever.
 *
 * 500 ms nominal. The IWDG is clocked from the LSI, which the F407 datasheet
 * specifies as 32 kHz typical but 17..47 kHz over the full voltage and
 * temperature range, so the real window is roughly 340..940 ms. That spread does
 * not matter here: the refresh happens every 2 ms in normal operation, so even
 * the 340 ms corner keeps a 170x margin -- while 500 ms is still short enough
 * that a hung chassis has rebooted before the yaw board's 100 ms comms interlock
 * (YAW_REPLY_HOLD_MS) has done anything worse than hold the axis in place.
 */
#define CHASSIS_IWDG_ENABLE         1U
#define CHASSIS_IWDG_PERIOD_MS      500U

/*
 * Stop the IWDG counter while a debugger has the core halted.
 *
 * Without this every SWD breakpoint -- and every `halt` in the openocd scripts
 * that read imu_telem / g_nrf_frames / wheel_speeds out of live RAM, which is
 * now the only observability this board has -- turns into a watchdog reset a few
 * hundred ms later. That both destroys the state being inspected and looks
 * exactly like the 3V3 rail browning out. The bit has no effect unless a
 * debugger owns the core, so leaving it set costs nothing in a match.
 */
#define CHASSIS_IWDG_DEBUG_FREEZE   1U

/*
 * One bit per supervised task.
 *
 * A single HAL_IWDG_Refresh() from one task would only catch total scheduler
 * death. It would not catch the failure that actually matters here: one task
 * stuck forever -- a dead I2C bus, a corrupted vTaskDelayUntil wake time, a
 * priority-7 task spinning -- while the others keep running happily and keep
 * petting the dog. The counter is therefore refreshed only once every
 * supervised task has checked in since the previous refresh, so any single
 * stalled task is enough to trip it.
 *
 * The refresh itself is issued from UART_Trans_task, the lowest priority of the
 * four: a higher-priority task that spins without yielding then starves the
 * refresh as well, which is the correct outcome.
 */
#define IWDG_TASK_IMU        (1U << 0)   /* data_get_task,      1 ms */
#define IWDG_TASK_NRF        (1U << 1)   /* DATA_COMM_task,     2 ms */
#define IWDG_TASK_EXCHANGE   (1U << 2)   /* data_exchange_task, 1 ms */
#define IWDG_TASK_UART       (1U << 3)   /* UART_Trans_task,    1 ms */
#define IWDG_TASK_ALL        (IWDG_TASK_IMU | IWDG_TASK_NRF | \
                              IWDG_TASK_EXCHANGE | IWDG_TASK_UART)

/*
 * Task priorities. Higher number = higher priority (the FreeRTOS convention,
 * which is the opposite of several other RTOSes). The ordering encodes what must
 * not be starved, and it is deliberate rather than incidental:
 *
 *   9  data_get_task       IMU. Highest because everything downstream is built on
 *                          attitude, and because a late sample here is worse than
 *                          a late anything else: process_continuous_angle()
 *                          silently gains a whole turn of error if two samples
 *                          straddle the +-PI branch cut (see THRESHOLD in data.h).
 *   8  DATA_COMM_task      Radio, above its consumer.
 *   7  data_exchange_task  Decode, mix, wheel output.
 *   6  UART_Trans_task     Serial transmit AND the watchdog refresh. LOWEST of the
 *                          four on purpose -- a higher-priority task that spins
 *                          without yielding then starves the refresh as well, so
 *                          the watchdog fires. Put the refresh in the
 *                          highest-priority task instead and that whole class of
 *                          failure becomes invisible.
 *  10  start_task          runs once, creates the others, deletes itself
 *   2  oled_show_task      not created (commented out in start_task)
 *   1  print_task          not created
 *
 * The four supervised tasks occupy 6..9 with no gaps, so nothing unsupervised can
 * sit between them and preempt one without also being watched.
 */
#define START_TASK_PRIO    10  // 启动任务优先级
#define DATA_GET_PRIO      9   // IMU任务（最高）
#define DATA_COMM_PRIO     8   /* NRF radio (second highest) */
#define OLED_SHOW_PRIO     2   /* OLED display (not created) */
#define PRINT_PRIO         1   /* debug print (not created) */
#define DATA_EXCHANGE_PRIO 7   /* data exchange */
#define UART_Trans_PRIO	   6   /* serial transmit + watchdog refresh */

/*
 * Stack sizes, in WORDS not bytes: FreeRTOS multiplies by sizeof(StackType_t),
 * which is 4 on Cortex-M4. So DATA_GET_STK_SIZE 512 means 2 KB, and the five
 * created tasks take 128+512+256+128+256 = 1280 words = 5 KB out of
 * configTOTAL_HEAP_SIZE in FreeRTOSConfig.h.
 *
 * data_get_task gets four times the default because it is the only one that
 * calls into the AHRS: MahonyAHRS plus the quaternion-to-Euler conversion are
 * float-heavy, and at -O0 every intermediate lands on the stack instead of in a
 * register. Lowering the optimisation level raises this requirement -- relevant
 * because this build stays at -O0 on purpose.
 *
 * Overflow is not silent: configCHECK_FOR_STACK_OVERFLOW is enabled, so
 * vApplicationStackOverflowHook() in my_task.c catches it, kills the motor
 * outputs and parks the CPU for the watchdog to reset.
 */
#define START_STK_SIZE     128
#define DATA_GET_STK_SIZE  512
#define DATA_COMM_SIZE     256
#define OLED_SHOW_STK_SIZE 256
#define PRINT_STK_SIZE     256
#define DATA_EXCHANGE_SIZE 128
#define UART_Trans_SIZE	   256


/* Function declarations */
void create_task(void);
void start_task(void *pvParameters);
void print_task(void *pvParameters);
void data_get_task(void *pvParameters);
void DATA_COMM_task(void *pvParameters);
void oled_show_task(void *pvParameters);
void data_exchange_task(void *pvParameters);
void UART_Trans_task(void *pvParameters);
void create_task(void);

#if CHASSIS_IWDG_ENABLE
void iwdg_init(void);
void iwdg_task_checkin(uint32_t task_bit);
void iwdg_kick(void);

/*
 * Boot and margin diagnostics, meant to be read over SWD (they have no printf
 * path -- CHASSIS_UART3_DEBUG_ENABLE is 0 on any build wired to the yaw board).
 *
 *   g_reset_csr       RCC->CSR exactly as found at boot, before RMVF cleared it.
 *                     Bit 29 = IWDG reset, 27 = POR/PDR, 25 = BOR, 26 = NRST
 *                     pin, 28 = software. This is what separates "the watchdog
 *                     bit" from "the rail sagged", which are otherwise identical
 *                     symptoms and both plausible on DAPLink 3V3 power.
 *   g_iwdg_kicks      successful refreshes since boot; frozen == tasks not all
 *                     checking in.
 *   g_iwdg_gap_max_ms worst interval between two successful refreshes. The only
 *                     honest measure of how much margin the 500 ms window really
 *                     has on this build -- read it after a run instead of
 *                     trusting the 2 ms the task periods promise on paper.
 */
extern volatile uint32_t g_reset_csr;
extern volatile uint32_t g_iwdg_kicks;
extern volatile uint32_t g_iwdg_gap_max_ms;
#else
#define iwdg_init()             do {} while (0)
#define iwdg_task_checkin(bit)  do {} while (0)
#define iwdg_kick()             do {} while (0)
#endif

#endif
