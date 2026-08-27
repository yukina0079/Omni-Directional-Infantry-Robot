#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "led.h"
#include "key.h"
#include "malloc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bmi055.h"
#include "ist8310.h"
#include "imu.h"
#include "spi.h"
#include "laser.h"
#include "oled.h"
#include "nrf24l01.h"
#include "pid.h"
#include "lvbo.h"
#include "my_task.h"
#include "iic_hw.h"
#include "data.h"
#include "ws2812b.h"

/* Set to 1 for the standalone PC attitude viewer; set to 0 for normal control. */
#define IMU_GRAPH_TEST 0

/*
 * Gimbal main control board -- STM32F407VET6.
 *
 * Receives the operator frame over nRF24L01 and relays a pitch command to the
 * PITCH 4310 FOC board over USART3. Drives no motor directly. Yaw belongs to the
 * chassis board; this one owns pitch, the laser and the WS2812 strip.
 *
 * BOOT ORDER:
 *
 *   sys_stm32_clock_init(336, 8, 2, 7)
 *       HSE 8 MHz / 8 = 1 MHz, x336 = 336 MHz, /2 = 168 MHz SYSCLK. PLLQ = 7
 *       gives the 48 MHz USB/SDIO clock, unused but required to be legal.
 *       Identical to the chassis, so the two boards share one baud divisor and
 *       one 1 ms SysTick.
 *
 *   usart_init(115200)
 *       Brings up BOTH ports: USART3 for the binary FOC frames, and USART2 as
 *       the printf console (usart.c calls usart2_log_init() first). Having two
 *       is what lets data_print() run permanently here -- the chassis, with only
 *       USART3, has to keep its debug output switched off.
 *
 *   spi2_init()  before  nrf24l01_init()   -- the radio is on SPI2.
 *   iic_init()   before  BMI055_init()     -- the IMU is on I2C.
 *   BMI055_init() before imu_init()        -- the AHRS seeds its quaternion from
 *                                             a first accelerometer reading, so
 *                                             the sensor must already answer.
 *   data_init()  before  create_task()     -- sets up pitch_pos_pid and stamps
 *                                             the frame header/tail. A task
 *                                             running first would call PID_calc()
 *                                             on a zeroed struct: Kp = 0, so it
 *                                             would quietly command nothing.
 *
 * IMU_GRAPH_TEST is a standalone bench mode: it never starts the scheduler and
 * never brings up the radio, just streams attitude to the PC at 50 Hz for
 * plotting. Useful for judging whether the AHRS is converging before any of the
 * control path is involved. Must be 0 for a normal build.
 *
 * Note IMU_HAS_MAGNETOMETER gates ist8310_init(). It defaults to 0 in imu.h, so
 * the AHRS runs six-axis and yaw is free to drift -- which does not matter on this
 * board, because only pitch is used and pitch is gravity-referenced by the
 * accelerometer.
 *
 * As on the chassis there is no while(1) after vTaskStartScheduler(); control
 * falls off the end of main() if the idle task cannot be allocated.
 */
int main(void)
{
    HAL_Init();                         /* 初始化HAL库 */
	sys_stm32_clock_init(336, 8, 2, 7); /* 设置时钟,168Mhz */
    delay_init(168);                    /* 延时初始化 */
    usart_init(115200);                 /* 串口初始化为115200 */
    led_init();                         /* 初始化LED */
    key_init();                         /* 初始化按键 */	
	spi2_init();
	iic_init();
	BMI055_init();
	imu_init();

#if IMU_GRAPH_TEST
	{
		uint32_t sample_count = 0;
		printf("IMU_GRAPH_READY\r\n");
		while (1)
		{
			imu_updata();
			if (++sample_count >= 20)
			{
				printf("ATT %.3f %.3f %.3f ACC %.3f %.3f %.3f GYR %.3f %.3f %.3f TMP %.2f\r\n",
				       INS_angle[0] * 57.2957795f,
				       INS_angle[1] * 57.2957795f,
				       INS_angle[2] * 57.2957795f,
				       accel[0], accel[1], accel[2],
				       gyro[0], gyro[1], gyro[2], temp[0]);
				sample_count = 0;
			}
			delay_ms(1);
		}
	}
#else
	nrf24l01_init();
#if IMU_HAS_MAGNETOMETER
	ist8310_init();
#endif
	NRF_check();
	data_init();

	printf("runing......\r\n");
	
    create_task();
	vTaskStartScheduler();
#endif

}
