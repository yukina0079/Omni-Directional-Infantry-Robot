#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "led.h"
#include "key.h"
#include "malloc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "imu660ra.h"
#include "ist8310.h"
#include "imu.h"
#include "spi.h"
#include "oled.h"
#include "nrf24l01.h"
#include "pid.h"
#include "my_task.h"
#include "iic_hw.h"
#include "data.h"
#include "motor.h"

/*
 * Chassis main control board -- STM32F407VET6.
 *
 * Role in the robot: receives the operator's frame from the remote over
 * nRF24L01, drives four brushed omni wheels through two TB6612FNG H-bridges,
 * and relays the frame (annotated with its own IMU heading and a yaw command) to
 * the yaw FOC board over USART3. It is one of four independent receivers on the
 * same radio address -- gimbal, shooter and referee/lightbar boards get the same
 * broadcast -- so nothing here is a master of anything except the wheels.
 *
 * BOOT ORDER. Almost every line below has to come where it is:
 *
 *   sys_stm32_clock_init(336, 8, 2, 7)
 *       PLLN=336, PLLM=8, PLLP=2, PLLQ=7. HSE 8 MHz / 8 = 1 MHz, x336 = 336 MHz,
 *       /2 = 168 MHz SYSCLK -- the part's maximum. PLLQ=7 gives 336/7 = 48 MHz
 *       for the USB/SDIO clock, unused here but required to be legal. Everything
 *       downstream derives from this: the 10 kHz motor PWM, the 115200 baud
 *       divisor, and the 1 ms SysTick that both HAL and FreeRTOS run on.
 *
 *   delay_init(168)  before anything that busy-waits.
 *
 *   usart_init(115200)  brings up USART3, which is BOTH the FOC link and the
 *       printf target. The boot messages below therefore go out on the wire the
 *       yaw board is listening to. Harmless: they are not 0x55..0xFF frames, so
 *       that board's frame_sync() simply finds nothing until real frames start.
 *       It stops being harmless once the TX DMA is running, which is why
 *       CHASSIS_UART3_DEBUG_ENABLE must stay 0 (see my_task.h).
 *
 *   motor_init()  before  pwm_init()
 *       Direction pins get a defined level before the timer can gate any current
 *       through them. pwm_init() then starts all four channels with a compare
 *       value of 0, so the bridges are in short brake from here until the first
 *       MOTOR_PWM_UPDATE() -- never floating, never driving.
 *
 *   spi2_init()  before  nrf24l01_init()  -- the radio hangs off SPI2.
 *   iic_init()   before  IMU660RA_init()  -- the IMU hangs off I2C2.
 *
 *   data_init()  before  create_task()
 *       Sets up both PID structs. A task that ran before this would call
 *       PID_calc() on a zeroed pid_type_def: Kp = 0, so it would silently
 *       command nothing rather than crash, which is the kind of bug that
 *       presents as "the gimbal does not follow" three days later.
 *
 *   iwdg_init()  LAST, immediately before the scheduler.
 *       Boot is data-dependent in length -- iic_scan() walks 128 addresses and
 *       imu_gyro_calibrate() can spend most of a second waiting for the board to
 *       be held still -- so arming earlier would make a slow-but-correct boot
 *       indistinguishable from a hang, and reset-loop the board forever.
 *
 * Note there is no while(1) after vTaskStartScheduler(). That call does not
 * return unless the idle task cannot be created (heap exhaustion), in which case
 * control falls off the end of main() -- into whatever the C runtime does next,
 * with the motor outputs still in whatever state they were left. It has never
 * happened here because heap use is static and known, but a trailing
 * for(;;) MOTOR_PWM_UPDATE(0,0,0,0); would be the honest belt and braces.
 */
int main(void)
{
#if CHASSIS_IMU_ENABLE
    IMU660RA_Status imu_status;
#endif

    HAL_Init();                         
	sys_stm32_clock_init(336, 8, 2, 7); 
    delay_init(168);                    
    usart_init(115200);                 
    led_init(); 

	/* LED3 on: reached the end of low-level init. Left on deliberately -- it is
	 * also what a watchdog reset-loop looks like, blinking at the reboot rate. */
	LED3(1);
	
    key_init();                         
	motor_init();               
	pwm_init(16799, 0);
	spi2_init();
#if CHASSIS_IMU_ENABLE
	iic_init();
	printf("\r\nchassis IMU I2C scan starting...\r\n");
	iic_scan();
#endif
	nrf24l01_init();
	
#if CHASSIS_IMU_ENABLE
	imu_status = IMU660RA_init();
	if (imu_status != IMU660RA_OK)
	{
		printf("IMU660RA init failed, status=%d chip=0x%02X addr=0x%02X\r\n",
		       (int)imu_status, IMU660RA_get_chip_id(), IMU660RA_get_i2c_addr());
		printf("hint: 0x24=BMI270 ok, 0xFF=no ACK, 0x00=APS dummy, 0x1E=BMI088 accel, 0xFA=BMI055 accel\r\n");
	}
	else
	{
		printf("IMU660RA init ok, chip=0x%02X addr=0x%02X\r\n",
		       IMU660RA_get_chip_id(), IMU660RA_get_i2c_addr());
	}
	imu_init();
#endif
	
	NRF_check();
	data_init();


	create_task();

	/*
	 * Arm the watchdog last. Everything above is data-dependent in length:
	 * iic_scan() walks 128 addresses and imu_gyro_calibrate() can burn 880
	 * sample periods waiting for the board to be held still, so a correct
	 * cold boot legitimately takes most of a second. Under a watchdog that
	 * was already running, a slow boot and a hung boot would be the same
	 * event. From here on the four supervised tasks refresh it; see
	 * IWDG_TASK_ALL in my_task.h.
	 */
	iwdg_init();

	vTaskStartScheduler();

}
