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

	iwdg_init();

	vTaskStartScheduler();

}
