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

#define IMU_GRAPH_TEST 0

int main(void)
{
    HAL_Init();
	sys_stm32_clock_init(336, 8, 2, 7);
    delay_init(168);
    usart_init(115200);
    led_init();
    key_init();
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
