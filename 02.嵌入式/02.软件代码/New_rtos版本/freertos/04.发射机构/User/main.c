#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "motor.h"
#include "Biu.h"
#include "nrf24L01.h"
#include "control.h"
#include "ws2812b.h"
#include "FreeRTOS.h"
#include "task.h"
#include "my_task.h"

int main(void)
{
    HAL_Init();                        
    sys_stm32_clock_init(RCC_PLL_MUL9);
    delay_init(72);                    
    usart_init(115200);  
	printf("runing......");
    led_init();  
	LED1(1);
	LASER(0);
	key_init();
	motor_init();
	dma_init();
	NRF24L01_Init();
    ws2812_init(90-1,0); 
	pwm_init(1440,1);
	Biu_pwm_init(19999,71);

	usart_init(115200);
	
	printf("running......\r\n");
     
	WS2812_SetALL(255,0,0);
	WS2812_Updata();

	Biu_pwm_compare_set(1000);
	delay_ms(3000);
	NRF24L01_find();
	
    create_task();
	vTaskStartScheduler();
}
