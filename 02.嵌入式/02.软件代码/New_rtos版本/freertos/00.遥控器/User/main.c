#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "my_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "data.h"
#include "adc.h"
#include "encoder.h"
#include "oled.h"
#include "nrf24L01.h"

int main(void)
{
    HAL_Init();                         
    sys_stm32_clock_init(RCC_PLL_MUL9); 
    delay_init(72);                     
    usart_init(115200);                 
    led_init();                         
    adc_init();
	adc_dma_init((uint32_t *)&adc_value);
	encoder_init();
	//oled_init();
	NRF24L01_Init();
	NRF_check();
	printf("runing........r\n");

	LED1(1);
	create_task();
	vTaskStartScheduler();
}
