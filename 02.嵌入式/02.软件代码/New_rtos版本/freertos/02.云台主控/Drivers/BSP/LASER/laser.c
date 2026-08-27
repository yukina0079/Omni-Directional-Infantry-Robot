#include "laser.h"

void laser_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    
    LASER1_GPIO_CLK_ENABLE();                                 
                              
    gpio_init_struct.Pin = LASER1_GPIO_PIN|LASER2_GPIO_PIN;     
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;            
    gpio_init_struct.Pull = GPIO_PULLUP;                    
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;          
    HAL_GPIO_Init(LASER1_GPIO_PORT, &gpio_init_struct);  
	
	LASER1(1);
	LASER2(1);
}
