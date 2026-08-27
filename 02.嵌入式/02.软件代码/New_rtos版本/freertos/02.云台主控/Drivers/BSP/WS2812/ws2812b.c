#include "ws2812b.h"
#include "stdint.h"

DMA_HandleTypeDef htim3_dma = {0};
TIM_HandleTypeDef htim3 = {0};

#define code0      30
#define code1      60
#define codeReset   0
#define LED_COUNT  9


uint8_t color[LED_COUNT][3];


void WS2812_Set(uint8_t index,uint8_t r,uint8_t g,uint8_t b)
{
	color[index][0] = r;
	color[index][1] = g; 
	color[index][2] = b;
}
void WS2812_SetALL(uint8_t r,uint8_t g,uint8_t b)
{
	for(uint8_t i = 0;i<LED_COUNT;i++){
		WS2812_Set(i,r,g,b);
	}
}

void WS2812_Updata(void)
{
	static uint16_t data[LED_COUNT * 3 * 8 + 1];
	
	for(uint8_t i = 0;i<LED_COUNT;i++){
		uint8_t r = color[i][0];
		uint8_t g = color[i][1];
		uint8_t b = color[i][2];
		
		for(uint8_t j = 0;j<8;j++){
			data[24 * i + j] = (g & (0x80 >> j)) ? code1 : code0;
			data[24 * i + 8 + j] = (r & (0x80 >> j)) ? code1 : code0;
			data[24 * i + 16 + j] = (b & (0x80 >> j)) ? code1 : code0;
		}		
	}
	data[LED_COUNT * 24] = codeReset;

    // ·¢ËÍ DMA
    HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_4);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4,(uint32_t*) data,sizeof(data)/sizeof(uint16_t));
}

void pwm_init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 104;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim3);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM3) {
        GPIO_InitTypeDef GPIO_InitStruct = {0};

        __HAL_RCC_TIM3_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    }
}

void dma_init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    htim3_dma.Instance = DMA1_Stream2;
    htim3_dma.Init.Channel = DMA_CHANNEL_5;
    htim3_dma.Init.Direction = DMA_MEMORY_TO_PERIPH;
    htim3_dma.Init.PeriphInc = DMA_PINC_DISABLE;
    htim3_dma.Init.MemInc = DMA_MINC_ENABLE;
    htim3_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    htim3_dma.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    htim3_dma.Init.Mode = DMA_NORMAL;
    htim3_dma.Init.Priority = DMA_PRIORITY_HIGH;
    htim3_dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    HAL_DMA_Init(&htim3_dma);
    __HAL_LINKDMA(&htim3, hdma[TIM_DMA_ID_CC4], htim3_dma);
}
