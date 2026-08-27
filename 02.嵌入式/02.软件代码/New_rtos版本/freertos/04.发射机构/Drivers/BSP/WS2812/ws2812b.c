#include "ws2812b.h"
#include "stdint.h"

/* 句柄定义修改为TIM4 */
DMA_HandleTypeDef tim4_ch2 = {0};
TIM_HandleTypeDef htim4 = {0};	

#define code0 30
#define code1 60
#define codeReset 0
#define LED_COUNT 9

uint8_t color[LED_COUNT][3];

void WS2812_Set(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
	color[index][0] = r;
	color[index][1] = g; 
	color[index][2] = b;
}

void WS2812_SetALL(uint8_t r, uint8_t g, uint8_t b)
{
	for(uint8_t i = 0; i < LED_COUNT; i++){
		WS2812_Set(i, r, g, b);
	}
}

void WS2812_Updata(void)
{
	static uint16_t data[LED_COUNT * 3 * 8 + 1];
	
	for(uint8_t i = 0; i < LED_COUNT; i++){
		uint8_t r = color[i][0];
		uint8_t g = color[i][1];
		uint8_t b = color[i][2];
		
		for(uint8_t j = 0; j < 8; j++){
			data[24 * i + j] = (g & (0x80 >> j)) ? code1 : code0;
			data[24 * i + 8 + j] = (r & (0x80 >> j)) ? code1 : code0;
			data[24 * i + 16 + j] = (b & (0x80 >> j)) ? code1 : code0;
		}		
	}
	data[LED_COUNT * 24] = codeReset;
	
	/* 改为TIM4_CH2 */
	HAL_TIM_PWM_Stop_DMA(&htim4, TIM_CHANNEL_2);  
	__HAL_TIM_SetCounter(&htim4, 0);
	HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_2, (uint32_t*)data, sizeof(data)/sizeof(uint16_t));
}

/* 基础流水灯效果（保持不变） */
void WS2812_FlowLight(uint8_t r, uint8_t g, uint8_t b, uint16_t delay_ms, uint8_t direction)
{
    if(direction == 0) {
        for(uint8_t i = 1; i < LED_COUNT; i++) {
            WS2812_SetALL(0, 0, 0);
			WS2812_Set(0, 140, 0, 180);
            WS2812_Set(i, r, g, b);
            WS2812_Updata();
            HAL_Delay(delay_ms);
        }
    }
    else if(direction == 1) {
        for(int8_t i = LED_COUNT - 1; i >= 0; i--) {
            WS2812_SetALL(0, 0, 0);
            WS2812_Set(i, r, g, b);
            WS2812_Updata();
            HAL_Delay(delay_ms);
        }
    }
    else if(direction == 2) {
        uint8_t mid = LED_COUNT / 2;
        for(uint8_t offset = 0; offset <= mid; offset++) {
            WS2812_SetALL(0, 0, 0);
            if(mid + offset < LED_COUNT) WS2812_Set(mid + offset, r, g, b);
            if(mid - offset >= 0) WS2812_Set(mid - offset, r, g, b);
            WS2812_Updata();
            HAL_Delay(delay_ms);
        }
    }
}

/* TIM4初始化函数 */
void ws2812_init(uint16_t arr, uint16_t psc)
{
	TIM_OC_InitTypeDef htim4_config = {0};
	
	htim4.Instance = TIM4;					
	htim4.Init.Prescaler = psc;
	htim4.Init.Period = arr;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;		
	HAL_TIM_PWM_Init(&htim4);
	
	htim4_config.OCMode = TIM_OCMODE_PWM1;
	htim4_config.Pulse = 0;
	htim4_config.OCPolarity = TIM_OCPOLARITY_HIGH;
	htim4_config.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim4, &htim4_config, TIM_CHANNEL_2);  // 改为CH2
}

/* DMA1_Channel3初始化（对应TIM4_CH2） */
void dma_init(void)
{
	__HAL_RCC_DMA1_CLK_ENABLE();
	
	tim4_ch2.Instance = DMA1_Channel4;  // TIM4_CH2对应DMA1_Channel3
	tim4_ch2.Init.Direction = DMA_MEMORY_TO_PERIPH;
	tim4_ch2.Init.PeriphInc = DMA_PINC_DISABLE;
	tim4_ch2.Init.MemInc = DMA_MINC_ENABLE;
	tim4_ch2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	tim4_ch2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	tim4_ch2.Init.Mode = DMA_NORMAL;
	tim4_ch2.Init.Priority = DMA_PRIORITY_LOW;									
											
	HAL_DMA_Init(&tim4_ch2);

	__HAL_LINKDMA(&htim4, hdma[TIM_DMA_ID_CC2], tim4_ch2);  // 链接TIM4_CH2
}

