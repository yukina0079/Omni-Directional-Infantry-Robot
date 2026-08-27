#include "ws2812b.h"
#include "stdint.h"
#include "main.h"  // HAL库必须包含

// F407 正确 DMA 句柄
DMA_HandleTypeDef htim3_dma = {0};
TIM_HandleTypeDef htim3 = {0};	

// WS2812 时序参数
#define code0 30
#define code1 60
#define codeReset 0
#define LED_COUNT 8

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
	
	// 停止 DMA 并重启
	HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_4);
	__HAL_TIM_SetCounter(&htim3,0);
	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4,(uint32_t*)data, LED_COUNT*24+1);
}

// 流水灯效果
void WS2812_FlowLight(uint8_t r, uint8_t g, uint8_t b, uint16_t delay_ms, uint8_t direction)
{
    if(direction == 0) {
        for(uint8_t i = 0; i < LED_COUNT; i++) {
            WS2812_SetALL(0, 0, 0);
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

// PWM 初始化（只使用 TIM3_CH4）
void pwm_init(uint16_t arr,uint16_t psc)
{
	TIM_OC_InitTypeDef htim3_config = {0};
	
	htim3.Instance = TIM3;					
	htim3.Init.Prescaler = psc;
	htim3.Init.Period = arr;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;		
	
	HAL_TIM_PWM_Init(&htim3);
	
	// 只配置 CH4
	htim3_config.OCMode = TIM_OCMODE_PWM1;
	htim3_config.Pulse  = 0;
	htim3_config.OCPolarity = TIM_OCPOLARITY_HIGH;
	htim3_config.OCFastMode = TIM_OCFAST_DISABLE;
	
	HAL_TIM_PWM_ConfigChannel(&htim3,&htim3_config,TIM_CHANNEL_4);
}

// 底层硬件初始化（时钟 + GPIO）
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3){
		GPIO_InitTypeDef gpio_initstruct;
		
		__HAL_RCC_TIM3_CLK_ENABLE();
		__HAL_RCC_GPIOC_CLK_ENABLE();
	
		// PC9 → TIM3_CH4 复用推挽输出
		gpio_initstruct.Pin = GPIO_PIN_9;
		gpio_initstruct.Mode = GPIO_MODE_AF_PP;
		gpio_initstruct.Pull = GPIO_PULLUP;
		gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;
		gpio_initstruct.Alternate = GPIO_AF2_TIM3;  // F4 必须配置复用映射
	
		HAL_GPIO_Init(GPIOC, &gpio_initstruct);  // 正确端口：GPIOC
	}
}

// DMA 初始化（F407 正确配置）
void dma_init(void)
{
	__HAL_RCC_DMA1_CLK_ENABLE();
	
	// TIM3_CH4 硬件固定：DMA1_Stream2 / Channel5
	htim3_dma.Instance = DMA1_Stream2;
	htim3_dma.Init.Channel = DMA_CHANNEL_5;
	htim3_dma.Init.Direction = DMA_MEMORY_TO_PERIPH;
	htim3_dma.Init.PeriphInc = DMA_PINC_DISABLE;
	htim3_dma.Init.MemInc = DMA_MINC_ENABLE;
	htim3_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	htim3_dma.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	htim3_dma.Init.Mode = DMA_NORMAL;
	htim3_dma.Init.Priority = DMA_PRIORITY_MEDIUM;
	htim3_dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	
	HAL_DMA_Init(&htim3_dma);
	
	// 关联 DMA 到 TIM3_CH4
	__HAL_LINKDMA(&htim3, hdma[TIM_DMA_ID_CC4], htim3_dma);
}
