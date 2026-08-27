#include "ws2812b.h"
#include "stdint.h"

DMA_HandleTypeDef tim1_ch4 = {0};
DMA_HandleTypeDef tim2_ch3 = {0};
DMA_HandleTypeDef tim2_ch4 = {0};
DMA_HandleTypeDef tim3_ch3 = {0};
DMA_HandleTypeDef tim3_ch4 = {0};
TIM_HandleTypeDef htim1 = {0};
TIM_HandleTypeDef htim2 = {0};
TIM_HandleTypeDef htim3 = {0};	

#define code0 30
#define code1 60
#define codeReset 0
#define LED_COUNT       WS2812_LED_COUNT
#define STRIP_COUNT     WS2812_STRIP_COUNT
#define WS_BUF_LEN      (LED_COUNT * 24 + 1)

static uint8_t color[STRIP_COUNT][LED_COUNT][3];

void WS2812_SetStrip(uint8_t strip, uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
	if ((strip >= STRIP_COUNT) || (index >= LED_COUNT)) {
		return;
	}
	color[strip][index][0] = r;
	color[strip][index][1] = g;
	color[strip][index][2] = b;
}

void WS2812_FillStrip(uint8_t strip, uint8_t r, uint8_t g, uint8_t b)
{
	uint8_t i;
	for (i = 0; i < LED_COUNT; i++) {
		WS2812_SetStrip(strip, i, r, g, b);
	}
}

void WS2812_Set(uint8_t index,uint8_t r,uint8_t g,uint8_t b)
{
	uint8_t s;
	for (s = 0; s < STRIP_COUNT; s++) {
		WS2812_SetStrip(s, index, r, g, b);
	}
}
void WS2812_SetALL(uint8_t r,uint8_t g,uint8_t b)
{
	uint8_t s;
	for (s = 0; s < STRIP_COUNT; s++) {
		WS2812_FillStrip(s, r, g, b);
	}
}

static void ws2812_encode_strip(uint8_t strip, uint16_t *data)
{
	uint8_t i;
	uint8_t j;
	for (i = 0; i < LED_COUNT; i++) {
		uint8_t r = color[strip][i][0];
		uint8_t g = color[strip][i][1];
		uint8_t b = color[strip][i][2];
		for (j = 0; j < 8; j++) {
			data[24 * i + j]      = (g & (0x80 >> j)) ? code1 : code0;
			data[24 * i + 8 + j]  = (r & (0x80 >> j)) ? code1 : code0;
			data[24 * i + 16 + j] = (b & (0x80 >> j)) ? code1 : code0;
		}
	}
	data[LED_COUNT * 24] = codeReset;
}

void WS2812_Updata(void)
{
	static uint16_t data[STRIP_COUNT][WS_BUF_LEN];
	uint8_t s;

	for (s = 0; s < STRIP_COUNT; s++) {
		ws2812_encode_strip(s, data[s]);
	}

	HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_4);

	__HAL_TIM_SetCounter(&htim1,0);
	__HAL_TIM_SetCounter(&htim2,0);
	__HAL_TIM_SetCounter(&htim3,0);

	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3,(uint32_t*)data[0], WS_BUF_LEN);
	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4,(uint32_t*)data[1], WS_BUF_LEN);
	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_4,(uint32_t*)data[2], WS_BUF_LEN);
	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3,(uint32_t*)data[3], WS_BUF_LEN);
	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_4,(uint32_t*)data[4], WS_BUF_LEN);
}
// 基础流水灯效果
void WS2812_FlowLight(uint8_t r, uint8_t g, uint8_t b, uint16_t delay_ms, uint8_t direction)
{
    // direction: 0=从左到右, 1=从右到左, 2=从中间到两边, 3=从两边到中间
    
    if(direction == 0) {
        // 从左到右
        for(uint8_t i = 1; i < LED_COUNT; i++) {
            WS2812_SetALL(0, 0, 0);  // 全部关闭
			WS2812_Set(0,140, 0, 180);
            WS2812_Set(i, r, g, b);  // 点亮当前灯珠
            WS2812_Updata();
            HAL_Delay(delay_ms);
        }
    }
    else if(direction == 1) {
        // 从右到左
        for(int8_t i = LED_COUNT - 1; i >= 0; i--) {
            WS2812_SetALL(0, 0, 0);
            WS2812_Set(i, r, g, b);
            WS2812_Updata();
            HAL_Delay(delay_ms);
        }
    }
    else if(direction == 2) {
        // 从中间到两边
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
void pwm_init(uint16_t arr,uint16_t psc)
{
		TIM_OC_InitTypeDef htim1_config = {0};
		TIM_OC_InitTypeDef htim2_config = {0};
		TIM_OC_InitTypeDef htim3_config = {0};
		
		htim1.Instance = TIM1;					
		htim1.Init.Prescaler = psc;
		htim1.Init.Period = arr;
		htim1.Init.CounterMode = TIM_COUNTERMODE_UP;//向上计数
		htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;		
		HAL_TIM_PWM_Init(&htim1);
		
		htim1_config.OCMode = TIM_OCMODE_PWM1; //通用定时器
		htim1_config.Pulse  = 0;
		htim1_config.OCPolarity = TIM_OCPOLARITY_HIGH;//计数方式
		htim1_config.OCFastMode = TIM_OCFAST_DISABLE;
		HAL_TIM_PWM_ConfigChannel(&htim1,&htim1_config,TIM_CHANNEL_4);

		htim2.Instance = TIM2;					
		htim2.Init.Prescaler = psc;
		htim2.Init.Period = arr;
		htim2.Init.CounterMode = TIM_COUNTERMODE_UP;//向上计数
		htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;		
		HAL_TIM_PWM_Init(&htim2);
		
		htim2_config.OCMode = TIM_OCMODE_PWM1; //通用定时器
		htim2_config.Pulse  = 0;
		htim2_config.OCPolarity = TIM_OCPOLARITY_HIGH;//计数方式
		htim2_config.OCFastMode = TIM_OCFAST_DISABLE;
		HAL_TIM_PWM_ConfigChannel(&htim2,&htim2_config,TIM_CHANNEL_3);
		HAL_TIM_PWM_ConfigChannel(&htim2,&htim2_config,TIM_CHANNEL_4);
		
		htim3.Instance = TIM3;					
		htim3.Init.Prescaler = psc;
		htim3.Init.Period = arr;
		htim3.Init.CounterMode = TIM_COUNTERMODE_UP;//向上计数
		htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;		
		HAL_TIM_PWM_Init(&htim3);
		
		htim3_config.OCMode = TIM_OCMODE_PWM1; //通用定时器
		htim3_config.Pulse  = 0;
		htim3_config.OCPolarity = TIM_OCPOLARITY_HIGH;//计数方式
		htim3_config.OCFastMode = TIM_OCFAST_DISABLE;
		HAL_TIM_PWM_ConfigChannel(&htim3,&htim3_config,TIM_CHANNEL_3);
		HAL_TIM_PWM_ConfigChannel(&htim3,&htim3_config,TIM_CHANNEL_4);
}
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)		//PWM硬件参数
{
	if(htim->Instance == TIM1){
		GPIO_InitTypeDef gpio_initstruct;//定义结构体参数
			
		__HAL_RCC_TIM1_CLK_ENABLE();		//定时器4时钟使能	
		__HAL_RCC_GPIOA_CLK_ENABLE();		//开启GPIOB组引脚
	
		gpio_initstruct.Pin = GPIO_PIN_11;//配置引脚号
		gpio_initstruct.Mode = GPIO_MODE_AF_PP;//配置工作模式 复用推挽输出
		gpio_initstruct.Pull = GPIO_PULLUP;//配置上拉下拉
		gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;//配置输出速度
	
		HAL_GPIO_Init(GPIOA,&gpio_initstruct);		
	}
	if(htim->Instance == TIM2){
		GPIO_InitTypeDef gpio_initstruct;//定义结构体参数
			
		__HAL_RCC_TIM2_CLK_ENABLE();		//定时器4时钟使能	
		__HAL_RCC_GPIOA_CLK_ENABLE();		//开启GPIOB组引脚
	
		gpio_initstruct.Pin = GPIO_PIN_2|GPIO_PIN_3;//配置引脚号
		gpio_initstruct.Mode = GPIO_MODE_AF_PP;//配置工作模式 复用推挽输出
		gpio_initstruct.Pull = GPIO_PULLUP;//配置上拉下拉
		gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;//配置输出速度
	
		HAL_GPIO_Init(GPIOA,&gpio_initstruct);		
	}
	if(htim->Instance == TIM3){
		GPIO_InitTypeDef gpio_initstruct;//定义结构体参数
			
		__HAL_RCC_TIM3_CLK_ENABLE();		//定时器4时钟使能	
		__HAL_RCC_GPIOB_CLK_ENABLE();		//开启GPIOB组引脚
	
		gpio_initstruct.Pin = GPIO_PIN_0|GPIO_PIN_1;//配置引脚号
		gpio_initstruct.Mode = GPIO_MODE_AF_PP;//配置工作模式 复用推挽输出
		gpio_initstruct.Pull = GPIO_PULLUP;//配置上拉下拉
		gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;//配置输出速度
	
		HAL_GPIO_Init(GPIOB,&gpio_initstruct);		
	}
}
void dma_init(void)
{
		__HAL_RCC_DMA1_CLK_ENABLE();
	
		tim1_ch4.Instance = DMA1_Channel4;
		tim1_ch4.Init.Direction = DMA_MEMORY_TO_PERIPH;
		tim1_ch4.Init.PeriphInc = DMA_PINC_DISABLE;
		tim1_ch4.Init.MemInc = DMA_MINC_ENABLE;
		tim1_ch4.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
		tim1_ch4.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
		tim1_ch4.Init.Mode = DMA_NORMAL;
		tim1_ch4.Init.Priority = DMA_PRIORITY_LOW;											

		tim2_ch3.Instance = DMA1_Channel1;
		tim2_ch3.Init.Direction = DMA_MEMORY_TO_PERIPH;
		tim2_ch3.Init.PeriphInc = DMA_PINC_DISABLE;
		tim2_ch3.Init.MemInc = DMA_MINC_ENABLE;
		tim2_ch3.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
		tim2_ch3.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
		tim2_ch3.Init.Mode = DMA_NORMAL;
		tim2_ch3.Init.Priority = DMA_PRIORITY_LOW;											

		tim2_ch4.Instance = DMA1_Channel7;
		tim2_ch4.Init.Direction = DMA_MEMORY_TO_PERIPH;
		tim2_ch4.Init.PeriphInc = DMA_PINC_DISABLE;
		tim2_ch4.Init.MemInc = DMA_MINC_ENABLE;
		tim2_ch4.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
		tim2_ch4.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
		tim2_ch4.Init.Mode = DMA_NORMAL;
		tim2_ch4.Init.Priority = DMA_PRIORITY_LOW;											

		tim3_ch3.Instance = DMA1_Channel2;
		tim3_ch3.Init.Direction = DMA_MEMORY_TO_PERIPH;
		tim3_ch3.Init.PeriphInc = DMA_PINC_DISABLE;
		tim3_ch3.Init.MemInc = DMA_MINC_ENABLE;
		tim3_ch3.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
		tim3_ch3.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
		tim3_ch3.Init.Mode = DMA_NORMAL;
		tim3_ch3.Init.Priority = DMA_PRIORITY_LOW;											

		tim3_ch4.Instance = DMA1_Channel3;
		tim3_ch4.Init.Direction = DMA_MEMORY_TO_PERIPH;
		tim3_ch4.Init.PeriphInc = DMA_PINC_DISABLE;
		tim3_ch4.Init.MemInc = DMA_MINC_ENABLE;
		tim3_ch4.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
		tim3_ch4.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
		tim3_ch4.Init.Mode = DMA_NORMAL;
		tim3_ch4.Init.Priority = DMA_PRIORITY_LOW;											
	
		HAL_DMA_Init(&tim1_ch4);
		HAL_DMA_Init(&tim2_ch3);
		HAL_DMA_Init(&tim2_ch4);
		HAL_DMA_Init(&tim3_ch3);
		HAL_DMA_Init(&tim3_ch4);
		
		__HAL_LINKDMA(&htim1,hdma[TIM_DMA_ID_CC4],tim1_ch4);
		__HAL_LINKDMA(&htim2,hdma[TIM_DMA_ID_CC3],tim2_ch3);
		__HAL_LINKDMA(&htim2,hdma[TIM_DMA_ID_CC4],tim2_ch4);
		__HAL_LINKDMA(&htim3,hdma[TIM_DMA_ID_CC3],tim3_ch3);
		__HAL_LINKDMA(&htim3,hdma[TIM_DMA_ID_CC4],tim3_ch4);
}
