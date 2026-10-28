#include "adc.h"
#include "sys.h"

ADC_HandleTypeDef adc_handle = {0};
DMA_HandleTypeDef dma_handle = {0};

void adc_init(void)
{
		adc_handle.Instance = ADC1;
		adc_handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
		adc_handle.Init.ScanConvMode = ADC_SCAN_DISABLE;
		adc_handle.Init.ContinuousConvMode = DISABLE;
		adc_handle.Init.NbrOfConversion = 4;
		adc_handle.Init.DiscontinuousConvMode = DISABLE;
		adc_handle.Init.NbrOfDiscConversion = 0;
 		adc_handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;

		HAL_ADC_Init(&adc_handle);

		HAL_ADCEx_Calibration_Start(&adc_handle);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
		if(hadc->Instance == ADC1){

			GPIO_InitTypeDef gpio_initstruct;
			RCC_PeriphCLKInitTypeDef adc_clk_init = {0};

			__HAL_RCC_ADC1_CLK_ENABLE();
			__HAL_RCC_GPIOA_CLK_ENABLE();

			gpio_initstruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
			gpio_initstruct.Mode = GPIO_MODE_ANALOG;

			HAL_GPIO_Init(GPIOA,&gpio_initstruct);

			adc_clk_init.PeriphClockSelection = RCC_PERIPHCLK_ADC;
			adc_clk_init.AdcClockSelection = RCC_ADCPCLK2_DIV6;
			HAL_RCCEx_PeriphCLKConfig(&adc_clk_init);
		}

}
void adc_channel_config(ADC_HandleTypeDef* hadc, uint32_t ch, uint32_t rank, uint32_t stime)
{
			ADC_ChannelConfTypeDef adc_ch_config = {0};

			adc_ch_config.Channel = ch;
			adc_ch_config.Rank = rank;
			adc_ch_config.SamplingTime  = stime;

			HAL_ADC_ConfigChannel(&adc_handle,&adc_ch_config);
}

uint32_t adc_get_result(uint32_t ch)
{
		adc_channel_config(&adc_handle,ch,ADC_REGULAR_RANK_1,ADC_SAMPLETIME_239CYCLES_5);

		HAL_ADC_Start(&adc_handle);

		HAL_ADC_PollForConversion(&adc_handle,10);

		HAL_ADC_GetValue(&adc_handle);
		return (uint16_t)HAL_ADC_GetValue(&adc_handle);
}

void adc_dma_set(void)
{
	adc_handle.Instance = ADC1;
	adc_handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;

	adc_handle.Init.ScanConvMode = ADC_SCAN_ENABLE;
	adc_handle.Init.ContinuousConvMode = ENABLE;
	adc_handle.Init.NbrOfConversion = 4;
	adc_handle.Init.DiscontinuousConvMode = DISABLE;
	adc_handle.Init.NbrOfDiscConversion = 0;
 	adc_handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;

	HAL_ADC_Init(&adc_handle);

	HAL_ADCEx_Calibration_Start(&adc_handle);

	 __HAL_RCC_DMA1_CLK_ENABLE();
    dma_handle.Instance = DMA1_Channel1;
    dma_handle.Init.Direction = DMA_PERIPH_TO_MEMORY;

    dma_handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    dma_handle.Init.MemInc = DMA_MINC_ENABLE;

    dma_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    dma_handle.Init.PeriphInc = DMA_PINC_DISABLE;

    dma_handle.Init.Priority = DMA_PRIORITY_MEDIUM;
    dma_handle.Init.Mode = DMA_CIRCULAR;
    HAL_DMA_Init(&dma_handle);

    __HAL_LINKDMA(&adc_handle, DMA_Handle, dma_handle);

}
void adc_dma_init(uint32_t *adc_value)
{
		adc_dma_set();
		adc_channel_config(&adc_handle,ADC_CHANNEL_0,ADC_REGULAR_RANK_1,ADC_SAMPLETIME_239CYCLES_5);
		adc_channel_config(&adc_handle,ADC_CHANNEL_1,ADC_REGULAR_RANK_2,ADC_SAMPLETIME_239CYCLES_5);
		adc_channel_config(&adc_handle,ADC_CHANNEL_2,ADC_REGULAR_RANK_3,ADC_SAMPLETIME_239CYCLES_5);
		adc_channel_config(&adc_handle,ADC_CHANNEL_3,ADC_REGULAR_RANK_4,ADC_SAMPLETIME_239CYCLES_5);
		HAL_ADC_Start_DMA(&adc_handle,adc_value,4);
}
