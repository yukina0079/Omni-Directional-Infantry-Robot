#include "dma.h"
#include "stdio.h"
#include "uart1.h"

DMA_HandleTypeDef dam_handle = {0};

void dma_init(void)
{
		__HAL_RCC_DMA1_CLK_ENABLE();

		dam_handle.Instance = DMA1_Channel7;
		dam_handle.Init.Direction	= DMA_MEMORY_TO_PERIPH;

		dam_handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		dam_handle.Init.MemInc =	DMA_MINC_ENABLE;

		dam_handle.Init.PeriphDataAlignment	=	DMA_PDATAALIGN_BYTE;
		dam_handle.Init.PeriphInc	=	DMA_PINC_DISABLE;

		dam_handle.Init.Priority	=	DMA_PRIORITY_MEDIUM;
		dam_handle.Init.Mode	=		DMA_CIRCULAR;

		HAL_DMA_Init(&dam_handle);

		__HAL_LINKDMA(&uart1_handle,hdmatx,dam_handle);
}
