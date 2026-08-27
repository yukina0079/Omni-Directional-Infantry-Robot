#include "dma.h"
#include "stdio.h"
#include "uart1.h"


DMA_HandleTypeDef dam_handle = {0};


void dma_init(void)
{
		__HAL_RCC_DMA1_CLK_ENABLE();
	
		dam_handle.Instance = DMA1_Channel7;												//定时器1通道4  uart1_tx				
		dam_handle.Init.Direction	= DMA_MEMORY_TO_PERIPH;						//方向  内存到外设
		
		//内存相关配置
		dam_handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;		  //数据对齐方式
		dam_handle.Init.MemInc =	DMA_MINC_ENABLE;						      //增长方式
	
		//外设相关配置
		dam_handle.Init.PeriphDataAlignment	=	DMA_PDATAALIGN_BYTE;  //数据对齐方式
		dam_handle.Init.PeriphInc	=	DMA_PINC_DISABLE;				        //增长方式  外设不递增
	
		dam_handle.Init.Priority	=	DMA_PRIORITY_MEDIUM;						//优先级
		dam_handle.Init.Mode	=		DMA_CIRCULAR;												//模式：循环，非循环
	
		HAL_DMA_Init(&dam_handle);
	
		__HAL_LINKDMA(&uart1_handle,hdmatx,dam_handle);
}

