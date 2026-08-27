#include "can.h"

CAN_HandleTypeDef can_handle = {0};
void can_init(void)
{
		can_handle.Instance = CAN1;				//can通道
//		can_handle.Init.Mode = CAN_MODE_LOOPBACK;					//工作模式  环回模式
		can_handle.Init.Mode = CAN_MODE_NORMAL;					//工作模式		正常模式
		can_handle.Init.Prescaler = 4;			//预分频器 
		can_handle.Init.TimeSeg1 = CAN_BS1_9TQ;
		can_handle.Init.TimeSeg2 = CAN_BS2_8TQ;
		can_handle.Init.SyncJumpWidth = CAN_SJW_1TQ;
	
		can_handle.Init.AutoBusOff						= DISABLE;	//禁止自动离线管理
		can_handle.Init.AutoRetransmission		= DISABLE;	//禁止自动重发
		can_handle.Init.AutoWakeUp						= DISABLE;	//禁止自动唤醒
		can_handle.Init.ReceiveFifoLocked			= DISABLE;	//禁止接收FIFO锁定
		can_handle.Init.TimeTriggeredMode			= DISABLE;	//禁止时间触发通信模式
		can_handle.Init.TransmitFifoPriority	= DISABLE;	//禁止发送FIFO优先级
		HAL_CAN_Init(&can_handle);
	
		CAN_FilterTypeDef can_filterconfig = {0};					//过滤器设置
		can_filterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
		can_filterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
		can_filterconfig.FilterMaskIdHigh = 0;
		can_filterconfig.FilterMaskIdLow = 0;
		can_filterconfig.FilterIdHigh = 0;
		can_filterconfig.FilterIdLow = 0;
		
		can_filterconfig.FilterBank = 0;
		can_filterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
		can_filterconfig.FilterActivation = CAN_FILTER_ENABLE;
		can_filterconfig.SlaveStartFilterBank = 14;
		HAL_CAN_ConfigFilter(&can_handle,&can_filterconfig);
		
		HAL_CAN_Start(&can_handle);
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan)
{
		__HAL_RCC_CAN1_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
	
		GPIO_InitTypeDef gpio_initstruct;//定义结构体参数
	
		gpio_initstruct.Pin = GPIO_PIN_9;
		gpio_initstruct.Mode = GPIO_MODE_AF_PP;//配置工作模式 复用输出
		gpio_initstruct.Pull = GPIO_PULLUP;//配置上拉下拉
		gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;//配置输出速度
	
		HAL_GPIO_Init(GPIOB,&gpio_initstruct);
		gpio_initstruct.Pin = GPIO_PIN_8;
		gpio_initstruct.Mode = GPIO_MODE_AF_INPUT;//配置工作模式 复用输入
		HAL_GPIO_Init(GPIOB,&gpio_initstruct);
}


void can_send_data(uint32_t id,uint8_t *buf,uint8_t len)
{
	uint32_t tx_mail = CAN_TX_MAILBOX0;
	CAN_TxHeaderTypeDef tx_header = {0};
	tx_header.ExtId = id;
	tx_header.DLC = len;
	tx_header.IDE = CAN_ID_EXT;
	tx_header.RTR = CAN_RTR_DATA;	//帧格式
	HAL_CAN_AddTxMessage(&can_handle,&tx_header,buf,&tx_mail);

	while(HAL_CAN_GetTxMailboxesFreeLevel(&can_handle) != 3);
	
	uint8_t i = 0;
	printf("发送数据:\r\n");
	for(i=0;i<len;i++)
			printf("%X",buf[i]);
	printf("\r\n");
	
}
uint8_t can_receive_data(uint8_t *buf)
{
		CAN_RxHeaderTypeDef rx_header ={0};
		
		if(HAL_CAN_GetRxFifoFillLevel(&can_handle,CAN_RX_FIFO0) == 0)
				return 0;
		
		HAL_CAN_GetRxMessage(&can_handle,CAN_RX_FIFO0,&rx_header,buf);
		
			uint8_t i = 0;
			printf("接收数据:\r\n");
			
			for(i=0;i<rx_header.DLC;i++)
					printf("%X",buf[i]);
			printf("\r\n");
			return rx_header.DLC;
}




















