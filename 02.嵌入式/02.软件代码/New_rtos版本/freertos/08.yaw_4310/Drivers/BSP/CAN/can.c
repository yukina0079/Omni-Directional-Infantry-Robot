#include "can.h"

CAN_HandleTypeDef can_handle = {0};
void can_init(void)
{
		can_handle.Instance = CAN1;

		can_handle.Init.Mode = CAN_MODE_NORMAL;
		can_handle.Init.Prescaler = 4;
		can_handle.Init.TimeSeg1 = CAN_BS1_9TQ;
		can_handle.Init.TimeSeg2 = CAN_BS2_8TQ;
		can_handle.Init.SyncJumpWidth = CAN_SJW_1TQ;

		can_handle.Init.AutoBusOff						= DISABLE;
		can_handle.Init.AutoRetransmission		= DISABLE;
		can_handle.Init.AutoWakeUp						= DISABLE;
		can_handle.Init.ReceiveFifoLocked			= DISABLE;
		can_handle.Init.TimeTriggeredMode			= DISABLE;
		can_handle.Init.TransmitFifoPriority	= DISABLE;
		HAL_CAN_Init(&can_handle);

		CAN_FilterTypeDef can_filterconfig = {0};
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

		GPIO_InitTypeDef gpio_initstruct;

		gpio_initstruct.Pin = GPIO_PIN_9;
		gpio_initstruct.Mode = GPIO_MODE_AF_PP;
		gpio_initstruct.Pull = GPIO_PULLUP;
		gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;

		HAL_GPIO_Init(GPIOB,&gpio_initstruct);
		gpio_initstruct.Pin = GPIO_PIN_8;
		gpio_initstruct.Mode = GPIO_MODE_AF_INPUT;
		HAL_GPIO_Init(GPIOB,&gpio_initstruct);
}

void can_send_data(uint32_t id,uint8_t *buf,uint8_t len)
{
	uint32_t tx_mail = CAN_TX_MAILBOX0;
	CAN_TxHeaderTypeDef tx_header = {0};
	tx_header.ExtId = id;
	tx_header.DLC = len;
	tx_header.IDE = CAN_ID_EXT;
	tx_header.RTR = CAN_RTR_DATA;
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
