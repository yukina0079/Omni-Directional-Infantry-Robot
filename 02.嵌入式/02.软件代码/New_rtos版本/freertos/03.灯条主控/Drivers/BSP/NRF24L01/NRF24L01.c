#include "nrf24L01.h"
#include "led.h"
#include "uart1.h"
#include "delay.h"

#define ADDR_AB_LEN    5
#define ADDR_BC_LEN    5
#define DATA_WIDTH     13

uint8_t ADDR_AB[ADDR_AB_LEN] = {0x11, 0x22, 0x33, 0x44, 0x55};
#define CH_AB          30

uint8_t ADDR_BC[ADDR_BC_LEN] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE};
#define CH_BC          40

const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01};
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01};

SPI_HandleTypeDef SPI1_handle = {0};

void NRF24L01_Init(void)
{
		SPI1_handle.Instance = SPI1;
		SPI1_handle.Init.Mode = SPI_MODE_MASTER;
		SPI1_handle.Init.Direction = SPI_DIRECTION_2LINES;
		SPI1_handle.Init.DataSize = SPI_DATASIZE_8BIT;
		SPI1_handle.Init.CLKPolarity = SPI_POLARITY_LOW;
		SPI1_handle.Init.CLKPhase = SPI_PHASE_1EDGE;
		SPI1_handle.Init.NSS = SPI_NSS_SOFT;
		SPI1_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
		SPI1_handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
		SPI1_handle.Init.TIMode = SPI_TIMODE_DISABLE;
		SPI1_handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
		SPI1_handle.Init.CRCPolynomial = 7;
		HAL_SPI_Init(&SPI1_handle);
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
		if(hspi->Instance == SPI1){

				GPIO_InitTypeDef gpio_initstruct;
				__HAL_RCC_GPIOA_CLK_ENABLE();
				__HAL_RCC_SPI1_CLK_ENABLE();

				gpio_initstruct.Pin = GPIO_PIN_4;
				gpio_initstruct.Mode = GPIO_MODE_OUTPUT_PP;
				gpio_initstruct.Pull = GPIO_PULLUP;
				gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;
				HAL_GPIO_Init(GPIOA,&gpio_initstruct);

				gpio_initstruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
				gpio_initstruct.Mode = GPIO_MODE_AF_PP;
				HAL_GPIO_Init(GPIOA,&gpio_initstruct);

				gpio_initstruct.Pin = GPIO_PIN_6;
				gpio_initstruct.Mode = GPIO_MODE_INPUT;
				HAL_GPIO_Init(GPIOA,&gpio_initstruct);

				__HAL_RCC_GPIOA_CLK_ENABLE();
				gpio_initstruct.Pin = GPIO_PIN_1;
				gpio_initstruct.Mode = GPIO_MODE_OUTPUT_PP;
				gpio_initstruct.Pull = GPIO_PULLUP;
				gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;
				HAL_GPIO_Init(GPIOA,&gpio_initstruct);

				gpio_initstruct.Pin = GPIO_PIN_0;
				gpio_initstruct.Mode = GPIO_MODE_INPUT;
				HAL_GPIO_Init(GPIOA,&gpio_initstruct);
		}

}

uint8_t SPI_RT_byte(uint8_t data)
{
		uint8_t recv_data = 0;
		HAL_SPI_TransmitReceive(&SPI1_handle,&data,&recv_data,1,1000);
		return recv_data;
}

uint8_t NRF24L01_Check(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t i;

	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5);
	NRF24L01_Read_Buf(TX_ADDR,buf,5);
	for(i=0;i<5;i++)if(buf[i]!=0XA5)break;
	if(i!=5)return 1;
	return 0;
}

uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value)
{
    uint8_t status;
  NRF24L01_SPI_CS_ENABLE();
  status =SPI_RT_byte(reg);
  SPI_RT_byte(value);
  NRF24L01_SPI_CS_DISABLE();
  return(status);
}

uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;
 	NRF24L01_SPI_CS_ENABLE();
  SPI_RT_byte(reg);
  reg_val=SPI_RT_byte(0XFF);
  NRF24L01_SPI_CS_DISABLE();
  return(reg_val);
}

uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
	uint8_t status,uint8_t_ctr;

  NRF24L01_SPI_CS_ENABLE();
  status=SPI_RT_byte(reg);
 	for(uint8_t_ctr=0;uint8_t_ctr<len;uint8_t_ctr++)
  {
    pBuf[uint8_t_ctr]=SPI_RT_byte(0XFF);
  }
  NRF24L01_SPI_CS_DISABLE();
  return status;
}

uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status,uint8_t_ctr;
 	NRF24L01_SPI_CS_ENABLE();
  status = SPI_RT_byte(reg);
  for(uint8_t_ctr=0; uint8_t_ctr<len; uint8_t_ctr++)
  {
    SPI_RT_byte(*pBuf++);
  }
  NRF24L01_SPI_CS_DISABLE();
  return status;
}

uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
	uint8_t sta;
	NRF24L01_CE_LOW();
    NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);
 	NRF24L01_CE_HIGH();

	while(NRF24L01_IRQ_PIN_READ()!=0);

	sta=NRF24L01_Read_Reg(STATUS);
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta);
	if(sta&MAX_TX)
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);
		return MAX_TX;
	}
	if(sta&TX_OK)
	{
		return TX_OK;
	}
	return 0xff;
}

uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t sta;
	sta=NRF24L01_Read_Reg(STATUS);
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta);
	if(sta&RX_OK)
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);
		NRF24L01_Write_Reg(FLUSH_RX,0xff);
		return 0;
	}
	return 1;
}

void NRF24L01_RX_Mode(void)
{
  NRF24L01_CE_LOW();
  NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0F);
  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);
  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);
  NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);
  NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);
  NRF24L01_CE_HIGH();
  HAL_Delay(1);
}

void NRF24L01_TX_Mode(void)
{
  NRF24L01_CE_LOW();
  NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);
  NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);
  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);
  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);
  NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0xff);
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);
  NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG,0x0e);
  NRF24L01_CE_HIGH();
  HAL_Delay(1);
}

void NRF_LowPower_Mode(void)
{
	NRF24L01_CE_LOW();
	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x00);

}

void B_nrf24l01_switch_rx_from_A(void)
{
    NRF24L01_CE_LOW();

    NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, ADDR_AB, ADDR_AB_LEN);

    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, CH_AB);
    NRF24L01_Write_Reg(NRF_WRITE_REG + RX_PW_P0, DATA_WIDTH);
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0f);
    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);

    NRF24L01_CE_HIGH();
    HAL_Delay(100);
}

void B_nrf24l01_switch_tx_to_C(void)
{
    NRF24L01_CE_LOW();

    NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, ADDR_BC, ADDR_BC_LEN);

    NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, ADDR_BC, ADDR_BC_LEN);

    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);
    NRF24L01_Write_Reg(NRF_WRITE_REG + SETUP_RETR, 0x1a);
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, CH_BC);
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0f);
    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);

    HAL_Delay(100);
}

void C_nrf24l01_rx_from_B(void)
{
    NRF24L01_CE_LOW();

    NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, ADDR_BC, ADDR_BC_LEN);

    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, CH_BC);
    NRF24L01_Write_Reg(NRF_WRITE_REG + RX_PW_P0, DATA_WIDTH);
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0f);
    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);

    NRF24L01_CE_HIGH();
    HAL_Delay(100);
}

static void nrf24_dump_regs(const char *tag)
{
	uint8_t status, config, rf_ch, rf_setup, fifo;
	uint8_t tx_addr[5];
	uint8_t i;

	status   = NRF24L01_Read_Reg(STATUS);
	config   = NRF24L01_Read_Reg(CONFIG);
	rf_ch    = NRF24L01_Read_Reg(RF_CH);
	rf_setup = NRF24L01_Read_Reg(RF_SETUP);
	fifo     = NRF24L01_Read_Reg(NRF_FIFO_STATUS);
	NRF24L01_Read_Buf(TX_ADDR, tx_addr, 5);

	printf("[%s] STATUS=0x%02X CONFIG=0x%02X RF_CH=0x%02X RF_SETUP=0x%02X FIFO=0x%02X\r\n",
	       tag, status, config, rf_ch, rf_setup, fifo);
	printf("[%s] TX_ADDR=", tag);
	for (i = 0; i < 5; i++) {
		printf("%02X%s", tx_addr[i], (i == 4) ? "\r\n" : " ");
	}
}

void nrf2401_chack(void)
{
	uint8_t retry = 0;
	uint8_t ok = 0;

	printf("NRF24L01 SPI check start\r\n");

	for (retry = 1; retry <= 5; retry++) {
		if (NRF24L01_Check() == 0) {
			ok = 1;
			break;
		}
		lde1_toggle();
		printf("NRF24L01 SPI check FAIL, retry=%u/5\r\n", retry);
		nrf24_dump_regs("fail");
		delay_ms(400);
	}

	if (ok) {
		printf("NRF24L01 SPI check PASS\r\n");
		nrf24_dump_regs("pass");
		B_nrf24l01_switch_rx_from_A();
		printf("NRF24L01 RX mode: addr=11:22:33:44:55 ch=30\r\n");
		nrf24_dump_regs("rx");
	} else {
		printf("NRF24L01 NOT FOUND after 5 retries\r\n");
		printf("Check: NRF_3V3, SPI1 PA4/5/6/7, CE=PA1, solder joints\r\n");
	}
}
