#include "nrf24L01.h"
#include "led.h"
#include "uart1.h"
#include "delay.h"
//#include "spi.h"
#define ADDR_AB_LEN    5   // 锟斤拷夂拷锟絋X_ADR_WIDTH=5一锟斤拷
#define ADDR_BC_LEN    5   // 锟斤拷夂拷锟絋X_ADR_WIDTH=5一锟斤拷
#define DATA_WIDTH     13  // 锟斤拷夂拷锟絋X_PLOAD_WIDTH=32一锟斤拷

// A锟斤拷B 通锟脚诧拷锟斤拷锟斤拷A锟斤拷B模锟介共锟矫ｏ拷
uint8_t ADDR_AB[ADDR_AB_LEN] = {0x11, 0x22, 0x33, 0x44, 0x55};
#define CH_AB          30  // A-B通锟斤拷频锟斤拷

// B锟斤拷C 通锟脚诧拷锟斤拷锟斤拷B锟斤拷C模锟介共锟矫ｏ拷
uint8_t ADDR_BC[ADDR_BC_LEN] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE};
#define CH_BC          40  // B-C通锟斤拷频锟斤拷

const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //锟斤拷锟酵碉拷址
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //锟斤拷锟秸碉拷址

SPI_HandleTypeDef SPI1_handle = {0};
/**
  * 锟斤拷锟斤拷锟斤拷锟斤拷: 锟斤拷始锟斤拷SPI1
  * 锟斤拷锟斤拷锟斤拷锟: 锟斤拷
  * 锟斤拷 锟斤拷 值: 锟斤拷
  * 说    锟斤拷锟斤拷锟斤拷
  */
void NRF24L01_Init(void)
{
		SPI1_handle.Instance = SPI1;
		SPI1_handle.Init.Mode = SPI_MODE_MASTER;//指锟斤拷SPI锟斤拷锟斤拷模式锟斤拷
		SPI1_handle.Init.Direction = SPI_DIRECTION_2LINES;//指锟斤拷SPI双锟斤拷模式状态锟斤拷
		SPI1_handle.Init.DataSize = SPI_DATASIZE_8BIT;//指锟斤拷SPI锟斤拷锟捷达拷小锟斤拷
		SPI1_handle.Init.CLKPolarity = SPI_POLARITY_LOW;//指锟斤拷锟斤拷锟斤拷时锟斤拷锟饺讹拷状态锟斤拷
		SPI1_handle.Init.CLKPhase = SPI_PHASE_1EDGE;//指锟斤拷位锟斤拷锟斤拷锟绞憋拷踊疃拷亍锟
		SPI1_handle.Init.NSS = SPI_NSS_SOFT;//指锟斤拷NSS锟脚猴拷锟角凤拷锟缴癸拷锟斤拷硬锟斤拷锟斤拷NSS锟斤拷锟脚ｏ拷锟斤拷使锟斤拷SSI位锟斤拷锟斤拷锟斤拷锟斤拷	//锟皆讹拷锟斤拷锟斤拷锟斤拷
		SPI1_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;//指锟斤拷锟斤拷锟斤拷锟斤拷预锟斤拷频锟斤拷值锟斤拷锟斤拷值锟斤拷为锟斤拷锟斤拷锟斤拷锟矫凤拷锟酵和斤拷锟斤拷SCK时锟接★拷
		SPI1_handle.Init.FirstBit = SPI_FIRSTBIT_MSB;//指锟斤拷锟斤拷锟捷达拷锟斤拷锟角达拷MSB锟斤拷锟斤拷LSB位锟斤拷始
		SPI1_handle.Init.TIMode = SPI_TIMODE_DISABLE;//指锟斤拷锟角凤拷锟斤拷锟斤拷TI模式锟斤拷
		SPI1_handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;//指锟斤拷锟角凤拷锟斤拷锟斤拷CRC锟斤拷锟姐。
		SPI1_handle.Init.CRCPolynomial = 7;//指锟斤拷锟斤拷锟斤拷CRC锟斤拷锟斤拷亩锟斤拷锟绞斤拷锟
		HAL_SPI_Init(&SPI1_handle);
}
/**
  * 锟斤拷锟斤拷锟斤拷锟斤拷: 锟斤拷始锟斤拷SPI1硬锟斤拷
  * 锟斤拷锟斤拷锟斤拷锟: 锟斤拷
  * 锟斤拷 锟斤拷 值: 锟斤拷
  * 说    锟斤拷锟斤拷锟斤拷
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
		if(hspi->Instance == SPI1){
			
				GPIO_InitTypeDef gpio_initstruct;
				__HAL_RCC_GPIOA_CLK_ENABLE();		
				__HAL_RCC_SPI1_CLK_ENABLE();
			
				gpio_initstruct.Pin = GPIO_PIN_4;//NSS
				gpio_initstruct.Mode = GPIO_MODE_OUTPUT_PP;//锟斤拷锟斤拷锟斤拷锟
				gpio_initstruct.Pull = GPIO_PULLUP;//锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
				gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;//锟斤拷锟斤拷锟斤拷锟斤拷俣锟	
				HAL_GPIO_Init(GPIOA,&gpio_initstruct);

				gpio_initstruct.Pin = GPIO_PIN_5|GPIO_PIN_7;//SLK,DI
				gpio_initstruct.Mode = GPIO_MODE_AF_PP;//锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟
				HAL_GPIO_Init(GPIOA,&gpio_initstruct);		
			
				gpio_initstruct.Pin = GPIO_PIN_6;//DO
				gpio_initstruct.Mode = GPIO_MODE_INPUT;//锟斤拷锟斤拷
				HAL_GPIO_Init(GPIOA,&gpio_initstruct);
						
				__HAL_RCC_GPIOA_CLK_ENABLE();
				gpio_initstruct.Pin = GPIO_PIN_1;//使锟斤拷
				gpio_initstruct.Mode = GPIO_MODE_OUTPUT_PP;//锟斤拷锟斤拷锟斤拷锟
				gpio_initstruct.Pull = GPIO_PULLUP;//锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
				gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;//锟斤拷锟斤拷锟斤拷锟斤拷俣锟	
				HAL_GPIO_Init(GPIOA,&gpio_initstruct);
				
				gpio_initstruct.Pin = GPIO_PIN_0;//锟叫讹拷
				gpio_initstruct.Mode = GPIO_MODE_INPUT;//锟斤拷锟斤拷
				HAL_GPIO_Init(GPIOA,&gpio_initstruct);
		}

}
/**
  * 锟斤拷锟斤拷锟斤拷锟斤拷: 锟斤拷锟斤拷锟斤拷Flash锟斤拷取写锟斤拷一锟斤拷锟街斤拷锟斤拷锟捷诧拷锟斤拷锟斤拷一锟斤拷锟街斤拷锟斤拷锟斤拷
  * 锟斤拷锟斤拷锟斤拷锟: data锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
  * 锟斤拷 锟斤拷 值: uint8_t锟斤拷锟斤拷锟秸碉拷锟斤拷锟斤拷锟斤拷
  * 说    锟斤拷锟斤拷锟斤拷
  */
uint8_t SPI_RT_byte(uint8_t data)
{
		uint8_t recv_data = 0;
		HAL_SPI_TransmitReceive(&SPI1_handle,&data,&recv_data,1,1000);
		return recv_data;
}

/**
  * 锟斤拷锟斤拷锟斤拷锟斤拷: 锟斤拷锟24L01锟角凤拷锟斤拷锟
  * 锟斤拷锟斤拷锟斤拷锟: 锟斤拷
  * 锟斤拷 锟斤拷 值: 0锟斤拷锟缴癸拷;1锟斤拷失锟斤拷
  * 说    锟斤拷锟斤拷锟斤拷          
  */ 
uint8_t NRF24L01_Check(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t i;
   
	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5);//写锟斤拷5锟斤拷锟街节的碉拷址.	
	NRF24L01_Read_Buf(TX_ADDR,buf,5); //锟斤拷锟斤拷写锟斤拷牡锟街  
	for(i=0;i<5;i++)if(buf[i]!=0XA5)break;	 							   
	if(i!=5)return 1;   //锟斤拷锟24L01锟斤拷锟斤拷	
	return 0;		 	//锟斤拷獾24L01
}	
 
/**
  * 锟斤拷锟斤拷锟斤拷锟斤拷: SPI写锟侥达拷锟斤拷
  * 锟斤拷锟斤拷锟斤拷锟: 锟斤拷
  * 锟斤拷 锟斤拷 值: 锟斤拷
  * 说    锟斤拷锟斤拷reg:指锟斤拷锟侥达拷锟斤拷锟斤拷址
  *           
  */ 
uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value)
{
    uint8_t status;	
  NRF24L01_SPI_CS_ENABLE();                 //使锟斤拷SPI锟斤拷锟斤拷
  status =SPI_RT_byte(reg);   //锟斤拷锟酵寄达拷锟斤拷锟斤拷 
  SPI_RT_byte(value);         //写锟斤拷拇锟斤拷锟斤拷锟街
  NRF24L01_SPI_CS_DISABLE();                //锟斤拷止SPI锟斤拷锟斤拷	   
  return(status);       			//锟斤拷锟斤拷状态值
}
 
/**
  * 锟斤拷锟斤拷锟斤拷锟斤拷: 锟斤拷取SPI锟侥达拷锟斤拷值
  * 锟斤拷锟斤拷锟斤拷锟: 锟斤拷
  * 锟斤拷 锟斤拷 值: 锟斤拷
  * 说    锟斤拷锟斤拷reg:要锟斤拷锟侥寄达拷锟斤拷
  *           
  */ 
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;	    
 	NRF24L01_SPI_CS_ENABLE();          //使锟斤拷SPI锟斤拷锟斤拷		
  SPI_RT_byte(reg);   //锟斤拷锟酵寄达拷锟斤拷锟斤拷
  reg_val=SPI_RT_byte(0XFF);//锟斤拷取锟侥达拷锟斤拷锟斤拷锟斤拷
  NRF24L01_SPI_CS_DISABLE();          //锟斤拷止SPI锟斤拷锟斤拷		    
  return(reg_val);           //锟斤拷锟斤拷状态值
}		
 
/**
  * 锟斤拷锟斤拷锟斤拷锟斤拷: 锟斤拷指锟斤拷位锟矫讹拷锟斤拷指锟斤拷锟斤拷锟饺碉拷锟斤拷锟斤拷
  * 锟斤拷锟斤拷锟斤拷锟: 锟斤拷
  * 锟斤拷 锟斤拷 值: 锟剿次讹拷锟斤拷锟斤拷状态锟侥达拷锟斤拷值 
  * 说    锟斤拷锟斤拷锟斤拷
  *           
  */ 
uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
	uint8_t status,uint8_t_ctr;	   
  
  NRF24L01_SPI_CS_ENABLE();           //使锟斤拷SPI锟斤拷锟斤拷
  status=SPI_RT_byte(reg);//锟斤拷锟酵寄达拷锟斤拷值(位锟斤拷),锟斤拷锟斤拷取状态值   	   
 	for(uint8_t_ctr=0;uint8_t_ctr<len;uint8_t_ctr++)
  {
    pBuf[uint8_t_ctr]=SPI_RT_byte(0XFF);//锟斤拷锟斤拷锟斤拷锟斤拷
  }
  NRF24L01_SPI_CS_DISABLE();       //锟截憋拷SPI锟斤拷锟斤拷
  return status;        //锟斤拷锟截讹拷锟斤拷锟斤拷状态值
}
 
/**
  * 锟斤拷锟斤拷锟斤拷锟斤拷: 锟斤拷指锟斤拷位锟斤拷写指锟斤拷锟斤拷锟饺碉拷锟斤拷锟斤拷
  * 锟斤拷锟斤拷锟斤拷锟: 锟斤拷
  * 锟斤拷 锟斤拷 值: 锟斤拷
  * 说    锟斤拷锟斤拷reg:锟侥达拷锟斤拷(位锟斤拷)  *pBuf:锟斤拷锟斤拷指锟斤拷  len:锟斤拷锟捷筹拷锟斤拷
  *           
  */ 
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status,uint8_t_ctr;	    
 	NRF24L01_SPI_CS_ENABLE();          //使锟斤拷SPI锟斤拷锟斤拷
  status = SPI_RT_byte(reg);//锟斤拷锟酵寄达拷锟斤拷值(位锟斤拷),锟斤拷锟斤拷取状态值
  for(uint8_t_ctr=0; uint8_t_ctr<len; uint8_t_ctr++)
  {
    SPI_RT_byte(*pBuf++); //写锟斤拷锟斤拷锟斤拷	 
  }
  NRF24L01_SPI_CS_DISABLE();       //锟截憋拷SPI锟斤拷锟斤拷
  return status;          //锟斤拷锟截讹拷锟斤拷锟斤拷状态值
}		
 
/**
  * 锟斤拷锟斤拷锟斤拷锟斤拷: 锟斤拷锟斤拷NRF24L01锟斤拷锟斤拷一锟斤拷锟斤拷锟斤拷
  * 锟斤拷锟斤拷锟斤拷锟: 锟斤拷
  * 锟斤拷 锟斤拷 值: 锟斤拷锟斤拷锟斤拷锟阶达拷锟
  * 说    锟斤拷锟斤拷txbuf:锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟阶碉拷址
  *           
  */ 
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
	uint8_t sta; 
	NRF24L01_CE_LOW();
    NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写锟斤拷锟捷碉拷TX BUF  32锟斤拷锟街斤拷
 	NRF24L01_CE_HIGH();//锟斤拷锟斤拷锟斤拷锟斤拷	 
  
	while(NRF24L01_IRQ_PIN_READ()!=0);//锟饺达拷锟斤拷锟斤拷锟斤拷锟
  
	sta=NRF24L01_Read_Reg(STATUS);  //锟斤拷取状态锟侥达拷锟斤拷锟斤拷值	   
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //锟斤拷锟絋X_DS锟斤拷MAX_RT锟叫断憋拷志
	if(sta&MAX_TX)//锟斤到锟斤拷锟斤拷胤锟斤拷锟斤拷锟
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//锟斤拷锟絋X FIFO锟侥达拷锟斤拷 
		return MAX_TX; 
	}
	if(sta&TX_OK)//锟斤拷锟斤拷锟斤拷锟
	{
		return TX_OK;
	}
	return 0xff;//锟斤拷锟斤拷原锟斤拷锟斤拷失锟斤拷
}
 
/**
  * 锟斤拷锟斤拷锟斤拷锟斤拷:锟斤拷锟斤拷NRF24L01锟斤拷锟斤拷一锟斤拷锟斤拷锟斤拷
  * 锟斤拷锟斤拷锟斤拷锟: 锟斤拷
  * 锟斤拷 锟斤拷 值: 锟斤拷
  * 说    锟斤拷锟斤拷锟斤拷
  *           
  */ 
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t sta;		 
	sta=NRF24L01_Read_Reg(STATUS);  //锟斤拷取状态锟侥达拷锟斤拷锟斤拷值    	 
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //锟斤拷锟絋X_DS锟斤拷MAX_RT锟叫断憋拷志
	if(sta&RX_OK)//锟斤拷锟秸碉拷锟斤拷锟斤拷
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//锟斤拷取锟斤拷锟斤拷
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//锟斤拷锟絉X FIFO锟侥达拷锟斤拷 
		return 0; 
	}	   
	return 1;//没锟秸碉拷锟轿猴拷锟斤拷锟斤拷
}			
 
/**
  * 锟斤拷锟斤拷锟斤拷锟斤拷: 锟矫猴拷锟斤拷锟斤拷始锟斤拷NRF24L01锟斤拷RX模式
  * 锟斤拷锟斤拷锟斤拷锟: 锟斤拷
  * 锟斤拷 锟斤拷 值: 锟斤拷
  * 说    锟斤拷锟斤拷锟斤拷
  *           
  */ 
void NRF24L01_RX_Mode(void)
{
  NRF24L01_CE_LOW();	  
  NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0F);//锟斤拷锟矫伙拷锟斤拷锟斤拷锟斤拷模式锟侥诧拷锟斤拷;PWR_UP,EN_CRC,16BIT_CRC 
  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);    //使锟斤拷通锟斤拷0锟斤拷锟皆讹拷应锟斤拷    
  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);//使锟斤拷通锟斤拷0锟侥斤拷锟秸碉拷址  	 
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);	     //锟斤拷锟斤拷RF通锟斤拷频锟斤拷		  
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);//锟斤拷锟斤拷TX锟斤拷锟斤拷锟斤拷锟,0db锟斤拷锟斤拷,2Mbps,锟斤拷锟斤拷锟斤拷锟斤拷锟芥开锟斤拷   
  NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选锟斤拷通锟斤拷0锟斤拷锟斤拷效锟斤拷锟捷匡拷锟斤拷 	       
  NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);//写RX锟节碉拷锟街
  NRF24L01_CE_HIGH(); //CE为锟斤拷,锟斤拷锟斤拷锟斤拷锟侥Ｊ 
  HAL_Delay(1);
}	
 
/**
  * 锟斤拷锟斤拷锟斤拷锟斤拷: 锟矫猴拷锟斤拷锟斤拷始锟斤拷NRF24L01锟斤拷TX模式
  * 锟斤拷锟斤拷锟斤拷锟: 锟斤拷
  * 锟斤拷 锟斤拷 值: 锟斤拷
  * 说    锟斤拷锟斤拷锟斤拷
  *           
  */ 
void NRF24L01_TX_Mode(void)
{														 
  NRF24L01_CE_LOW();	    
  NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);//写TX锟节碉拷锟街 
  NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //锟斤拷锟斤拷TX锟节碉拷锟街,锟斤拷要为锟斤拷使锟斤拷ACK	  
  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);     //使锟斤拷通锟斤拷0锟斤拷锟皆讹拷应锟斤拷    
  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //使锟斤拷通锟斤拷0锟侥斤拷锟秸碉拷址  
  NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0xff);//锟斤拷锟斤拷锟皆讹拷锟截凤拷锟斤拷锟绞憋拷锟:4000us + 86us;锟斤拷锟斤拷远锟斤拷胤锟斤拷锟斤拷锟:15锟斤拷
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);       //锟斤拷锟斤拷RF通锟斤拷为40
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //锟斤拷锟斤拷TX锟斤拷锟斤拷锟斤拷锟,0db锟斤拷锟斤拷,2Mbps,锟斤拷锟斤拷锟斤拷锟斤拷锟芥开锟斤拷   
  NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG,0x0e);    //锟斤拷锟矫伙拷锟斤拷锟斤拷锟斤拷模式锟侥诧拷锟斤拷;PWR_UP,EN_CRC,16BIT_CRC,锟斤拷锟斤拷模式,锟斤拷锟斤拷锟斤拷锟斤拷锟叫讹拷
  NRF24L01_CE_HIGH();//CE为锟斤拷,10us锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
  HAL_Delay(1);
}
 
/**
  * 锟斤拷锟斤拷锟斤拷锟斤拷: 锟矫猴拷锟斤拷NRF24L01锟斤拷锟斤拷凸锟斤拷锟侥Ｊ
  * 锟斤拷锟斤拷锟斤拷锟: 锟斤拷
  * 锟斤拷 锟斤拷 值: 锟斤拷
  * 说    锟斤拷锟斤拷锟斤拷
  *           
  */
void NRF_LowPower_Mode(void)
{
	NRF24L01_CE_LOW();	 
	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x00);		//锟斤拷锟矫癸拷锟斤拷模式:锟斤拷锟斤拷模式
	
}
/**
 * @brief  B模锟斤拷锟斤拷锟斤拷1锟斤拷锟叫伙拷为锟斤拷锟斤拷锟斤拷A锟斤拷锟捷★拷锟斤拷模式锟斤拷A-B锟斤拷锟斤拷锟斤拷
 * @note   锟斤拷锟节库函锟斤拷nrf24l01_rx_mode()锟睫改ｏ拷锟斤拷锟斤拷ADDR_AB锟斤拷CH_AB
 */
void B_nrf24l01_switch_rx_from_A(void)
{
    NRF24L01_CE_LOW();  // 锟斤拷锟斤拷CE锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷模式锟斤拷停止锟斤拷前模式锟斤拷
    
    // 1. 写RX_ADDR_P0锟斤拷A锟斤拷B锟侥碉拷址 ADDR_AB锟斤拷
    NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, ADDR_AB, ADDR_AB_LEN);
    
    // 2. 锟解函锟斤拷默锟斤拷锟斤拷锟矫ｏ拷锟斤拷锟街诧拷锟戒，锟斤拷锟睫革拷频锟斤拷锟酵碉拷址锟斤拷
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);        // 使锟斤拷通锟斤拷0锟皆讹拷应锟斤拷锟斤拷A锟截达拷ACK锟斤拷
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);    // 使锟斤拷通锟斤拷0锟斤拷锟秸碉拷址
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, CH_AB);       // 锟叫伙拷锟斤拷A-B频锟斤拷 30
    NRF24L01_Write_Reg(NRF_WRITE_REG + RX_PW_P0, DATA_WIDTH); // 锟斤拷锟捷筹拷锟斤拷32锟街斤拷
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0f);     // 0db锟斤拷锟芥，2Mbps锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟芥开锟斤拷
    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);       // 锟斤拷锟斤拷模式锟斤拷PWR_UP锟斤拷EN_CRC
    
    NRF24L01_CE_HIGH();  // 锟斤拷锟斤拷CE锟斤拷锟斤拷锟斤拷锟斤拷锟阶刺
    HAL_Delay(100);   // 锟斤拷时锟斤拷证锟斤拷锟斤拷锟斤拷效锟斤拷锟斤拷锟斤拷模式锟斤拷突
}

/**
 * @brief  B模锟斤拷锟斤拷锟斤拷2锟斤拷锟叫伙拷为锟斤拷锟斤拷锟斤拷锟斤拷锟捷革拷C锟斤拷锟斤拷模式锟斤拷B-C锟斤拷锟斤拷锟斤拷
 * @note   锟斤拷锟节库函锟斤拷nrf24l01_tx_mode()锟睫改ｏ拷锟斤拷锟斤拷ADDR_BC锟斤拷CH_BC
 */
void B_nrf24l01_switch_tx_to_C(void)
{
    NRF24L01_CE_LOW();  // 锟斤拷锟斤拷CE锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷模式锟斤拷停止锟斤拷前模式锟斤拷
    
    // 1. 写TX锟斤拷址锟斤拷B锟斤拷C锟侥碉拷址 ADDR_BC锟斤拷
    NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, ADDR_BC, ADDR_BC_LEN);
    // 2. 写RX_ADDR_P0锟斤拷使锟斤拷ACK锟斤拷锟斤拷锟斤拷锟斤拷C锟斤拷TX锟斤拷址一锟铰ｏ拷锟斤拷ADDR_BC锟斤拷
    NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, ADDR_BC, ADDR_BC_LEN);
    
    // 3. 锟解函锟斤拷默锟斤拷锟斤拷锟矫ｏ拷锟斤拷锟街诧拷锟戒，锟斤拷锟睫革拷频锟斤拷锟酵碉拷址锟斤拷
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);        // 使锟斤拷通锟斤拷0锟皆讹拷应锟斤拷
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);    // 使锟斤拷通锟斤拷0锟斤拷锟秸碉拷址
    NRF24L01_Write_Reg(NRF_WRITE_REG + SETUP_RETR, 0x1a);   // 锟皆讹拷锟截凤拷锟斤拷10锟轿ｏ拷锟斤拷锟500us+86us
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, CH_BC);       // 锟叫伙拷锟斤拷B-C频锟斤拷 40
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0f);     // 0db锟斤拷锟芥，2Mbps锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟芥开锟斤拷
    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);       // 锟斤拷锟斤拷模式锟斤拷PWR_UP锟斤拷EN_CRC
    
    HAL_Delay(100);   // 锟斤拷时锟斤拷证锟斤拷锟斤拷锟斤拷效锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷CE锟斤拷锟斤拷锟斤拷时锟斤拷nrf24l01_tx_packet锟斤拷锟斤拷锟斤拷
}

/**
 * @brief  C模锟斤拷锟斤拷锟矫ｏ拷锟斤拷锟斤拷B转锟斤拷锟斤拷锟斤拷锟捷ｏ拷使锟斤拷B-C通锟脚诧拷锟斤拷锟斤拷
 * @note   锟斤拷锟节库函锟斤拷nrf24l01_rx_mode()锟睫改ｏ拷锟芥换锟斤拷址锟斤拷频锟斤拷为ADDR_BC锟斤拷CH_BC
 */
void C_nrf24l01_rx_from_B(void)
{
    NRF24L01_CE_LOW();  // 锟斤拷锟斤拷CE锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷模式
    
    // 1. 写RX_ADDR_P0锟斤拷B锟斤拷C锟侥碉拷址 ADDR_BC锟斤拷
    NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, ADDR_BC, ADDR_BC_LEN);
    
    // 2. 锟解函锟斤拷默锟斤拷锟斤拷锟矫ｏ拷锟皆讹拷应锟斤拷锟斤拷锟捷匡拷锟饺等ｏ拷锟斤拷锟街诧拷锟戒）
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);        // 使锟斤拷通锟斤拷0锟皆讹拷应锟斤拷锟斤拷B锟截达拷ACK锟斤拷
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);    // 使锟斤拷通锟斤拷0锟斤拷锟秸碉拷址
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, CH_BC);       // 锟斤拷锟斤拷为B-C频锟斤拷 40
    NRF24L01_Write_Reg(NRF_WRITE_REG + RX_PW_P0, DATA_WIDTH); // 锟斤拷锟捷筹拷锟斤拷32锟街斤拷
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0f);     // 0db锟斤拷锟芥，2Mbps锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟芥开锟斤拷
    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);       // 锟斤拷锟斤拷模式锟斤拷PWR_UP锟斤拷EN_CRC
    
    NRF24L01_CE_HIGH();  // 锟斤拷锟斤拷CE锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟阶刺
    HAL_Delay(100);   // 锟斤拷时锟斤拷证锟斤拷锟斤拷锟斤拷效
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
