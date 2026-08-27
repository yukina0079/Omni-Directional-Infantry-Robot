#include "nrf24L01.h"
#include "led.h"
#include "delay.h"

#define ADDR_AB_LEN    5   // 与库函数TX_ADR_WIDTH=5一致
#define ADDR_BC_LEN    5   // 与库函数TX_ADR_WIDTH=5一致
#define DATA_WIDTH     13  // 与库函数TX_PLOAD_WIDTH=32一致

// A→B 通信参数（A、B模块共用）
uint8_t ADDR_AB[ADDR_AB_LEN] = {0x11, 0x22, 0x33, 0x44, 0x55};
#define CH_AB          30  // A-B通信频道

// B→C 通信参数（B、C模块共用）
uint8_t ADDR_BC[ADDR_BC_LEN] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE};
#define CH_BC          40  // B-C通信频道

const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //发送地址
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //接收地址

SPI_HandleTypeDef SPI1_handle = {0};
/**
  * 函数功能: 初始化SPI1
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void NRF24L01_Init(void)
{
		SPI1_handle.Instance = SPI2;
		SPI1_handle.Init.Mode = SPI_MODE_MASTER;//指定SPI操作模式。
		SPI1_handle.Init.Direction = SPI_DIRECTION_2LINES;//指定SPI双向模式状态。
		SPI1_handle.Init.DataSize = SPI_DATASIZE_8BIT;//指定SPI数据大小。
		SPI1_handle.Init.CLKPolarity = SPI_POLARITY_LOW;//指定串行时钟稳定状态。
		SPI1_handle.Init.CLKPhase = SPI_PHASE_1EDGE;//指定位捕获的时钟活动沿。
		SPI1_handle.Init.NSS = SPI_NSS_SOFT;//指定NSS信号是否由管理硬件（NSS引脚）或使用SSI位的软件。	//自定义引脚
		SPI1_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;//指定波特率预分频器值，该值将为用于配置发送和接收SCK时钟。
		SPI1_handle.Init.FirstBit = SPI_FIRSTBIT_MSB;//指定数据传输是从MSB还是LSB位开始
		SPI1_handle.Init.TIMode = SPI_TIMODE_DISABLE;//指定是否启用TI模式。
		SPI1_handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;//指定是否启用CRC计算。
		SPI1_handle.Init.CRCPolynomial = 7;//指定用于CRC计算的多项式。
		HAL_SPI_Init(&SPI1_handle);
}
/**
  * 函数功能: 初始化SPI1硬件
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
//		if(hspi->Instance == SPI2){
//			
//				GPIO_InitTypeDef gpio_initstruct;
//				__HAL_RCC_GPIOA_CLK_ENABLE();		
//				__HAL_RCC_SPI2_CLK_ENABLE();
//			
//				gpio_initstruct.Pin = GPIO_PIN_11;//NSS
//				gpio_initstruct.Mode = GPIO_MODE_OUTPUT_PP;//推挽输出
//				gpio_initstruct.Pull = GPIO_PULLUP;//配置上拉下拉
//				gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;//配置输出速度	
//				HAL_GPIO_Init(GPIOA,&gpio_initstruct);

//				gpio_initstruct.Pin = GPIO_PIN_15;//使能
//				gpio_initstruct.Mode = GPIO_MODE_OUTPUT_PP;//推挽输出
//				gpio_initstruct.Pull = GPIO_PULLUP;//配置上拉下拉
//				gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;//配置输出速度	
//				HAL_GPIO_Init(GPIOA,&gpio_initstruct);
//			
//				__HAL_RCC_GPIOB_CLK_ENABLE();
//			
//				gpio_initstruct.Pin = GPIO_PIN_13|GPIO_PIN_15;//SLK,DI
//				gpio_initstruct.Mode = GPIO_MODE_AF_PP;//复用推挽输出
//				HAL_GPIO_Init(GPIOB,&gpio_initstruct);		
//			
//				gpio_initstruct.Pin = GPIO_PIN_14;//DO
//				gpio_initstruct.Mode = GPIO_MODE_INPUT;//输入
//				HAL_GPIO_Init(GPIOB,&gpio_initstruct);
//								
//				gpio_initstruct.Pin = GPIO_PIN_3;//中断
//				gpio_initstruct.Mode = GPIO_MODE_INPUT;//输入
//				HAL_GPIO_Init(GPIOB,&gpio_initstruct);
//		}
		if(hspi->Instance == SPI2){
			
				GPIO_InitTypeDef gpio_initstruct;
				__HAL_RCC_GPIOB_CLK_ENABLE();		
				__HAL_RCC_SPI2_CLK_ENABLE();
			
				gpio_initstruct.Pin = GPIO_PIN_12;//NSS
				gpio_initstruct.Mode = GPIO_MODE_OUTPUT_PP;//推挽输出
				gpio_initstruct.Pull = GPIO_PULLUP;//配置上拉下拉
				gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;//配置输出速度	
				HAL_GPIO_Init(GPIOB,&gpio_initstruct);

				gpio_initstruct.Pin = GPIO_PIN_13|GPIO_PIN_15;//SLK,DI
				gpio_initstruct.Mode = GPIO_MODE_AF_PP;//复用推挽输出
				HAL_GPIO_Init(GPIOB,&gpio_initstruct);		
			
				gpio_initstruct.Pin = GPIO_PIN_14;//DO
				gpio_initstruct.Mode = GPIO_MODE_INPUT;//输入
				HAL_GPIO_Init(GPIOB,&gpio_initstruct);

				gpio_initstruct.Pin = GPIO_PIN_2;//使能
				gpio_initstruct.Mode = GPIO_MODE_OUTPUT_PP;//推挽输出
				gpio_initstruct.Pull = GPIO_PULLUP;//配置上拉下拉
				gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;//配置输出速度	
				HAL_GPIO_Init(GPIOB,&gpio_initstruct);
				
				gpio_initstruct.Pin = GPIO_PIN_1;//中断
				gpio_initstruct.Mode = GPIO_MODE_INPUT;//输入
				HAL_GPIO_Init(GPIOB,&gpio_initstruct);
		}

}
/**
  * 函数功能: 往串行Flash读取写入一个字节数据并接收一个字节数据
  * 输入参数: data：待发送数据
  * 返 回 值: uint8_t：接收到的数据
  * 说    明：无
  */
uint8_t SPI_RT_byte(uint8_t data)
{
		uint8_t recv_data = 0;
		HAL_SPI_TransmitReceive(&SPI1_handle,&data,&recv_data,1,1000);
		return recv_data;
}

/**
  * 函数功能: 检测24L01是否存在
  * 输入参数: 无
  * 返 回 值: 0，成功;1，失败
  * 说    明：无          
  */ 
uint8_t NRF24L01_Check(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t i;
   
	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.	
	NRF24L01_Read_Buf(TX_ADDR,buf,5); //读出写入的地址  
	for(i=0;i<5;i++)if(buf[i]!=0XA5)break;	 							   
	if(i!=5)return 1;   //检测24L01错误	
	return 0;		 	//检测到24L01
}	
 
/**
  * 函数功能: SPI写寄存器
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：reg:指定寄存器地址
  *           
  */ 
uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value)
{
    uint8_t status;	
  NRF24L01_SPI_CS_ENABLE();                 //使能SPI传输
  status =SPI_RT_byte(reg);   //发送寄存器号 
  SPI_RT_byte(value);         //写入寄存器的值
  NRF24L01_SPI_CS_DISABLE();                //禁止SPI传输	   
  return(status);       			//返回状态值
}
 
/**
  * 函数功能: 读取SPI寄存器值
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：reg:要读的寄存器
  *           
  */ 
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;	    
 	NRF24L01_SPI_CS_ENABLE();          //使能SPI传输		
  SPI_RT_byte(reg);   //发送寄存器号
  reg_val=SPI_RT_byte(0XFF);//读取寄存器内容
  NRF24L01_SPI_CS_DISABLE();          //禁止SPI传输		    
  return(reg_val);           //返回状态值
}		
 
/**
  * 函数功能: 在指定位置读出指定长度的数据
  * 输入参数: 无
  * 返 回 值: 此次读到的状态寄存器值 
  * 说    明：无
  *           
  */ 
uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
	uint8_t status,uint8_t_ctr;	   
  
  NRF24L01_SPI_CS_ENABLE();           //使能SPI传输
  status=SPI_RT_byte(reg);//发送寄存器值(位置),并读取状态值   	   
 	for(uint8_t_ctr=0;uint8_t_ctr<len;uint8_t_ctr++)
  {
    pBuf[uint8_t_ctr]=SPI_RT_byte(0XFF);//读出数据
  }
  NRF24L01_SPI_CS_DISABLE();       //关闭SPI传输
  return status;        //返回读到的状态值
}
 
/**
  * 函数功能: 在指定位置写指定长度的数据
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：reg:寄存器(位置)  *pBuf:数据指针  len:数据长度
  *           
  */ 
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status,uint8_t_ctr;	    
 	NRF24L01_SPI_CS_ENABLE();          //使能SPI传输
  status = SPI_RT_byte(reg);//发送寄存器值(位置),并读取状态值
  for(uint8_t_ctr=0; uint8_t_ctr<len; uint8_t_ctr++)
  {
    SPI_RT_byte(*pBuf++); //写入数据	 
  }
  NRF24L01_SPI_CS_DISABLE();       //关闭SPI传输
  return status;          //返回读到的状态值
}		
 
/**
  * 函数功能: 启动NRF24L01发送一次数据
  * 输入参数: 无
  * 返 回 值: 发送完成状况
  * 说    明：txbuf:待发送数据首地址
  *           
  */ 
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
	uint8_t sta; 
	NRF24L01_CE_LOW();
    NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节
 	NRF24L01_CE_HIGH();//启动发送	 
  
	while(NRF24L01_IRQ_PIN_READ()!=0);//等待发送完成
  
	sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值	   
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&MAX_TX)//达到最大重发次数
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
		return MAX_TX; 
	}
	if(sta&TX_OK)//发送完成
	{
		return TX_OK;
	}
	return 0xff;//其他原因发送失败
}
 
/**
  * 函数功能:启动NRF24L01接收一次数据
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  *           
  */ 
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t sta;		 
	sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值    	 
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&RX_OK)//接收到数据
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
		return 0; 
	}	   
	return 1;//没收到任何数据
}			
 
/**
  * 函数功能: 该函数初始化NRF24L01到RX模式
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  *           
  */ 
void NRF24L01_RX_Mode(void)
{
  NRF24L01_CE_LOW();	  
  NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0F);//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC 
  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);    //使能通道0的自动应答    
  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);//使能通道0的接收地址  	 
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);	     //设置RF通信频率		  
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);//设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度 	       
  NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址
  NRF24L01_CE_HIGH(); //CE为高,进入接收模式 
  HAL_Delay(1);
}	
 
/**
  * 函数功能: 该函数初始化NRF24L01到TX模式
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  *           
  */ 
void NRF24L01_TX_Mode(void)
{														 
  NRF24L01_CE_LOW();	    
  NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 
  NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK	  
  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答    
  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址  
  NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0xff);//设置自动重发间隔时间:4000us + 86us;最大自动重发次数:15次
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);       //设置RF通道为40
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
  NRF24L01_CE_HIGH();//CE为高,10us后启动发送
  HAL_Delay(1);
}
 
/**
  * 函数功能: 该函数NRF24L01进入低功耗模式
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  *           
  */
void NRF_LowPower_Mode(void)
{
	NRF24L01_CE_LOW();	 
	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x00);		//配置工作模式:掉电模式
	
}
/**
 * @brief  A模块配置：发送数据给B（使用A-B通信参数）
 * @note   基于库函数nrf24l01_tx_mode()修改，替换地址和频道为ADDR_AB、CH_AB
 */
void A_nrf24l01_tx_to_B(void)
{
    NRF24L01_CE_LOW();  // 拉低CE，进入配置模式
    
    // 1. 写TX地址（A→B的地址 ADDR_AB）
    NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR,ADDR_AB,ADDR_AB_LEN);
    // 2. 写RX_ADDR_P0（使能ACK，必须与B的TX地址一致，即ADDR_AB）
    NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, ADDR_AB, ADDR_AB_LEN);
    
    // 3. 库函数默认配置（自动应答、重发参数等，保持不变）
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);        // 使能通道0自动应答
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);    // 使能通道0接收地址
    NRF24L01_Write_Reg(NRF_WRITE_REG + SETUP_RETR, 0x1a);   // 自动重发：10次，间隔500us+86us
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, CH_AB);       // 配置为A-B频道 30（替换默认的40）
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0f);     // 0db增益，2Mbps，低噪声增益开启
    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);       // 发送模式，PWR_UP，EN_CRC
    
    NRF24L01_CE_HIGH();  // 拉高CE，进入发送准备状态（10us后启动发送）
    HAL_Delay(100);   // 延时保证配置生效
}
void NRF_check(void)
{
	 while(NRF24L01_Check()){
		LED0_TOGGLE();
		 delay_ms(1000);
	 }
		LED0(0);
		A_nrf24l01_tx_to_B();//设置为发送模式
		
}

