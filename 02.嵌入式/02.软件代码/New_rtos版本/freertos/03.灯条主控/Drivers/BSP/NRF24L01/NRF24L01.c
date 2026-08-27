#include "nrf24L01.h"
#include "led.h"
#include "uart1.h"
#include "delay.h"
//#include "spi.h"
#define ADDR_AB_LEN    5   // ��⺯��TX_ADR_WIDTH=5һ��
#define ADDR_BC_LEN    5   // ��⺯��TX_ADR_WIDTH=5һ��
#define DATA_WIDTH     13  // ��⺯��TX_PLOAD_WIDTH=32һ��

// A��B ͨ�Ų�����A��Bģ�鹲�ã�
uint8_t ADDR_AB[ADDR_AB_LEN] = {0x11, 0x22, 0x33, 0x44, 0x55};
#define CH_AB          30  // A-Bͨ��Ƶ��

// B��C ͨ�Ų�����B��Cģ�鹲�ã�
uint8_t ADDR_BC[ADDR_BC_LEN] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE};
#define CH_BC          40  // B-Cͨ��Ƶ��

const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //���͵�ַ
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //���յ�ַ

SPI_HandleTypeDef SPI1_handle = {0};
/**
  * ��������: ��ʼ��SPI1
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void NRF24L01_Init(void)
{
		SPI1_handle.Instance = SPI1;
		SPI1_handle.Init.Mode = SPI_MODE_MASTER;//ָ��SPI����ģʽ��
		SPI1_handle.Init.Direction = SPI_DIRECTION_2LINES;//ָ��SPI˫��ģʽ״̬��
		SPI1_handle.Init.DataSize = SPI_DATASIZE_8BIT;//ָ��SPI���ݴ�С��
		SPI1_handle.Init.CLKPolarity = SPI_POLARITY_LOW;//ָ������ʱ���ȶ�״̬��
		SPI1_handle.Init.CLKPhase = SPI_PHASE_1EDGE;//ָ��λ�����ʱ�ӻ�ء�
		SPI1_handle.Init.NSS = SPI_NSS_SOFT;//ָ��NSS�ź��Ƿ��ɹ���Ӳ����NSS���ţ���ʹ��SSIλ��������	//�Զ�������
		SPI1_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;//ָ��������Ԥ��Ƶ��ֵ����ֵ��Ϊ�������÷��ͺͽ���SCKʱ�ӡ�
		SPI1_handle.Init.FirstBit = SPI_FIRSTBIT_MSB;//ָ�����ݴ����Ǵ�MSB����LSBλ��ʼ
		SPI1_handle.Init.TIMode = SPI_TIMODE_DISABLE;//ָ���Ƿ�����TIģʽ��
		SPI1_handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;//ָ���Ƿ�����CRC���㡣
		SPI1_handle.Init.CRCPolynomial = 7;//ָ������CRC����Ķ���ʽ��
		HAL_SPI_Init(&SPI1_handle);
}
/**
  * ��������: ��ʼ��SPI1Ӳ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
		if(hspi->Instance == SPI1){
			
				GPIO_InitTypeDef gpio_initstruct;
				__HAL_RCC_GPIOA_CLK_ENABLE();		
				__HAL_RCC_SPI1_CLK_ENABLE();
			
				gpio_initstruct.Pin = GPIO_PIN_4;//NSS
				gpio_initstruct.Mode = GPIO_MODE_OUTPUT_PP;//�������
				gpio_initstruct.Pull = GPIO_PULLUP;//������������
				gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;//��������ٶ�	
				HAL_GPIO_Init(GPIOA,&gpio_initstruct);

				gpio_initstruct.Pin = GPIO_PIN_5|GPIO_PIN_7;//SLK,DI
				gpio_initstruct.Mode = GPIO_MODE_AF_PP;//�����������
				HAL_GPIO_Init(GPIOA,&gpio_initstruct);		
			
				gpio_initstruct.Pin = GPIO_PIN_6;//DO
				gpio_initstruct.Mode = GPIO_MODE_INPUT;//����
				HAL_GPIO_Init(GPIOA,&gpio_initstruct);
						
				__HAL_RCC_GPIOA_CLK_ENABLE();
				gpio_initstruct.Pin = GPIO_PIN_1;//ʹ��
				gpio_initstruct.Mode = GPIO_MODE_OUTPUT_PP;//�������
				gpio_initstruct.Pull = GPIO_PULLUP;//������������
				gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;//��������ٶ�	
				HAL_GPIO_Init(GPIOA,&gpio_initstruct);
				
				gpio_initstruct.Pin = GPIO_PIN_0;//�ж�
				gpio_initstruct.Mode = GPIO_MODE_INPUT;//����
				HAL_GPIO_Init(GPIOA,&gpio_initstruct);
		}

}
/**
  * ��������: ������Flash��ȡд��һ���ֽ����ݲ�����һ���ֽ�����
  * �������: data������������
  * �� �� ֵ: uint8_t�����յ�������
  * ˵    ������
  */
uint8_t SPI_RT_byte(uint8_t data)
{
		uint8_t recv_data = 0;
		HAL_SPI_TransmitReceive(&SPI1_handle,&data,&recv_data,1,1000);
		return recv_data;
}

/**
  * ��������: ���24L01�Ƿ����
  * �������: ��
  * �� �� ֵ: 0���ɹ�;1��ʧ��
  * ˵    ������          
  */ 
uint8_t NRF24L01_Check(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t i;
   
	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5);//д��5���ֽڵĵ�ַ.	
	NRF24L01_Read_Buf(TX_ADDR,buf,5); //����д��ĵ��  
	for(i=0;i<5;i++)if(buf[i]!=0XA5)break;	 							   
	if(i!=5)return 1;   //���24L01����	
	return 0;		 	//���24L01
}	
 
/**
  * ��������: SPIд�Ĵ���
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ����reg:ָ���Ĵ�����ַ
  *           
  */ 
uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value)
{
    uint8_t status;	
  NRF24L01_SPI_CS_ENABLE();                 //ʹ��SPI����
  status =SPI_RT_byte(reg);   //���ͼĴ����� 
  SPI_RT_byte(value);         //д��Ĵ������
  NRF24L01_SPI_CS_DISABLE();                //��ֹSPI����	   
  return(status);       			//����״ֵ̬
}
 
/**
  * ��������: ��ȡSPI�Ĵ���ֵ
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ����reg:Ҫ���ļĴ���
  *           
  */ 
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;	    
 	NRF24L01_SPI_CS_ENABLE();          //ʹ��SPI����		
  SPI_RT_byte(reg);   //���ͼĴ�����
  reg_val=SPI_RT_byte(0XFF);//��ȡ�Ĵ�������
  NRF24L01_SPI_CS_DISABLE();          //��ֹSPI����		    
  return(reg_val);           //����״ֵ̬
}		
 
/**
  * ��������: ��ָ��λ�ö���ָ�����ȵ�����
  * �������: ��
  * �� �� ֵ: �˴ζ�����״̬�Ĵ���ֵ 
  * ˵    ������
  *           
  */ 
uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
	uint8_t status,uint8_t_ctr;	   
  
  NRF24L01_SPI_CS_ENABLE();           //ʹ��SPI����
  status=SPI_RT_byte(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬   	   
 	for(uint8_t_ctr=0;uint8_t_ctr<len;uint8_t_ctr++)
  {
    pBuf[uint8_t_ctr]=SPI_RT_byte(0XFF);//��������
  }
  NRF24L01_SPI_CS_DISABLE();       //�ر�SPI����
  return status;        //���ض�����״ֵ̬
}
 
/**
  * ��������: ��ָ��λ��дָ�����ȵ�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ����reg:�Ĵ���(λ��)  *pBuf:����ָ��  len:���ݳ���
  *           
  */ 
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status,uint8_t_ctr;	    
 	NRF24L01_SPI_CS_ENABLE();          //ʹ��SPI����
  status = SPI_RT_byte(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
  for(uint8_t_ctr=0; uint8_t_ctr<len; uint8_t_ctr++)
  {
    SPI_RT_byte(*pBuf++); //д������	 
  }
  NRF24L01_SPI_CS_DISABLE();       //�ر�SPI����
  return status;          //���ض�����״ֵ̬
}		
 
/**
  * ��������: ����NRF24L01����һ������
  * �������: ��
  * �� �� ֵ: �������״��
  * ˵    ����txbuf:�����������׵�ַ
  *           
  */ 
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
	uint8_t sta; 
	NRF24L01_CE_LOW();
    NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//д���ݵ�TX BUF  32���ֽ�
 	NRF24L01_CE_HIGH();//��������	 
  
	while(NRF24L01_IRQ_PIN_READ()!=0);//�ȴ��������
  
	sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ	   
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	if(sta&MAX_TX)//�ﵽ����ط�����
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
		return MAX_TX; 
	}
	if(sta&TX_OK)//�������
	{
		return TX_OK;
	}
	return 0xff;//����ԭ����ʧ��
}
 
/**
  * ��������:����NRF24L01����һ������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  *           
  */ 
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t sta;		 
	sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ    	 
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	if(sta&RX_OK)//���յ�����
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ��� 
		return 0; 
	}	   
	return 1;//û�յ��κ�����
}			
 
/**
  * ��������: �ú�����ʼ��NRF24L01��RXģʽ
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  *           
  */ 
void NRF24L01_RX_Mode(void)
{
  NRF24L01_CE_LOW();	  
  NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0F);//���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC 
  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);    //ʹ��ͨ��0���Զ�Ӧ��    
  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);//ʹ��ͨ��0�Ľ��յ�ַ  	 
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);	     //����RFͨ��Ƶ��		  
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);//����TX�������,0db����,2Mbps,���������濪��   
  NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ��� 	       
  NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);//дRX�ڵ���
  NRF24L01_CE_HIGH(); //CEΪ��,�������ģ� 
  HAL_Delay(1);
}	
 
/**
  * ��������: �ú�����ʼ��NRF24L01��TXģʽ
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  *           
  */ 
void NRF24L01_TX_Mode(void)
{														 
  NRF24L01_CE_LOW();	    
  NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);//дTX�ڵ��� 
  NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ���,��ҪΪ��ʹ��ACK	  
  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ��    
  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ  
  NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0xff);//�����Զ��ط����ʱ��:4000us + 86us;����Զ��ط�����:15��
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);       //����RFͨ��Ϊ40
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //����TX�������,0db����,2Mbps,���������濪��   
  NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG,0x0e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
  NRF24L01_CE_HIGH();//CEΪ��,10us����������
  HAL_Delay(1);
}
 
/**
  * ��������: �ú���NRF24L01����͹���ģ�
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  *           
  */
void NRF_LowPower_Mode(void)
{
	NRF24L01_CE_LOW();	 
	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x00);		//���ù���ģʽ:����ģʽ
	
}
/**
 * @brief  Bģ������1���л�Ϊ������A���ݡ���ģʽ��A-B������
 * @note   ���ڿ⺯��nrf24l01_rx_mode()�޸ģ�����ADDR_AB��CH_AB
 */
void B_nrf24l01_switch_rx_from_A(void)
{
    NRF24L01_CE_LOW();  // ����CE����������ģʽ��ֹͣ��ǰģʽ��
    
    // 1. дRX_ADDR_P0��A��B�ĵ�ַ ADDR_AB��
    NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, ADDR_AB, ADDR_AB_LEN);
    
    // 2. �⺯��Ĭ�����ã����ֲ��䣬���޸�Ƶ���͵�ַ��
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);        // ʹ��ͨ��0�Զ�Ӧ����A�ش�ACK��
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);    // ʹ��ͨ��0���յ�ַ
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, CH_AB);       // �л���A-BƵ�� 30
    NRF24L01_Write_Reg(NRF_WRITE_REG + RX_PW_P0, DATA_WIDTH); // ���ݳ���32�ֽ�
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0f);     // 0db���棬2Mbps�����������濪��
    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);       // ����ģʽ��PWR_UP��EN_CRC
    
    NRF24L01_CE_HIGH();  // ����CE���������״�
    HAL_Delay(100);   // ��ʱ��֤������Ч������ģʽ��ͻ
}

/**
 * @brief  Bģ������2���л�Ϊ���������ݸ�C����ģʽ��B-C������
 * @note   ���ڿ⺯��nrf24l01_tx_mode()�޸ģ�����ADDR_BC��CH_BC
 */
void B_nrf24l01_switch_tx_to_C(void)
{
    NRF24L01_CE_LOW();  // ����CE����������ģʽ��ֹͣ��ǰģʽ��
    
    // 1. дTX��ַ��B��C�ĵ�ַ ADDR_BC��
    NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, ADDR_BC, ADDR_BC_LEN);
    // 2. дRX_ADDR_P0��ʹ��ACK��������C��TX��ַһ�£���ADDR_BC��
    NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, ADDR_BC, ADDR_BC_LEN);
    
    // 3. �⺯��Ĭ�����ã����ֲ��䣬���޸�Ƶ���͵�ַ��
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);        // ʹ��ͨ��0�Զ�Ӧ��
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);    // ʹ��ͨ��0���յ�ַ
    NRF24L01_Write_Reg(NRF_WRITE_REG + SETUP_RETR, 0x1a);   // �Զ��ط���10�Σ����500us+86us
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, CH_BC);       // �л���B-CƵ�� 40
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0f);     // 0db���棬2Mbps�����������濪��
    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);       // ����ģʽ��PWR_UP��EN_CRC
    
    HAL_Delay(100);   // ��ʱ��֤������Ч��������������CE������ʱ��nrf24l01_tx_packet������
}

/**
 * @brief  Cģ�����ã�����Bת�������ݣ�ʹ��B-Cͨ�Ų�����
 * @note   ���ڿ⺯��nrf24l01_rx_mode()�޸ģ��滻��ַ��Ƶ��ΪADDR_BC��CH_BC
 */
void C_nrf24l01_rx_from_B(void)
{
    NRF24L01_CE_LOW();  // ����CE����������ģʽ
    
    // 1. дRX_ADDR_P0��B��C�ĵ�ַ ADDR_BC��
    NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, ADDR_BC, ADDR_BC_LEN);
    
    // 2. �⺯��Ĭ�����ã��Զ�Ӧ�����ݿ��ȵȣ����ֲ��䣩
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);        // ʹ��ͨ��0�Զ�Ӧ����B�ش�ACK��
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);    // ʹ��ͨ��0���յ�ַ
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, CH_BC);       // ����ΪB-CƵ�� 40
    NRF24L01_Write_Reg(NRF_WRITE_REG + RX_PW_P0, DATA_WIDTH); // ���ݳ���32�ֽ�
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0f);     // 0db���棬2Mbps�����������濪��
    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);       // ����ģʽ��PWR_UP��EN_CRC
    
    NRF24L01_CE_HIGH();  // ����CE�������������״�
    HAL_Delay(100);   // ��ʱ��֤������Ч
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
