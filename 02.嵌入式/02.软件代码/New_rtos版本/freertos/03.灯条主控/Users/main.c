#include "sys.h"
#include "uart1.h"
#include "delay.h"
#include "LED.h"
#include "key.h"
#include "pwm.h"
#include "ws2812b.h"
#include "nrf24L01.h"
#include "can.h"
#include "oled.h"
extern uint8_t key_count;
char buf[20];
uint8_t Receive[32]={0};

#define ARMOR_COUNT        4u
#define FLASH_INTERVAL_MS  100u
#define FLASH_TIMES        3u
#define FLASH_STEPS        (FLASH_TIMES * 2u)

static uint8_t  s_flash_step[ARMOR_COUNT] = {FLASH_STEPS, FLASH_STEPS, FLASH_STEPS, FLASH_STEPS};
static uint32_t s_flash_tick[ARMOR_COUNT];

int main(void)
{
    HAL_Init();                    
    stm32_clock_init(RCC_PLL_MUL9);
	
	lde_init();
	key_init();
	dma_init();
//	oled_init();
	NRF24L01_Init();
	pwm_init(90-1,0);
    uart1_init(115200);

	printf("running...\r\n");
	
	nrf2401_chack();	
	lde2_on();

	WS2812_SetALL(255,0,0);
	WS2812_Updata();
	delay_ms(10);
	HAL_InitTick(TICK_INT_PRIORITY);

    while(1)
    {
		uint8_t i;
		uint8_t edges;
		uint32_t now;
		uint8_t team_r;
		uint8_t team_g;
		uint8_t team_b;
		uint8_t strip_r;
		uint8_t strip_g;
		uint8_t strip_b;

		key_scan();
		(void)NRF24L01_RxPacket(Receive);
		edges = key_edge_mask();
		now = HAL_GetTick();

		if ((Receive[1] & 0x08) != 0x08) {
			team_r = 255;
			team_g = 0;
			team_b = 0;
		} else {
			team_r = 0;
			team_g = 0;
			team_b = 255;
		}

		for (i = 0; i < ARMOR_COUNT; i++) {
			if (((edges >> i) & 1u) != 0u) {
				if (s_flash_step[i] >= FLASH_STEPS) {
					s_flash_step[i] = 0;
					s_flash_tick[i] = now;
				}
			}

			if (s_flash_step[i] < FLASH_STEPS) {
				if ((now - s_flash_tick[i]) >= FLASH_INTERVAL_MS) {
					s_flash_tick[i] = now;
					s_flash_step[i]++;
				}
			}

			if ((s_flash_step[i] < FLASH_STEPS) && ((s_flash_step[i] & 1u) == 0u)) {
				strip_r = 0;
				strip_g = 0;
				strip_b = 0;
			} else {
				strip_r = team_r;
				strip_g = team_g;
				strip_b = team_b;
			}
			WS2812_FillStrip(i, strip_r, strip_g, strip_b);
		}

		WS2812_FillStrip(4, team_r, team_g, team_b);
		WS2812_Updata();
		delay_ms(10);
	}
}
/*		
1.ws2812测试	
	
		WS2812_SetALL(255,0,0);
		WS2812_Updata();
		delay_ms(1000);
		WS2812_SetALL(0,255,0);
		WS2812_Updata();
		delay_ms(1000);			
		WS2812_SetALL(0,0,255);
		WS2812_Updata();
		delay_ms(1000);	
2.掉血测试
		
		if(key_edge_detect() == true){

		key_num++; // 每检测到一次下降沿，count加1
		
		if(key_num<10){
			WS2812_SetALL(0,0,255);
			WS2812_Updata();
			delay_ms(10);
			
		}else{
			WS2812_SetALL(0,0,0);
			WS2812_Updata();
			delay_ms(10);
		}
    }
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12) == GPIO_PIN_RESET){ 
			key_num  = 0;
			WS2812_SetALL(0,0,255);
			WS2812_Updata();
			delay_ms(10);
	}
3.NRF24L01无线通讯
初始化
	 while(NRF24L01_Check()){
			printf("硬件查寻不到NRF24L01无线模块\n"); 
			delay_ms(1000);
		}
			printf("NRF24L01无线模块硬件连接正常\r\n");

	
		NRF24L01_TX_Mode();//设置为发送模式
		printf("进入数据发送模式，每1s发送一次数据\r\n");
		
		NRF24L01_RX_Mode();
		printf("进入数据接收模式\n");	
循环
			//发送端   
		 if(NRF24L01_TxPacket(Send_Out)==TX_OK){
				printf("NRF24L01无线模块数据发送成功：%s\r\n",Send_Out);
			}else{
				printf("NRF24L01无线模块数据发送失败\r\n");
			} 

			//接收端
      if(NRF24L01_RxPacket(Receive)==0){
      Receive[32]=0;//加入字符串结束符      
      printf("NRF24L01无线模块数据接收成功：%s\r\n",Receive);
    }	
	
	*/

