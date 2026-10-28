#include "sys.h"
#include "frame13.h"
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
uint8_t Receive[FRAME13_LEN]={0};

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
