#include "key.h"
#include "sys.h"
#include "delay.h"
#include "ws2812b.h"

#define KEY_PRESSED  GPIO_PIN_SET
#define KEY_RELEASED GPIO_PIN_RESET

#define KEY1_READ    (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12))
#define KEY2_READ    (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13))
#define KEY3_READ    (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14))
#define KEY4_READ    (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15))

uint8_t key_count = 0;
static uint8_t s_edge_mask = 0;

static GPIO_PinState key_prev[4] = {KEY_RELEASED};

bool key_edge_detect(void)
{
    uint8_t i;
    bool edge = false;
    GPIO_PinState curr[4];
    s_edge_mask = 0;

    curr[0] = KEY1_READ;
    curr[1] = KEY2_READ;
    curr[2] = KEY3_READ;
    curr[3] = KEY4_READ;

    for(i=0; i<4; i++)
    {

        if(key_prev[i] == KEY_RELEASED && curr[i] == KEY_PRESSED)
        {
            edge = true;
            s_edge_mask |= (uint8_t)(1u << i);
        }
        key_prev[i] = curr[i];
    }
    return edge;
}

uint8_t key_edge_mask(void)
{
    return s_edge_mask;
}

void key_init(void)
{
    GPIO_InitTypeDef gpio_initstruct;
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio_initstruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    gpio_initstruct.Mode = GPIO_MODE_INPUT;
    gpio_initstruct.Pull = GPIO_PULLDOWN;
    gpio_initstruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOB,&gpio_initstruct);

    key_prev[0] = KEY1_READ;
    key_prev[1] = KEY2_READ;
    key_prev[2] = KEY3_READ;
    key_prev[3] = KEY4_READ;
}

void key_scan(void)
{
    if(key_edge_detect())
    {
        key_count++;
    }
}
