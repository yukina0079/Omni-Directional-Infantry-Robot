#include "key.h"
#include "sys.h"
#include "delay.h"
#include "ws2812b.h"

/* --- 宏定义 --- */
#define KEY_PRESSED  GPIO_PIN_SET    // 按下 = 高电平
#define KEY_RELEASED GPIO_PIN_RESET  // 默认 = 低电平

/* --- 独立按键读取 --- */
#define KEY1_READ    (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12))
#define KEY2_READ    (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13))
#define KEY3_READ    (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14))
#define KEY4_READ    (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15))

/* --- 全局变量 --- */
uint8_t key_count = 0;
static uint8_t s_edge_mask = 0;

/* --- 独立按键状态 --- */
static GPIO_PinState key_prev[4] = {KEY_RELEASED}; // 独立保存上一次状态

/* --- 独立边沿检测 --- */
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
        // 下降沿：上一次低，当前高 → 按下
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

/* 按键初始化 */
void key_init(void)
{
    GPIO_InitTypeDef gpio_initstruct;
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // 常态低 → 下拉模式 PULLDOWN
    gpio_initstruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    gpio_initstruct.Mode = GPIO_MODE_INPUT;
    gpio_initstruct.Pull = GPIO_PULLDOWN;  // ? 常态低
    gpio_initstruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOB,&gpio_initstruct);

    // 初始状态全部为低
    key_prev[0] = KEY1_READ;
    key_prev[1] = KEY2_READ;
    key_prev[2] = KEY3_READ;
    key_prev[3] = KEY4_READ;
}

/* 按键扫描（独立触发，不并发） */
void key_scan(void)
{
    if(key_edge_detect())
    {
        key_count++;
    }
}
