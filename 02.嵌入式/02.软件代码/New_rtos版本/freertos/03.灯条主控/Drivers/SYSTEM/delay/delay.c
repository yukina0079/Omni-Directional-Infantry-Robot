#include "delay.h"

void delay_us(uint32_t nus)
{
    uint32_t temp;
    SysTick->LOAD = 72 * nus;
    SysTick->VAL = 0x00;
    SysTick->CTRL |= 1 << 2;
    SysTick->CTRL |= 1 << 0;
    do
    {
        temp = SysTick->CTRL;
    } while ((temp & 0x01) && !(temp & (1 << 16)));
    SysTick->CTRL &= ~(1 << 0);
}

void delay_ms(uint32_t nms)
{
    while(nms--)
        delay_us(1000);
}

void delay_s(uint32_t ns)
{
    while(ns--)
        delay_ms(1000);
}

void HAL_Delay(uint32_t nms)
{
    delay_ms(nms);
}
