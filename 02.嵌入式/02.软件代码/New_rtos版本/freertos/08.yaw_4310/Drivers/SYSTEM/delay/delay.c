#include "delay.h"
#include "systime.h"

void delay_us(uint32_t nus)
{
    uint32_t start = micros();

    while (micros_since(start) < nus) {

    }
}

void delay_ms(uint32_t nms)
{

    while (nms--) {
        delay_us(1000);
    }
}

void delay_s(uint32_t ns)
{
    while (ns--) {
        delay_ms(1000);
    }
}
