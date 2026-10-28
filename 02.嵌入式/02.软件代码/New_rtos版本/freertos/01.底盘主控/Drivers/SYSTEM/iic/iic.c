#include "iic.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"

void iic_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    IIC_SCL_GPIO_CLK_ENABLE();
    IIC_SDA_GPIO_CLK_ENABLE();

    gpio_init_struct.Pin = IIC_SCL_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(IIC_SCL_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = IIC_SDA_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_OD;
    HAL_GPIO_Init(IIC_SDA_GPIO_PORT, &gpio_init_struct);

    iic_stop();
}

static void iic_delay(void)
{
    delay_us(0);
}

void iic_start(void)
{
    IIC_SDA(1);
    IIC_SCL(1);
    iic_delay();
    IIC_SDA(0);
    iic_delay();
    IIC_SCL(0);
    iic_delay();
}

void iic_stop(void)
{
    IIC_SDA(0);
    iic_delay();
    IIC_SCL(1);
    iic_delay();
    IIC_SDA(1);
    iic_delay();
}

uint8_t iic_wait_ack(void)
{
    uint8_t waittime = 0;
    uint8_t rack = 0;

    IIC_SDA(1);
    iic_delay();
    IIC_SCL(1);
    iic_delay();

    while (IIC_READ_SDA)
    {
        waittime++;

        if (waittime > 250)
        {
            iic_stop();
            rack = 1;
            break;
        }
    }

    IIC_SCL(0);
    iic_delay();
    return rack;
}

void iic_ack(void)
{
    IIC_SDA(0);
    iic_delay();
    IIC_SCL(1);
    iic_delay();
    IIC_SCL(0);
    iic_delay();
    IIC_SDA(1);
    iic_delay();
}

void iic_nack(void)
{
    IIC_SDA(1);
    iic_delay();
    IIC_SCL(1);
    iic_delay();
    IIC_SCL(0);
    iic_delay();
}

void iic_send_byte(uint8_t data)
{
    uint8_t t;

    for (t = 0; t < 8; t++)
    {
        IIC_SDA((data & 0x80) >> 7);
        iic_delay();
        IIC_SCL(1);
        iic_delay();
        IIC_SCL(0);
        data <<= 1;
    }
    IIC_SDA(1);
}

uint8_t iic_read_byte(uint8_t ack)
{
    uint8_t i, receive = 0;

    for (i = 0; i < 8; i++ )
    {
        receive <<= 1;
        IIC_SCL(1);
        iic_delay();

        if (IIC_READ_SDA)
        {
            receive++;
        }

        IIC_SCL(0);
        iic_delay();
    }

    if (!ack)
    {
        iic_nack();
    }
    else
    {
        iic_ack();
    }

    return receive;
}
void iic_write_register(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data)
{
    taskENTER_CRITICAL();

    iic_start();
    iic_send_byte((i2c_addr << 1) | 0x00);
    iic_wait_ack();
    iic_send_byte(reg_addr);
    iic_wait_ack();
    iic_send_byte(data);
    iic_wait_ack();
    iic_stop();

    taskEXIT_CRITICAL();
}

uint8_t iic_read_register(uint8_t i2c_addr, uint8_t reg_addr)
{
    uint8_t data = 0;

    iic_start();

    iic_send_byte((i2c_addr << 1) | 0x00);
    if(iic_wait_ack())
    {
        iic_stop();
        return 0xFF;
    }

    iic_send_byte(reg_addr);
    if(iic_wait_ack())
    {
        iic_stop();
        return 0xFF;
    }

    iic_start();

    iic_send_byte((i2c_addr << 1) | 0x01);
    if(iic_wait_ack())
    {
        iic_stop();
        return 0xFF;
    }

    data = iic_read_byte(0);

    iic_stop();

    return data;
}

uint8_t iic_read_len(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buf, uint8_t len)
{
    iic_start();
    iic_send_byte((i2c_addr << 1) | 0);
    if (iic_wait_ack()) { iic_stop(); return 1; }

    iic_send_byte(reg_addr);
    if (iic_wait_ack()) { iic_stop(); return 1; }

    iic_start();
    iic_send_byte((i2c_addr << 1) | 1);
    if (iic_wait_ack()) { iic_stop(); return 1; }

    while (len)
    {

        *buf = iic_read_byte(len > 1 ? 1 : 0);
        buf++;
        len--;
    }

    iic_stop();
    return 0;
}
