#include "iic.h"
#include "systime.h"

static volatile uint32_t s_delay_ticks = I2C_DELAY_TICKS_DEFAULT;

static void i2c_delay(void)
{
    volatile uint32_t i = s_delay_ticks;
    while (i--) {

    }
}

void i2c_set_delay_ticks(uint32_t ticks)
{

    s_delay_ticks = (ticks < 1u) ? 1u : ticks;
}

uint32_t i2c_get_delay_ticks(void)
{
    return s_delay_ticks;
}

uint32_t i2c_measure_delay_ns(void)
{
    const uint32_t reps = 10000u;
    uint32_t i, t0, t1;

    t0 = micros();
    for (i = 0; i < reps; i++) {
        i2c_delay();
    }
    t1 = micros();

    return ((t1 - t0) * 1000u) / reps;
}

void i2c_bus_recover(void)
{
    uint8_t i;

    I2C_SDA_SET();
    for (i = 0; i < 9u; i++) {
        I2C_SCL_SET();
        i2c_delay();
        I2C_SCL_RESET();
        i2c_delay();
    }
    i2c_stop();
}

void i2c_init(void)
{
    GPIO_InitTypeDef gpio_initstruct;

    I2C_SCL_CLK();
    I2C_SDA_CLK();

    gpio_initstruct.Pin   = I2C_SCL2_PIN | I2C_SDA2_PIN;
    gpio_initstruct.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio_initstruct.Pull  = GPIO_NOPULL;
    gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio_initstruct);

    I2C_SCL_SET();
    I2C_SDA_SET();
    i2c_delay();
}

void i2c_start(void)
{

    I2C_SDA_SET();
    I2C_SCL_SET();
    i2c_delay();
    I2C_SDA_RESET();
    i2c_delay();
    I2C_SCL_RESET();
    i2c_delay();
}

void i2c_stop(void)
{

    I2C_SCL_RESET();
    I2C_SDA_RESET();
    i2c_delay();
    I2C_SCL_SET();
    i2c_delay();
    I2C_SDA_SET();
    i2c_delay();
}

void i2c_send_byte(uint8_t data)
{
    uint8_t i;

    for (i = 0; i < 8; i++) {

        if (data & 0x80u) {
            I2C_SDA_SET();
        } else {
            I2C_SDA_RESET();
        }
        data <<= 1;

        i2c_delay();
        I2C_SCL_SET();
        i2c_delay();
        I2C_SCL_RESET();
        i2c_delay();
    }

    I2C_SDA_SET();
}

uint8_t i2c_receive_byte(void)
{
    uint8_t i;
    uint8_t data = 0;

    I2C_SDA_SET();

    for (i = 0; i < 8; i++) {
        data <<= 1;
        I2C_SCL_RESET();
        i2c_delay();
        I2C_SCL_SET();
        i2c_delay();
        if (I2C_SDA_VALUE()) {
            data |= 0x01u;
        }
    }

    I2C_SCL_RESET();
    i2c_delay();
    return data;
}

uint8_t i2c_receive_ack(void)
{
    uint8_t ack;

    I2C_SDA_SET();
    i2c_delay();
    I2C_SCL_SET();
    i2c_delay();

    ack = I2C_SDA_VALUE() ? I2C_ERR : I2C_OK;

    I2C_SCL_RESET();
    i2c_delay();
    return ack;
}

void i2c_send_ack(uint8_t ack)
{
    if (ack) {
        I2C_SDA_SET();
    } else {
        I2C_SDA_RESET();
    }
    i2c_delay();
    I2C_SCL_SET();
    i2c_delay();
    I2C_SCL_RESET();
    i2c_delay();
    I2C_SDA_SET();
}

uint8_t i2c_write_register(uint8_t i2c_address, uint8_t address, uint8_t data)
{
    i2c_start();

    i2c_send_byte(i2c_address << 1);
    if (i2c_receive_ack() != I2C_OK) { i2c_stop(); return I2C_ERR; }

    i2c_send_byte(address);
    if (i2c_receive_ack() != I2C_OK) { i2c_stop(); return I2C_ERR; }

    i2c_send_byte(data);
    if (i2c_receive_ack() != I2C_OK) { i2c_stop(); return I2C_ERR; }

    i2c_stop();
    return I2C_OK;
}

uint8_t i2c_read_register(uint8_t i2c_address, uint8_t address, uint8_t *out)
{
    return i2c_read_len(i2c_address, address, 1, out);
}

uint8_t i2c_read_len(uint8_t i2c_address, uint8_t reg_addr, uint8_t len, uint8_t *data)
{
    uint8_t i;

    if (len == 0 || data == 0) {
        return I2C_ERR;
    }

    i2c_start();

    i2c_send_byte(i2c_address << 1);
    if (i2c_receive_ack() != I2C_OK) { i2c_stop(); return I2C_ERR; }

    i2c_send_byte(reg_addr);
    if (i2c_receive_ack() != I2C_OK) { i2c_stop(); return I2C_ERR; }

    i2c_start();

    i2c_send_byte((i2c_address << 1) | 0x01u);
    if (i2c_receive_ack() != I2C_OK) { i2c_stop(); return I2C_ERR; }

    for (i = 0; i < len; i++) {
        data[i] = i2c_receive_byte();

        i2c_send_ack((i == len - 1) ? 1u : 0u);
    }

    i2c_stop();
    return I2C_OK;
}
