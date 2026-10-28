#ifndef __IIC_H__
#define __IIC_H__
#include "uart1.h"
#include "delay.h"

#define I2C_SCL_CLK()      __HAL_RCC_GPIOB_CLK_ENABLE();
#define I2C_SCL_PORT       GPIOB
#define I2C_SCL2_PIN        GPIO_PIN_10

#define I2C_SDA_CLK()      __HAL_RCC_GPIOB_CLK_ENABLE();
#define I2C_SDA_PORT       GPIOB
#define I2C_SDA2_PIN        GPIO_PIN_11

#define I2C_SCL_RESET()        (I2C_SCL_PORT->BSRR = (uint32_t)I2C_SCL2_PIN << 16u)
#define I2C_SCL_SET()          (I2C_SCL_PORT->BSRR = (uint32_t)I2C_SCL2_PIN)

#define I2C_SDA_RESET()        (I2C_SDA_PORT->BSRR = (uint32_t)I2C_SDA2_PIN << 16u)
#define I2C_SDA_SET()          (I2C_SDA_PORT->BSRR = (uint32_t)I2C_SDA2_PIN)

#define I2C_SDA_VALUE()        ((I2C_SDA_PORT->IDR & I2C_SDA2_PIN) ? 1u : 0u)

#define I2C_OK      0u
#define I2C_ERR     1u

#define I2C_DELAY_TICKS_DEFAULT   10u

void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_send_byte(uint8_t data);
void i2c_send_ack(uint8_t ack);
uint8_t i2c_receive_byte(void);
uint8_t i2c_receive_ack(void);

void     i2c_set_delay_ticks(uint32_t ticks);
uint32_t i2c_get_delay_ticks(void);

uint32_t i2c_measure_delay_ns(void);

void i2c_bus_recover(void);

uint8_t i2c_write_register(uint8_t i2c_address, uint8_t address, uint8_t data);
uint8_t i2c_read_register(uint8_t i2c_address, uint8_t address, uint8_t *out);
uint8_t i2c_read_len(uint8_t i2c_address, uint8_t reg_addr, uint8_t len, uint8_t *data);

#endif
