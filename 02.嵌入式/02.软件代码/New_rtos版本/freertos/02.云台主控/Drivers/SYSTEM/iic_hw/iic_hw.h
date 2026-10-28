#ifndef __MYIIC_HW_H
#define __MYIIC_HW_H

#include "./SYSTEM/sys/sys.h"
#include "FreeRTOS.h"
#include "task.h"

#define IIC_HW_I2Cx                I2C2
#define IIC_HW_CLK_ENABLE()        __HAL_RCC_I2C2_CLK_ENABLE()
#define IIC_HW_GPIO_PORT           GPIOB
#define IIC_HW_SCL_PIN             GPIO_PIN_10
#define IIC_HW_SDA_PIN             GPIO_PIN_11
#define IIC_HW_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define IIC_HW_TIMEOUT             5

extern I2C_HandleTypeDef hi2c2;

void iic_init(void);
void iic_write_register(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data);
uint8_t iic_read_register(uint8_t i2c_addr, uint8_t reg_addr);
uint8_t iic_read_len(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buf, uint8_t len);

#endif
