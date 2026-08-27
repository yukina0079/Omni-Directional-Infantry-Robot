#ifndef __IIC_H__
#define __IIC_H__

#include "stm32f1xx_hal.h"  // 根据你的MCU型号调整（如f4xx/f7xx等）

/* 硬件I2C句柄声明 */
extern I2C_HandleTypeDef hi2c2;

/* 函数声明（保持原有接口完全兼容） */
void i2c_init(void);
void i2c_write_register(uint8_t i2c_address, uint8_t address, uint8_t data);
uint8_t i2c_read_register(uint8_t i2c_address, uint8_t address);
void i2c_write_len(uint8_t i2c_address, uint8_t reg_addr, uint8_t len, uint8_t *data);
void i2c_read_len(uint8_t i2c_address, uint8_t reg_addr, uint8_t len, uint8_t *data);

/* 可选：调试用错误码定义 */
#define I2C_OK       0
#define I2C_ERROR    1

#endif
