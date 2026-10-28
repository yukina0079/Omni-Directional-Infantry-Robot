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
#define IIC_HW_TIMEOUT             10000

#define IIC_HW_TIMEOUT_FAST        5

extern I2C_HandleTypeDef hi2c2;
extern volatile uint8_t iic_last_hal_status;
extern volatile uint32_t iic_last_error_code;

#define IIC_SCAN_LOG_MAGIC  0x53434E31u
#define IIC_SCAN_LOG_MAX    16u

typedef struct
{
    uint32_t magic;
    uint8_t idle_scl;
    uint8_t idle_sda;
    uint8_t found;
    uint8_t done;
    uint8_t addr[IIC_SCAN_LOG_MAX];
    uint8_t id[IIC_SCAN_LOG_MAX];
} iic_scan_log_t;

extern volatile iic_scan_log_t iic_scan_log;

typedef struct
{
    uint8_t pullup_20us_scl;
    uint8_t pullup_20us_sda;
    uint8_t pullup_1ms_scl;
    uint8_t pullup_1ms_sda;
    uint8_t nopull_1ms_scl;
    uint8_t nopull_1ms_sda;
} iic_line_test_t;

extern volatile iic_line_test_t iic_line_test;

void iic_init(void);
void iic_bus_recover(void);
uint8_t iic_device_ready(uint8_t i2c_addr);
void iic_scan(void);
void iic_write_register(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data);
uint8_t iic_read_register(uint8_t i2c_addr, uint8_t reg_addr);
uint8_t iic_read_len(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buf, uint8_t len);
uint8_t iic_write_len(uint8_t i2c_addr, uint8_t reg_addr, const uint8_t *buf, uint16_t len);

#endif
