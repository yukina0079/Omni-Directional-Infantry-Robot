#ifndef __MYIIC_HW_H
#define __MYIIC_HW_H

#include "./SYSTEM/sys/sys.h"  // 复用原有系统头文件
#include "FreeRTOS.h"
#include "task.h"

/************************ 硬件IIC配置 ************************/
#define IIC_HW_I2Cx                I2C2                // 使用I2C2外设
#define IIC_HW_CLK_ENABLE()        __HAL_RCC_I2C2_CLK_ENABLE()  // I2C2时钟使能
#define IIC_HW_GPIO_PORT           GPIOB
#define IIC_HW_SCL_PIN             GPIO_PIN_10
#define IIC_HW_SDA_PIN             GPIO_PIN_11
#define IIC_HW_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()  // GPIOB时钟使能
#define IIC_HW_TIMEOUT             10000              // Init only: the 8192-byte IMU660RA config upload
/*
 * Runtime read path, milliseconds.
 *
 * One 12-byte burst at 400 kHz takes about 340 us, so 5 ms is roughly 15x
 * margin. It has to be short because iic_read_len() is called from
 * data_get_task, the highest-priority task: every millisecond spent blocking
 * in here is a millisecond the radio failsafe and the FOC frame stream do not
 * run. At the original 10000 a stuck bus froze the whole chassis for ten
 * seconds while the wheels held their last PWM.
 *
 * The long timeout above is still correct for the bulk config write, which
 * legitimately takes about 184 ms at this clock, and for the init-time probes.
 */
#define IIC_HW_TIMEOUT_FAST        5

/* I2C句柄声明 */
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

/* Power-on line samples captured before the address scan. */
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

/************************ 兼容原模拟IIC的函数接口 ************************/
void iic_init(void);                                    // 替换原模拟IIC初始化
void iic_bus_recover(void);
uint8_t iic_device_ready(uint8_t i2c_addr);
void iic_scan(void);
void iic_write_register(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data); // 兼容原函数名
uint8_t iic_read_register(uint8_t i2c_addr, uint8_t reg_addr);             // 兼容原函数名
uint8_t iic_read_len(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buf, uint8_t len); // 兼容原函数名
uint8_t iic_write_len(uint8_t i2c_addr, uint8_t reg_addr, const uint8_t *buf, uint16_t len);

#endif

/*
// 测试代码（与原模拟IIC调用方式完全一致）
int main(void)
{
    HAL_Init();
    sys_stm32_clock_init(336, 8, 2, 7); // 初始化时钟（根据MCU修改）
    delay_init(168);
    iic_init();  // 硬件IIC初始化（替换原模拟IIC）
    
    // 写入测试：向0x48设备的0x10寄存器写0x55
    iic_write_register(0x48, 0x10, 0x55);
    
    // 读取测试：从0x48设备的0x10寄存器读数据
    uint8_t data = iic_read_register(0x48, 0x10);
    
    // 多字节读取测试
    uint8_t buf[5];
    iic_read_len(0x48, 0x20, buf, 5);
    
    while(1);
}
*/

