#ifndef __MYIIC_HW_H
#define __MYIIC_HW_H

#include "./SYSTEM/sys/sys.h"  // ����ԭ��ϵͳͷ�ļ�
#include "FreeRTOS.h"
#include "task.h"

/************************ Ӳ��IIC���� ************************/
#define IIC_HW_I2Cx                I2C2                // ʹ��I2C2����
#define IIC_HW_CLK_ENABLE()        __HAL_RCC_I2C2_CLK_ENABLE()  // I2C2ʱ��ʹ��
#define IIC_HW_GPIO_PORT           GPIOB
#define IIC_HW_SCL_PIN             GPIO_PIN_10
#define IIC_HW_SDA_PIN             GPIO_PIN_11
#define IIC_HW_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()  // GPIOBʱ��ʹ��
#define IIC_HW_TIMEOUT             5

/* I2C������� */
extern I2C_HandleTypeDef hi2c2;

/************************ ����ԭģ��IIC�ĺ����ӿ� ************************/
void iic_init(void);                                    // �滻ԭģ��IIC��ʼ��
void iic_write_register(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data); // ����ԭ������
uint8_t iic_read_register(uint8_t i2c_addr, uint8_t reg_addr);             // ����ԭ������
uint8_t iic_read_len(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buf, uint8_t len); // ����ԭ������

#endif

/*
// ���Դ��루��ԭģ��IIC���÷�ʽ��ȫһ�£�
int main(void)
{
    HAL_Init();
    sys_stm32_clock_init(336, 8, 2, 7); // ��ʼ��ʱ�ӣ�����MCU�޸ģ�
    delay_init(168);
    iic_init();  // Ӳ��IIC��ʼ�����滻ԭģ��IIC��
    
    // д����ԣ���0x48�豸��0x10�Ĵ���д0x55
    iic_write_register(0x48, 0x10, 0x55);
    
    // ��ȡ���ԣ���0x48�豸��0x10�Ĵ���������
    uint8_t data = iic_read_register(0x48, 0x10);
    
    // ���ֽڶ�ȡ����
    uint8_t buf[5];
    iic_read_len(0x48, 0x20, buf, 5);
    
    while(1);
}
*/

