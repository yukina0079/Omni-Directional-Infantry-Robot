#include "iic.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
/**
 * @brief       ��ʼ��IIC
 * @param       ��
 * @retval      ��
 */
void iic_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    IIC_SCL_GPIO_CLK_ENABLE();  /* SCL����ʱ��ʹ�� */
    IIC_SDA_GPIO_CLK_ENABLE();  /* SDA����ʱ��ʹ�� */

    gpio_init_struct.Pin = IIC_SCL_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;        /* ������� */
    gpio_init_struct.Pull = GPIO_PULLUP;                /* ���� */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; /* ���� */
    HAL_GPIO_Init(IIC_SCL_GPIO_PORT, &gpio_init_struct);/* SCL */

    gpio_init_struct.Pin = IIC_SDA_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_OD;        /* ��©��� */
    HAL_GPIO_Init(IIC_SDA_GPIO_PORT, &gpio_init_struct);/* SDA */
    /* SDA����ģʽ����,��©���,����, �����Ͳ���������IO������, ��©�����ʱ��(=1), Ҳ���Զ�ȡ�ⲿ�źŵĸߵ͵�ƽ */

    iic_stop();     /* ֹͣ�����������豸 */
}

/**
 * @brief       IIC��ʱ����,���ڿ���IIC��д�ٶ�
 * @param       ��
 * @retval      ��
 */
static void iic_delay(void)
{
    delay_us(0);    /* 2us����ʱ, ��д�ٶ���250Khz���� */
}

/**
 * @brief       ����IIC��ʼ�ź�
 * @param       ��
 * @retval      ��
 */
void iic_start(void)
{
    IIC_SDA(1);
    IIC_SCL(1);
    iic_delay();
    IIC_SDA(0);     /* START�ź�: ��SCLΪ��ʱ, SDA�Ӹ߱�ɵ�, ��ʾ��ʼ�ź� */
    iic_delay();
    IIC_SCL(0);     /* ǯסI2C���ߣ�׼�����ͻ�������� */
    iic_delay();
}

/**
 * @brief       ����IICֹͣ�ź�
 * @param       ��
 * @retval      ��
 */
void iic_stop(void)
{
    IIC_SDA(0);     /* STOP�ź�: ��SCLΪ��ʱ, SDA�ӵͱ�ɸ�, ��ʾֹͣ�ź� */
    iic_delay();
    IIC_SCL(1);
    iic_delay();
    IIC_SDA(1);     /* ����I2C���߽����ź� */
    iic_delay();
}

/**
 * @brief       �ȴ�Ӧ���źŵ���
 * @param       ��
 * @retval      1������Ӧ��ʧ��
 *              0������Ӧ��ɹ�
 */
uint8_t iic_wait_ack(void)
{
    uint8_t waittime = 0;
    uint8_t rack = 0;

    IIC_SDA(1);     /* �����ͷ�SDA��(��ʱ�ⲿ������������SDA��) */
    iic_delay();
    IIC_SCL(1);     /* SCL=1, ��ʱ�ӻ����Է���ACK */
    iic_delay();

    while (IIC_READ_SDA)    /* �ȴ�Ӧ�� */
    {
        waittime++;

        if (waittime > 250)
        {
            iic_stop();
            rack = 1;
            break;
        }
    }

    IIC_SCL(0);     /* SCL=0, ����ACK��� */
    iic_delay();
    return rack;
}

/**
 * @brief       ����ACKӦ��
 * @param       ��
 * @retval      ��
 */
void iic_ack(void)
{
    IIC_SDA(0);     /* SCL 0 -> 1 ʱ SDA = 0,��ʾӦ�� */
    iic_delay();
    IIC_SCL(1);     /* ����һ��ʱ�� */
    iic_delay();
    IIC_SCL(0);
    iic_delay();
    IIC_SDA(1);     /* �����ͷ�SDA�� */
    iic_delay();
}

/**
 * @brief       ������ACKӦ��
 * @param       ��
 * @retval      ��
 */
void iic_nack(void)
{
    IIC_SDA(1);     /* SCL 0 -> 1  ʱ SDA = 1,��ʾ��Ӧ�� */
    iic_delay();
    IIC_SCL(1);     /* ����һ��ʱ�� */
    iic_delay();
    IIC_SCL(0);
    iic_delay();
}

/**
 * @brief       IIC����һ���ֽ�
 * @param       data: Ҫ���͵�����
 * @retval      ��
 */
void iic_send_byte(uint8_t data)
{
    uint8_t t;
    
    for (t = 0; t < 8; t++)
    {
        IIC_SDA((data & 0x80) >> 7);    /* ��λ�ȷ��� */
        iic_delay();
        IIC_SCL(1);
        iic_delay();
        IIC_SCL(0);
        data <<= 1;     /* ����1λ,������һ�η��� */
    }
    IIC_SDA(1);         /* �������, �����ͷ�SDA�� */
}

/**
 * @brief       IIC��ȡһ���ֽ�
 * @param       ack:  ack=1ʱ������ack; ack=0ʱ������nack
 * @retval      ���յ�������
 */
uint8_t iic_read_byte(uint8_t ack)
{
    uint8_t i, receive = 0;

    for (i = 0; i < 8; i++ )    /* ����1���ֽ����� */
    {
        receive <<= 1;  /* ��λ�����,�������յ�������λҪ���� */
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
        iic_nack();     /* ����nACK */
    }
    else
    {
        iic_ack();      /* ����ACK */
    }

    return receive;
}
void iic_write_register(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data)
{
    taskENTER_CRITICAL(); // �������ٽ�����������ֹ�����л����ж�

    iic_start();
    iic_send_byte((i2c_addr << 1) | 0x00);
    iic_wait_ack();
    iic_send_byte(reg_addr);
    iic_wait_ack();
    iic_send_byte(data);
    iic_wait_ack();
    iic_stop();

    taskEXIT_CRITICAL();  // ���˳��ٽ������ָ��������
}

/**
 * @brief       IIC��ȡ�Ĵ�������
 * @param       i2c_addr: I2C�豸��ַ
 * @param       reg_addr: �Ĵ�����ַ
 * @retval      ��ȡ��������
 */
uint8_t iic_read_register(uint8_t i2c_addr, uint8_t reg_addr)
{
    uint8_t data = 0;
    
    iic_start();                                    /* ��ʼ�ź� */
    
    iic_send_byte((i2c_addr << 1) | 0x00);          /* �����豸��ַ(дģʽ) */
    if(iic_wait_ack())
    {
        iic_stop();
        return 0xFF;                                /* ���ش���ֵ */
    }
    
    iic_send_byte(reg_addr);                        /* ���ͼĴ�����ַ */
    if(iic_wait_ack())
    {
        iic_stop();
        return 0xFF;                                /* ���ش���ֵ */
    }
    
    iic_start();                                    /* �ظ���ʼ�ź� */
    
    iic_send_byte((i2c_addr << 1) | 0x01);          /* �����豸��ַ(��ģʽ) */
    if(iic_wait_ack())
    {
        iic_stop();
        return 0xFF;                                /* ���ش���ֵ */
    }
    
    data = iic_read_byte(0);                        /* ��ȡ���ݣ�����NACK */
    
    iic_stop();                                     /* ֹͣ�ź� */
    
    return data;
}

/**
 * @brief       IIC������ȡ����ֽ�
 * @param       i2c_addr: I2C�豸��ַ
 * @param       reg_addr: �Ĵ�����ʼ��ַ
 * @param       buf: ��Ŷ�ȡ���ݵĻ�����
 * @param       len: ��ȡ����
 * @retval      0, �ɹ�; 1, ʧ��
 */
uint8_t iic_read_len(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buf, uint8_t len)
{
    uint8_t ret = 0;

    taskENTER_CRITICAL();

    iic_start();
    iic_send_byte((i2c_addr << 1) | 0);    /* write */
    if (iic_wait_ack()) { iic_stop(); ret = 1; goto out; }

    iic_send_byte(reg_addr);
    if (iic_wait_ack()) { iic_stop(); ret = 1; goto out; }

    iic_start();
    iic_send_byte((i2c_addr << 1) | 1);    /* read */
    if (iic_wait_ack()) { iic_stop(); ret = 1; goto out; }

    while (len)
    {
        *buf = iic_read_byte(len > 1 ? 1 : 0);
        buf++;
        len--;
    }

    iic_stop();

out:
    taskEXIT_CRITICAL();
    return ret;
}





