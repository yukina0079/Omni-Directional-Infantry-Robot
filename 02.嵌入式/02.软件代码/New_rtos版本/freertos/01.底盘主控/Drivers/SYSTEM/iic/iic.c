#include "iic.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
/**
 * @brief       初始化IIC
 * @param       无
 * @retval      无
 */
void iic_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    IIC_SCL_GPIO_CLK_ENABLE();  /* SCL引脚时钟使能 */
    IIC_SDA_GPIO_CLK_ENABLE();  /* SDA引脚时钟使能 */

    gpio_init_struct.Pin = IIC_SCL_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;        /* 推挽输出 */
    gpio_init_struct.Pull = GPIO_PULLUP;                /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; /* 快速 */
    HAL_GPIO_Init(IIC_SCL_GPIO_PORT, &gpio_init_struct);/* SCL */

    gpio_init_struct.Pin = IIC_SDA_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_OD;        /* 开漏输出 */
    HAL_GPIO_Init(IIC_SDA_GPIO_PORT, &gpio_init_struct);/* SDA */
    /* SDA引脚模式设置,开漏输出,上拉, 这样就不用再设置IO方向了, 开漏输出的时候(=1), 也可以读取外部信号的高低电平 */

    iic_stop();     /* 停止总线上所有设备 */
}

/**
 * @brief       IIC延时函数,用于控制IIC读写速度
 * @param       无
 * @retval      无
 */
static void iic_delay(void)
{
    delay_us(0);    /* 2us的延时, 读写速度在250Khz以内 */
}

/**
 * @brief       产生IIC起始信号
 * @param       无
 * @retval      无
 */
void iic_start(void)
{
    IIC_SDA(1);
    IIC_SCL(1);
    iic_delay();
    IIC_SDA(0);     /* START信号: 当SCL为高时, SDA从高变成低, 表示起始信号 */
    iic_delay();
    IIC_SCL(0);     /* 钳住I2C总线，准备发送或接收数据 */
    iic_delay();
}

/**
 * @brief       产生IIC停止信号
 * @param       无
 * @retval      无
 */
void iic_stop(void)
{
    IIC_SDA(0);     /* STOP信号: 当SCL为高时, SDA从低变成高, 表示停止信号 */
    iic_delay();
    IIC_SCL(1);
    iic_delay();
    IIC_SDA(1);     /* 发送I2C总线结束信号 */
    iic_delay();
}

/**
 * @brief       等待应答信号到来
 * @param       无
 * @retval      1，接收应答失败
 *              0，接收应答成功
 */
uint8_t iic_wait_ack(void)
{
    uint8_t waittime = 0;
    uint8_t rack = 0;

    IIC_SDA(1);     /* 主机释放SDA线(此时外部器件可以拉低SDA线) */
    iic_delay();
    IIC_SCL(1);     /* SCL=1, 此时从机可以返回ACK */
    iic_delay();

    while (IIC_READ_SDA)    /* 等待应答 */
    {
        waittime++;

        if (waittime > 250)
        {
            iic_stop();
            rack = 1;
            break;
        }
    }

    IIC_SCL(0);     /* SCL=0, 结束ACK检查 */
    iic_delay();
    return rack;
}

/**
 * @brief       产生ACK应答
 * @param       无
 * @retval      无
 */
void iic_ack(void)
{
    IIC_SDA(0);     /* SCL 0 -> 1 时 SDA = 0,表示应答 */
    iic_delay();
    IIC_SCL(1);     /* 产生一个时钟 */
    iic_delay();
    IIC_SCL(0);
    iic_delay();
    IIC_SDA(1);     /* 主机释放SDA线 */
    iic_delay();
}

/**
 * @brief       不产生ACK应答
 * @param       无
 * @retval      无
 */
void iic_nack(void)
{
    IIC_SDA(1);     /* SCL 0 -> 1  时 SDA = 1,表示不应答 */
    iic_delay();
    IIC_SCL(1);     /* 产生一个时钟 */
    iic_delay();
    IIC_SCL(0);
    iic_delay();
}

/**
 * @brief       IIC发送一个字节
 * @param       data: 要发送的数据
 * @retval      无
 */
void iic_send_byte(uint8_t data)
{
    uint8_t t;
    
    for (t = 0; t < 8; t++)
    {
        IIC_SDA((data & 0x80) >> 7);    /* 高位先发送 */
        iic_delay();
        IIC_SCL(1);
        iic_delay();
        IIC_SCL(0);
        data <<= 1;     /* 左移1位,用于下一次发送 */
    }
    IIC_SDA(1);         /* 发送完成, 主机释放SDA线 */
}

/**
 * @brief       IIC读取一个字节
 * @param       ack:  ack=1时，发送ack; ack=0时，发送nack
 * @retval      接收到的数据
 */
uint8_t iic_read_byte(uint8_t ack)
{
    uint8_t i, receive = 0;

    for (i = 0; i < 8; i++ )    /* 接收1个字节数据 */
    {
        receive <<= 1;  /* 高位先输出,所以先收到的数据位要左移 */
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
        iic_nack();     /* 发送nACK */
    }
    else
    {
        iic_ack();      /* 发送ACK */
    }

    return receive;
}
void iic_write_register(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data)
{
    taskENTER_CRITICAL(); // 【加入临界区保护】禁止任务切换和中断

    iic_start();
    iic_send_byte((i2c_addr << 1) | 0x00);
    iic_wait_ack();
    iic_send_byte(reg_addr);
    iic_wait_ack();
    iic_send_byte(data);
    iic_wait_ack();
    iic_stop();

    taskEXIT_CRITICAL();  // 【退出临界区】恢复任务调度
}

/**
 * @brief       IIC读取寄存器数据
 * @param       i2c_addr: I2C设备地址
 * @param       reg_addr: 寄存器地址
 * @retval      读取到的数据
 */
uint8_t iic_read_register(uint8_t i2c_addr, uint8_t reg_addr)
{
    uint8_t data = 0;
    
    iic_start();                                    /* 起始信号 */
    
    iic_send_byte((i2c_addr << 1) | 0x00);          /* 发送设备地址(写模式) */
    if(iic_wait_ack())
    {
        iic_stop();
        return 0xFF;                                /* 返回错误值 */
    }
    
    iic_send_byte(reg_addr);                        /* 发送寄存器地址 */
    if(iic_wait_ack())
    {
        iic_stop();
        return 0xFF;                                /* 返回错误值 */
    }
    
    iic_start();                                    /* 重复起始信号 */
    
    iic_send_byte((i2c_addr << 1) | 0x01);          /* 发送设备地址(读模式) */
    if(iic_wait_ack())
    {
        iic_stop();
        return 0xFF;                                /* 返回错误值 */
    }
    
    data = iic_read_byte(0);                        /* 读取数据，发送NACK */
    
    iic_stop();                                     /* 停止信号 */
    
    return data;
}

/**
 * @brief       IIC连续读取多个字节
 * @param       i2c_addr: I2C设备地址
 * @param       reg_addr: 寄存器起始地址
 * @param       buf: 存放读取数据的缓冲区
 * @param       len: 读取长度
 * @retval      0, 成功; 1, 失败
 */
uint8_t iic_read_len(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buf, uint8_t len)
{
    iic_start();
    iic_send_byte((i2c_addr << 1) | 0);    /* 写模式 */
    if (iic_wait_ack()) { iic_stop(); return 1; }
    
    iic_send_byte(reg_addr);               /* 发送起始寄存器地址 */
    if (iic_wait_ack()) { iic_stop(); return 1; }
    
    iic_start();                           /* 重复起始信号 */
    iic_send_byte((i2c_addr << 1) | 1);    /* 读模式 */
    if (iic_wait_ack()) { iic_stop(); return 1; }
    
    while (len)
    {
        // 如果是最后一个字节，发送 nACK (0)，否则发送 ACK (1)
        *buf = iic_read_byte(len > 1 ? 1 : 0);
        buf++;
        len--;
    }
    
    iic_stop();
    return 0;
}





