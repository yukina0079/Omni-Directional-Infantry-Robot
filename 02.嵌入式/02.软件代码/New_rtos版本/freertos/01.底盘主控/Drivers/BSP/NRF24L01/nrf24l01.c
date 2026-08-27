#include "spi.h"
#include "nrf24l01.h"
#include "delay.h"
#include "led.h"
#define ADDR_AB_LEN    5   // 与库函数TX_ADR_WIDTH=5一致
#define ADDR_BC_LEN    5   // 与库函数TX_ADR_WIDTH=5一致
#define DATA_WIDTH     13  // 与库函数TX_PLOAD_WIDTH=32一致

// A→B 通信参数（A、B模块共用）
uint8_t ADDR_AB[ADDR_AB_LEN] = {0x11, 0x22, 0x33, 0x44, 0x55};
#define CH_AB          30  // A-B通信频道

// B→C 通信参数（B、C模块共用）
uint8_t ADDR_BC[ADDR_BC_LEN] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE};
#define CH_BC          40  // B-C通信频道
 
extern SPI_HandleTypeDef g_spi2_handler;                /* SPI1句柄 */
const uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0x34, 0x43, 0x10, 0x10, 0x01};    /* 接收地址 */
const uint8_t RX_ADDRESS[RX_ADR_WIDTH] = {0x34, 0x43, 0x10, 0x10, 0x01};    /* 发送地址 */

/**
 * @brief       针对NRF24L01修改SPI1驱动
 * @param       无
 * @retval      无
 */
void nrf24l01_spi_init(void)
{
    __HAL_SPI_DISABLE(&g_spi2_handler);                 /* 先关闭SPI1 */
    g_spi2_handler.Init.CLKPolarity = SPI_POLARITY_LOW; /* 串行同步时钟的空闲状态为低电平 */
    g_spi2_handler.Init.CLKPhase = SPI_PHASE_1EDGE;     /* 串行同步时钟的第1个跳变沿（上升或下降）数据被采样 */
    HAL_SPI_Init(&g_spi2_handler);
    __HAL_SPI_ENABLE(&g_spi2_handler);                  /* 使能SPI1 */
}

/**
 * @brief       初始化24L01的IO口
 *   @note      将SPI1模式改成SCK空闲低电平,及SPI 模式0
 * @param       无
 * @retval      无
 */
void nrf24l01_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    NRF24L01_CE_GPIO_CLK_ENABLE();  /* CE脚时钟使能 */
    NRF24L01_CSN_GPIO_CLK_ENABLE(); /* CSN脚时钟使能 */
    NRF24L01_IRQ_GPIO_CLK_ENABLE(); /* IRQ脚时钟使能 */

    gpio_init_struct.Pin = NRF24L01_CE_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;             /* 推挽输出 */
    gpio_init_struct.Pull = GPIO_PULLUP;                     /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;           /* 高速 */
    HAL_GPIO_Init(NRF24L01_CE_GPIO_PORT, &gpio_init_struct); /* 初始化CE引脚 */

    gpio_init_struct.Pin = NRF24L01_CSN_GPIO_PIN;
    HAL_GPIO_Init(NRF24L01_CSN_GPIO_PORT, &gpio_init_struct);/* 初始化CSN引脚 */

    gpio_init_struct.Pin = NRF24L01_IRQ_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;                 /* 输入 */
    gpio_init_struct.Pull = GPIO_PULLUP;                     /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;           /* 高速 */
    HAL_GPIO_Init(NRF24L01_IRQ_GPIO_PORT, &gpio_init_struct);/* 初始化CE引脚 */

    spi2_init();                /* 初始化SPI1 */
    nrf24l01_spi_init();        /* 针对NRF的特点修改SPI的设置 */
    NRF24L01_CE(0);             /* 使能24L01 */
    NRF24L01_CSN(1);            /* SPI片选取消 */
}

/**
 * @brief       检测24L01是否存在
 * @param       无
 * @retval      0, 成功; 1, 失败;
 */
uint8_t nrf24l01_check(void)
{
    uint8_t buf[5] = {0XA5, 0XA5, 0XA5, 0XA5, 0XA5};
    uint8_t i;
    
    spi2_set_speed(SPI_SPEED_32);                         /* spi速度为7.5Mhz（24L01的最大SPI时钟为10Mhz） */
    nrf24l01_write_buf(NRF_WRITE_REG + TX_ADDR, buf, 5);  /* 写入5个字节的地址. */
    nrf24l01_read_buf(TX_ADDR, buf, 5);                   /* 读出写入的地址 */

    for (i = 0; i < 5; i++)
    {
        if (buf[i] != 0XA5) break;
    }
    
    if (i != 5) return 1;   /* 检测24L01错误 */

    return 0;               /* 检测到24L01 */
}

/**
 * @brief       NRF24L01写寄存器
 * @param       reg   : 寄存器地址
 * @param       value : 写入寄存器的值
 * @retval      状态寄存器值
 */
static uint8_t nrf24l01_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t status;
    
    NRF24L01_CSN(0);                    /* 使能SPI传输 */
    status = spi2_read_write_byte(reg); /* 发送寄存器号 */
    spi2_read_write_byte(value);        /* 写入寄存器的值 */
    NRF24L01_CSN(1);                    /* 禁止SPI传输 */
    
    return status;                      /* 返回状态值 */
}

/**
 * @brief       NRF24L01读寄存器
 * @param       reg   : 寄存器地址
 * @retval      读取到的寄存器值;
 */
static uint8_t nrf24l01_read_reg(uint8_t reg)
{
    uint8_t reg_val;
    
    NRF24L01_CSN(0);            /* 使能SPI传输 */
    spi2_read_write_byte(reg);  /* 发送寄存器号 */
    reg_val = spi2_read_write_byte(0XFF);   /* 读取寄存器内容 */
    NRF24L01_CSN(1);            /* 禁止SPI传输 */
    
    return reg_val;             /* 返回状态值 */
}

/**
 * @brief       在指定位置读出指定长度的数据
 * @param       reg   : 寄存器地址
 * @param       pbuf  : 数据指针
 * @param       len   : 数据长度
 * @retval      状态寄存器值
 */
static uint8_t nrf24l01_read_buf(uint8_t reg, uint8_t *pbuf, uint8_t len)
{
    uint8_t status, i;
    
    NRF24L01_CSN(0);                            /* 使能SPI传输 */
    status = spi2_read_write_byte(reg);         /* 发送寄存器值(位置),并读取状态值 */

    for (i = 0; i < len; i++)
    {
        pbuf[i] = spi2_read_write_byte(0XFF);   /* 读出数据 */
    }
    
    NRF24L01_CSN(1);    /* 关闭SPI传输 */
    
    return status;      /* 返回读到的状态值 */
}

/**
 * @brief       在指定位置写指定长度的数据
 * @param       reg   : 寄存器地址
 * @param       pbuf  : 数据指针
 * @param       len   : 数据长度
 * @retval      状态寄存器值
 */
static uint8_t nrf24l01_write_buf(uint8_t reg, uint8_t *pbuf, uint8_t len)
{
    uint8_t status, i;
    
    NRF24L01_CSN(0);    /* 使能SPI传输 */
    status = spi2_read_write_byte(reg);/* 发送寄存器值(位置),并读取状态值 */

    for (i = 0; i < len; i++)
    {
        spi2_read_write_byte(*pbuf++); /* 写入数据 */
    }
    
    NRF24L01_CSN(1);    /* 关闭SPI传输 */
    
    return status;      /* 返回读到的状态值 */
}

/**
 * @brief       启动NRF24L01发送一次数据(数据长度 = TX_PLOAD_WIDTH)
 * @param       ptxbuf : 待发送数据首地址
 * @retval      发送完成状态
 *   @arg       0    : 发送成功
 *   @arg       1    : 达到最大发送次数,失败
 *   @arg       0XFF : 其他错误
 */
uint8_t nrf24l01_tx_packet(uint8_t *ptxbuf)
{
    uint8_t sta;
    uint8_t rval = 0XFF;
    
    NRF24L01_CE(0);
    nrf24l01_write_buf(WR_TX_PLOAD, ptxbuf, TX_PLOAD_WIDTH);    /* 写数据到TX BUF  TX_PLOAD_WIDTH个字节 */
    NRF24L01_CE(1);                     /* 启动发送 */

    while (NRF24L01_IRQ != 0);          /* 等待发送完成 */

    sta = nrf24l01_read_reg(STATUS);    /* 读取状态寄存器的值 */
    nrf24l01_write_reg(NRF_WRITE_REG + STATUS, sta);    /* 清除TX_DS或MAX_RT中断标志 */

    if (sta & MAX_TX)   /* 达到最大重发次数 */
    {
        nrf24l01_write_reg(FLUSH_TX, 0xff);             /* 清除TX FIFO寄存器 */
        rval = 1;
    }

    if (sta & TX_OK)    /* 发送完成 */
    {
        rval = 0;       /* 标记发送成功 */
    }

    return rval;        /* 返回结果 */
}

/**
 * @brief       启动NRF24L01接收一次数据(数据长度 = RX_PLOAD_WIDTH)
 * @param       prxbuf : 接收数据缓冲区首地址
 * @retval      接收完成状态
 *   @arg       0 : 接收成功
 *   @arg       1 : 失败
 */
uint8_t nrf24l01_rx_packet(uint8_t *prxbuf)
{
    uint8_t sta;
    uint8_t rval = 1;
    
    sta = nrf24l01_read_reg(STATUS);                            /* 读取状态寄存器的值 */
    nrf24l01_write_reg(NRF_WRITE_REG + STATUS, sta);            /* 清除TX_DS或MAX_RT中断标志 */

    if (sta & RX_OK)    /* 接收到数据 */
    {
        nrf24l01_read_buf(RD_RX_PLOAD, prxbuf, RX_PLOAD_WIDTH); /* 读取数据 */
        nrf24l01_write_reg(FLUSH_RX, 0xff);                     /* 清除RX FIFO寄存器 */
        rval = 0;       /* 标记接收完成 */
    }

    return rval;        /* 返回结果 */
}

/**
 * @brief       NRF24L01进入接收模式
 *   @note      设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
 *              当CE变高后,即进入RX模式,并可以接收数据了
 * @param       无
 * @retval      无
 */
void nrf24l01_rx_mode(void)
{
    NRF24L01_CE(0);
    nrf24l01_write_buf(NRF_WRITE_REG + RX_ADDR_P0, (uint8_t *)RX_ADDRESS, RX_ADR_WIDTH);    /* 写RX节点地址 */

    nrf24l01_write_reg(NRF_WRITE_REG + EN_AA, 0x01);        /* 使能通道0的自动应答 */
    nrf24l01_write_reg(NRF_WRITE_REG + EN_RXADDR, 0x01);    /* 使能通道0的接收地址 */
    nrf24l01_write_reg(NRF_WRITE_REG + RF_CH, 40);          /* 设置RF通信频率 */
    nrf24l01_write_reg(NRF_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH);   /* 选择通道0的有效数据宽度 */
    nrf24l01_write_reg(NRF_WRITE_REG + RF_SETUP, 0x0f);     /* 设置TX发射参数,0db增益,2Mbps,低噪声增益开启 */
    nrf24l01_write_reg(NRF_WRITE_REG + CONFIG, 0x0f);       /* 配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 */
    NRF24L01_CE(1); /* CE为高,进入接收模式 */
}

/**
 * @brief       NRF24L01进入发送模式
 *   @note      设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,选择RF频道,波特率和
 *              LNA HCURR,PWR_UP,CRC使能
 *              当CE变高后,即进入TX模式,并可以发送数据了, CE为高大于10us,则启动发送.
 * @param       无
 * @retval      无
 */
void nrf24l01_tx_mode(void)
{
    NRF24L01_CE(0);
    nrf24l01_write_buf(NRF_WRITE_REG + TX_ADDR, (uint8_t *)TX_ADDRESS, TX_ADR_WIDTH);       /* 写TX节点地址 */
    nrf24l01_write_buf(NRF_WRITE_REG + RX_ADDR_P0, (uint8_t *)RX_ADDRESS, RX_ADR_WIDTH);    /* 设置RX节点地址,主要为了使能ACK */

    nrf24l01_write_reg(NRF_WRITE_REG + EN_AA, 0x01);        /* 使能通道0的自动应答 */
    nrf24l01_write_reg(NRF_WRITE_REG + EN_RXADDR, 0x01);    /* 使能通道0的接收地址 */
    nrf24l01_write_reg(NRF_WRITE_REG + SETUP_RETR, 0x1a);   /* 设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次 */
    nrf24l01_write_reg(NRF_WRITE_REG + RF_CH, 40);          /* 设置RF通道为40 */
    nrf24l01_write_reg(NRF_WRITE_REG + RF_SETUP, 0x0f);     /* 设置TX发射参数,0db增益,2Mbps,低噪声增益开启 */
    nrf24l01_write_reg(NRF_WRITE_REG + CONFIG, 0x0e);       /* 配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断 */
    NRF24L01_CE(1); /* CE为高,10us后启动发送 */
}


/**
 * @brief  B模块配置1：切换为「接收A数据」的模式（A-B参数）
 * @note   基于库函数nrf24l01_rx_mode()修改，适配ADDR_AB、CH_AB
 */
void B_nrf24l01_switch_rx_from_A(void)
{
    NRF24L01_CE(0);  // 拉低CE，进入配置模式（停止当前模式）
    
    // 1. 写RX_ADDR_P0（A→B的地址 ADDR_AB）
    nrf24l01_write_buf(NRF_WRITE_REG + RX_ADDR_P0, ADDR_AB, ADDR_AB_LEN);
    
    // 2. 库函数默认配置（保持不变，仅修改频道和地址）
    nrf24l01_write_reg(NRF_WRITE_REG + EN_AA, 0x01);        // 使能通道0自动应答（向A回传ACK）
    nrf24l01_write_reg(NRF_WRITE_REG + EN_RXADDR, 0x01);    // 使能通道0接收地址
    nrf24l01_write_reg(NRF_WRITE_REG + RF_CH, CH_AB);       // 切换到A-B频道 30
    nrf24l01_write_reg(NRF_WRITE_REG + RX_PW_P0, DATA_WIDTH); // 数据长度32字节
    nrf24l01_write_reg(NRF_WRITE_REG + RF_SETUP, 0x0f);     // 0db增益，2Mbps，低噪声增益开启
    nrf24l01_write_reg(NRF_WRITE_REG + CONFIG, 0x0f);       // 接收模式，PWR_UP，EN_CRC
    
    NRF24L01_CE(1);  // 拉高CE，进入接收状态
//    HAL_Delay(10);   // 延时保证配置生效，避免模式冲突
}

/**
 * @brief  B模块配置2：切换为「发送数据给C」的模式（B-C参数）
 * @note   基于库函数nrf24l01_tx_mode()修改，适配ADDR_BC、CH_BC
 */
void B_nrf24l01_switch_tx_to_C(void)
{
    NRF24L01_CE(0);  // 拉低CE，进入配置模式（停止当前模式）
    
    // 1. 写TX地址（B→C的地址 ADDR_BC）
    nrf24l01_write_buf(NRF_WRITE_REG + TX_ADDR, ADDR_BC, ADDR_BC_LEN);
    // 2. 写RX_ADDR_P0（使能ACK，必须与C的TX地址一致，即ADDR_BC）
    nrf24l01_write_buf(NRF_WRITE_REG + RX_ADDR_P0, ADDR_BC, ADDR_BC_LEN);
    
    // 3. 库函数默认配置（保持不变，仅修改频道和地址）
    nrf24l01_write_reg(NRF_WRITE_REG + EN_AA, 0x01);        // 使能通道0自动应答
    nrf24l01_write_reg(NRF_WRITE_REG + EN_RXADDR, 0x01);    // 使能通道0接收地址
    nrf24l01_write_reg(NRF_WRITE_REG + SETUP_RETR, 0x1a);   // 自动重发：10次，间隔500us+86us
    nrf24l01_write_reg(NRF_WRITE_REG + RF_CH, CH_BC);       // 切换到B-C频道 40
    nrf24l01_write_reg(NRF_WRITE_REG + RF_SETUP, 0x0f);     // 0db增益，2Mbps，低噪声增益开启
    nrf24l01_write_reg(NRF_WRITE_REG + CONFIG, 0x0e);       // 发送模式，PWR_UP，EN_CRC
    
//    HAL_Delay(10);   // 延时保证配置生效，无需立即拉高CE（发送时由nrf24l01_tx_packet触发）
}
void NRF_check(void)
{
    while (nrf24l01_check()){
	 LED0_TOGGLE();
		delay_ms(1000);
	} /* 检查NRF24L01是否在线 */
    
	B_nrf24l01_switch_rx_from_A();
	LED0(1);
}






