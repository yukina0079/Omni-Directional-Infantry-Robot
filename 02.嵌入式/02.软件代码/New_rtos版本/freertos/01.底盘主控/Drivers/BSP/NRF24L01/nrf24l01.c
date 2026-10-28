#include "spi.h"
#include "nrf24l01.h"
#include "delay.h"
#include "led.h"
#define ADDR_AB_LEN    5
#define ADDR_BC_LEN    5
#define DATA_WIDTH     13

uint8_t ADDR_AB[ADDR_AB_LEN] = {0x11, 0x22, 0x33, 0x44, 0x55};
#define CH_AB          30

uint8_t ADDR_BC[ADDR_BC_LEN] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE};
#define CH_BC          40

extern SPI_HandleTypeDef g_spi2_handler;
const uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0x34, 0x43, 0x10, 0x10, 0x01};
const uint8_t RX_ADDRESS[RX_ADR_WIDTH] = {0x34, 0x43, 0x10, 0x10, 0x01};

void nrf24l01_spi_init(void)
{
    __HAL_SPI_DISABLE(&g_spi2_handler);
    g_spi2_handler.Init.CLKPolarity = SPI_POLARITY_LOW;
    g_spi2_handler.Init.CLKPhase = SPI_PHASE_1EDGE;
    HAL_SPI_Init(&g_spi2_handler);
    __HAL_SPI_ENABLE(&g_spi2_handler);
}

void nrf24l01_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    NRF24L01_CE_GPIO_CLK_ENABLE();
    NRF24L01_CSN_GPIO_CLK_ENABLE();
    NRF24L01_IRQ_GPIO_CLK_ENABLE();

    gpio_init_struct.Pin = NRF24L01_CE_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(NRF24L01_CE_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = NRF24L01_CSN_GPIO_PIN;
    HAL_GPIO_Init(NRF24L01_CSN_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = NRF24L01_IRQ_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(NRF24L01_IRQ_GPIO_PORT, &gpio_init_struct);

    spi2_init();
    nrf24l01_spi_init();
    NRF24L01_CE(0);
    NRF24L01_CSN(1);
}

uint8_t nrf24l01_check(void)
{
    uint8_t buf[5] = {0XA5, 0XA5, 0XA5, 0XA5, 0XA5};
    uint8_t i;

    spi2_set_speed(SPI_SPEED_32);
    nrf24l01_write_buf(NRF_WRITE_REG + TX_ADDR, buf, 5);
    nrf24l01_read_buf(TX_ADDR, buf, 5);

    for (i = 0; i < 5; i++)
    {
        if (buf[i] != 0XA5) break;
    }

    if (i != 5) return 1;

    return 0;
}

static uint8_t nrf24l01_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t status;

    NRF24L01_CSN(0);
    status = spi2_read_write_byte(reg);
    spi2_read_write_byte(value);
    NRF24L01_CSN(1);

    return status;
}

static uint8_t nrf24l01_read_reg(uint8_t reg)
{
    uint8_t reg_val;

    NRF24L01_CSN(0);
    spi2_read_write_byte(reg);
    reg_val = spi2_read_write_byte(0XFF);
    NRF24L01_CSN(1);

    return reg_val;
}

static uint8_t nrf24l01_read_buf(uint8_t reg, uint8_t *pbuf, uint8_t len)
{
    uint8_t status, i;

    NRF24L01_CSN(0);
    status = spi2_read_write_byte(reg);

    for (i = 0; i < len; i++)
    {
        pbuf[i] = spi2_read_write_byte(0XFF);
    }

    NRF24L01_CSN(1);

    return status;
}

static uint8_t nrf24l01_write_buf(uint8_t reg, uint8_t *pbuf, uint8_t len)
{
    uint8_t status, i;

    NRF24L01_CSN(0);
    status = spi2_read_write_byte(reg);

    for (i = 0; i < len; i++)
    {
        spi2_read_write_byte(*pbuf++);
    }

    NRF24L01_CSN(1);

    return status;
}

uint8_t nrf24l01_tx_packet(uint8_t *ptxbuf)
{
    uint8_t sta;
    uint8_t rval = 0XFF;

    NRF24L01_CE(0);
    nrf24l01_write_buf(WR_TX_PLOAD, ptxbuf, TX_PLOAD_WIDTH);
    NRF24L01_CE(1);

    while (NRF24L01_IRQ != 0);

    sta = nrf24l01_read_reg(STATUS);
    nrf24l01_write_reg(NRF_WRITE_REG + STATUS, sta);

    if (sta & MAX_TX)
    {
        nrf24l01_write_reg(FLUSH_TX, 0xff);
        rval = 1;
    }

    if (sta & TX_OK)
    {
        rval = 0;
    }

    return rval;
}

uint8_t nrf24l01_rx_packet(uint8_t *prxbuf)
{
    uint8_t sta;
    uint8_t rval = 1;

    sta = nrf24l01_read_reg(STATUS);
    nrf24l01_write_reg(NRF_WRITE_REG + STATUS, sta);

    if (sta & RX_OK)
    {
        nrf24l01_read_buf(RD_RX_PLOAD, prxbuf, RX_PLOAD_WIDTH);
        nrf24l01_write_reg(FLUSH_RX, 0xff);
        rval = 0;
    }

    return rval;
}

void nrf24l01_rx_mode(void)
{
    NRF24L01_CE(0);
    nrf24l01_write_buf(NRF_WRITE_REG + RX_ADDR_P0, (uint8_t *)RX_ADDRESS, RX_ADR_WIDTH);

    nrf24l01_write_reg(NRF_WRITE_REG + EN_AA, 0x01);
    nrf24l01_write_reg(NRF_WRITE_REG + EN_RXADDR, 0x01);
    nrf24l01_write_reg(NRF_WRITE_REG + RF_CH, 40);
    nrf24l01_write_reg(NRF_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH);
    nrf24l01_write_reg(NRF_WRITE_REG + RF_SETUP, 0x0f);
    nrf24l01_write_reg(NRF_WRITE_REG + CONFIG, 0x0f);
    NRF24L01_CE(1);
}

void nrf24l01_tx_mode(void)
{
    NRF24L01_CE(0);
    nrf24l01_write_buf(NRF_WRITE_REG + TX_ADDR, (uint8_t *)TX_ADDRESS, TX_ADR_WIDTH);
    nrf24l01_write_buf(NRF_WRITE_REG + RX_ADDR_P0, (uint8_t *)RX_ADDRESS, RX_ADR_WIDTH);

    nrf24l01_write_reg(NRF_WRITE_REG + EN_AA, 0x01);
    nrf24l01_write_reg(NRF_WRITE_REG + EN_RXADDR, 0x01);
    nrf24l01_write_reg(NRF_WRITE_REG + SETUP_RETR, 0x1a);
    nrf24l01_write_reg(NRF_WRITE_REG + RF_CH, 40);
    nrf24l01_write_reg(NRF_WRITE_REG + RF_SETUP, 0x0f);
    nrf24l01_write_reg(NRF_WRITE_REG + CONFIG, 0x0e);
    NRF24L01_CE(1);
}

void B_nrf24l01_switch_rx_from_A(void)
{
    NRF24L01_CE(0);

    nrf24l01_write_buf(NRF_WRITE_REG + RX_ADDR_P0, ADDR_AB, ADDR_AB_LEN);

    nrf24l01_write_reg(NRF_WRITE_REG + EN_AA, 0x01);
    nrf24l01_write_reg(NRF_WRITE_REG + EN_RXADDR, 0x01);
    nrf24l01_write_reg(NRF_WRITE_REG + RF_CH, CH_AB);
    nrf24l01_write_reg(NRF_WRITE_REG + RX_PW_P0, DATA_WIDTH);
    nrf24l01_write_reg(NRF_WRITE_REG + RF_SETUP, 0x0f);
    nrf24l01_write_reg(NRF_WRITE_REG + CONFIG, 0x0f);

    NRF24L01_CE(1);

}

void B_nrf24l01_switch_tx_to_C(void)
{
    NRF24L01_CE(0);

    nrf24l01_write_buf(NRF_WRITE_REG + TX_ADDR, ADDR_BC, ADDR_BC_LEN);

    nrf24l01_write_buf(NRF_WRITE_REG + RX_ADDR_P0, ADDR_BC, ADDR_BC_LEN);

    nrf24l01_write_reg(NRF_WRITE_REG + EN_AA, 0x01);
    nrf24l01_write_reg(NRF_WRITE_REG + EN_RXADDR, 0x01);
    nrf24l01_write_reg(NRF_WRITE_REG + SETUP_RETR, 0x1a);
    nrf24l01_write_reg(NRF_WRITE_REG + RF_CH, CH_BC);
    nrf24l01_write_reg(NRF_WRITE_REG + RF_SETUP, 0x0f);
    nrf24l01_write_reg(NRF_WRITE_REG + CONFIG, 0x0e);

}
void NRF_check(void)
{
    while (nrf24l01_check()){
	 LED0_TOGGLE();
		delay_ms(1000);
	}

	B_nrf24l01_switch_rx_from_A();
	LED0(1);
}
