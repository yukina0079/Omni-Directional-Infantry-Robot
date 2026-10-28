#ifndef __24L01_H
#define __24L01_H

#include "sys.h"

#define NRF24L01_CE_GPIO_PORT              GPIOD
#define NRF24L01_CE_GPIO_PIN               GPIO_PIN_15
#define NRF24L01_CE_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)

#define NRF24L01_CSN_GPIO_PORT             GPIOB
#define NRF24L01_CSN_GPIO_PIN              GPIO_PIN_12
#define NRF24L01_CSN_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)

#define NRF24L01_IRQ_GPIO_PORT             GPIOD
#define NRF24L01_IRQ_GPIO_PIN              GPIO_PIN_14
#define NRF24L01_IRQ_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)

#define NRF24L01_CE(x)    do{ x ? \
                              HAL_GPIO_WritePin(NRF24L01_CE_GPIO_PORT, NRF24L01_CE_GPIO_PIN, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(NRF24L01_CE_GPIO_PORT, NRF24L01_CE_GPIO_PIN, GPIO_PIN_RESET); \
                          }while(0)

#define NRF24L01_CSN(x)   do{ x ? \
                              HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_PORT, NRF24L01_CSN_GPIO_PIN, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_PORT, NRF24L01_CSN_GPIO_PIN, GPIO_PIN_RESET); \
                          }while(0)

#define NRF24L01_IRQ      HAL_GPIO_ReadPin(NRF24L01_IRQ_GPIO_PORT, NRF24L01_IRQ_GPIO_PIN)

#define TX_ADR_WIDTH    5
#define RX_ADR_WIDTH    5
#define TX_PLOAD_WIDTH  13
#define RX_PLOAD_WIDTH  13

#define NRF_READ_REG    0x00
#define NRF_WRITE_REG   0x20
#define RD_RX_PLOAD     0x61
#define WR_TX_PLOAD     0xA0
#define FLUSH_TX        0xE1
#define FLUSH_RX        0xE2
#define REUSE_TX_PL     0xE3
#define NOP             0xFF

#define CONFIG          0x00

#define EN_AA           0x01
#define EN_RXADDR       0x02
#define SETUP_AW        0x03
#define SETUP_RETR      0x04
#define RF_CH           0x05
#define RF_SETUP        0x06
#define STATUS          0x07

#define MAX_TX          0x10
#define TX_OK           0x20
#define RX_OK           0x40

#define OBSERVE_TX      0x08
#define CD              0x09
#define RX_ADDR_P0      0x0A
#define RX_ADDR_P1      0x0B
#define RX_ADDR_P2      0x0C
#define RX_ADDR_P3      0x0D
#define RX_ADDR_P4      0x0E
#define RX_ADDR_P5      0x0F
#define TX_ADDR         0x10
#define RX_PW_P0        0x11
#define RX_PW_P1        0x12
#define RX_PW_P2        0x13
#define RX_PW_P3        0x14
#define RX_PW_P4        0x15
#define RX_PW_P5        0x16
#define NRF_FIFO_STATUS 0x17

static uint8_t nrf24l01_write_buf(uint8_t reg, uint8_t *pbuf, uint8_t uint8_ts);
static uint8_t nrf24l01_read_buf(uint8_t reg, uint8_t *pbuf, uint8_t uint8_ts);

void nrf24l01_spi_init(void);
void nrf24l01_init(void);
void nrf24l01_rx_mode(void);
void nrf24l01_tx_mode(void);
uint8_t nrf24l01_check(void);
uint8_t nrf24l01_tx_packet(uint8_t *ptxbuf);
uint8_t nrf24l01_rx_packet(uint8_t *prxbuf);
void B_nrf24l01_switch_rx_from_A(void);
void B_nrf24l01_switch_tx_to_C(void);
void NRF_check(void);

#endif
