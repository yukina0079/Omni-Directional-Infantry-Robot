#ifndef __nrf24L01_H
#define __nrf24L01_H
#include "stdint.h"
#include "sys.h"

#define W25Q128_CS(x) do{ x ?\
													HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET):\
													HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);\
												}while(0)

#define NRF24L01_SPI_CS_ENABLE()    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)
#define NRF24L01_SPI_CS_DISABLE()   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)

#define NRF24L01_CE_LOW()   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET)
#define NRF24L01_CE_HIGH()  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET)

#define NRF24L01_IRQ_PIN_READ()  HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)

#define TX_ADR_WIDTH                                  5
#define RX_ADR_WIDTH                                  5
#define TX_PLOAD_WIDTH  13
#define RX_PLOAD_WIDTH  13

#define NRF_READ_REG                                  0x00
#define NRF_WRITE_REG                                 0x20
#define RD_RX_PLOAD                                   0x61
#define WR_TX_PLOAD                                   0xA0
#define FLUSH_TX                                      0xE1
#define FLUSH_RX                                      0xE2
#define REUSE_TX_PL                                   0xE3
#define NOP                                           0xFF

#define CONFIG                                        0x00

#define EN_AA                                         0x01
#define EN_RXADDR                                     0x02
#define SETUP_AW                                      0x03
#define SETUP_RETR                                    0x04
#define RF_CH                                         0x05
#define RF_SETUP                                      0x06
#define STATUS                                        0x07

#define MAX_TX  		                                  0x10
#define TX_OK   		                                  0x20
#define RX_OK   		                                  0x40

#define OBSERVE_TX                                    0x08
#define CD                                            0x09
#define RX_ADDR_P0                                    0x0A
#define RX_ADDR_P1                                    0x0B
#define RX_ADDR_P2                                    0x0C
#define RX_ADDR_P3                                    0x0D
#define RX_ADDR_P4                                    0x0E
#define RX_ADDR_P5                                    0x0F
#define TX_ADDR                                       0x10

#define RX_PW_P0                                      0x11
#define RX_PW_P1                                      0x12
#define RX_PW_P2                                      0x13
#define RX_PW_P3                                      0x14
#define RX_PW_P4                                      0x15
#define RX_PW_P5                                      0x16

#define NRF_FIFO_STATUS                               0x17

void NRF24L01_Init(void);
void NRF24L01_RX_Mode(void);
void NRF24L01_TX_Mode(void);
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uint8_ts);
uint8_t NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uint8_ts);
uint8_t NRF24L01_Read_Reg(uint8_t reg);
uint8_t NRF24L01_Write_Reg(uint8_t reg, uint8_t value);
uint8_t NRF24L01_Check(void);
uint8_t NRF24L01_TxPacket(uint8_t *txbuf);
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf);
void NRF_LowPower_Mode(void);
void B_nrf24l01_switch_rx_from_A(void);
void B_nrf24l01_switch_tx_to_C(void);
void C_nrf24l01_rx_from_B(void);
void NRF24L01_find(void);

#endif
