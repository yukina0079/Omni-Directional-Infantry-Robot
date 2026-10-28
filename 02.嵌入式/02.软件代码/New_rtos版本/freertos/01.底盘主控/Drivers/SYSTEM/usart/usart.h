#ifndef _USART_H
#define _USART_H

#include "stdio.h"
#include "sys.h"

#define USART_TX_GPIO_PORT              GPIOC
#define USART_TX_GPIO_PIN               GPIO_PIN_10
#define USART_TX_GPIO_AF                GPIO_AF7_USART3
#define USART_TX_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)

#define USART_RX_GPIO_PORT              GPIOC
#define USART_RX_GPIO_PIN               GPIO_PIN_11
#define USART_RX_GPIO_AF                GPIO_AF7_USART3
#define USART_RX_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)

#define USART_UX                        USART3
#define USART_UX_IRQn                   USART3_IRQn
#define USART_UX_IRQHandler             USART3_IRQHandler
#define USART_UX_CLK_ENABLE()           do{ __HAL_RCC_USART3_CLK_ENABLE(); }while(0)

#define FRAME_SIZE              20
#define USART_EN_RX             1

#define USART_REC_LEN           200
#define RXBUFFERSIZE            1

extern UART_HandleTypeDef g_uart1_handle;
extern DMA_HandleTypeDef  g_uart3_dma_tx_handle;
extern DMA_HandleTypeDef  g_hdma_usart_rx;

extern uint8_t g_uart_rx_buf[FRAME_SIZE];
extern uint8_t g_usart_rx_buf[USART_REC_LEN];
extern uint16_t g_usart_rx_sta;
extern uint8_t g_rx_buffer[RXBUFFERSIZE];
extern uint8_t g_uart3_tx_buf[];
extern const uint16_t g_uart3_tx_buf_len;

void usart_init(uint32_t baudrate);
void usart_poll(void);
void usart3_dma_send(uint8_t *pbuf, uint16_t len);
void usart3_dma_stop_send(void);

#endif
