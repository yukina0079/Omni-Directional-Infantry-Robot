#ifndef _USART_H
#define _USART_H

#include "stdio.h"
#include "sys.h"

/*******************************************************************************************************/
/* ���� �� ���� ���� (����ԭ�� USART3 ����) */
#define USART_TX_GPIO_PORT              GPIOC
#define USART_TX_GPIO_PIN               GPIO_PIN_11
#define USART_TX_GPIO_AF                GPIO_AF7_USART3
#define USART_TX_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)  

#define USART_RX_GPIO_PORT              GPIOC
#define USART_RX_GPIO_PIN               GPIO_PIN_10
#define USART_RX_GPIO_AF                GPIO_AF7_USART3
#define USART_RX_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)  

#define USART_UX                        USART3
#define USART_UX_IRQn                   USART3_IRQn
#define USART_UX_IRQHandler             USART3_IRQHandler
#define USART_UX_CLK_ENABLE()           do{ __HAL_RCC_USART3_CLK_ENABLE(); }while(0) 

/*******************************************************************************************************/
/* ---------- ������DMAѭ������������� ---------- */
#define FRAME_SIZE              20      /* ֡���ȹ̶� */
#define USART_EN_RX             1       /* ʹ�ܽ��� */

/* ---------- ԭ�з�����ض��� ---------- */
#define USART_REC_LEN           200     /* ����ԭ�ж��� (���ݾɴ���) */
#define RXBUFFERSIZE            1       /* ����ԭ�ж��� (���ݾɴ���) */

/* ---------- ������� ---------- */
extern UART_HandleTypeDef g_uart1_handle;       /* UART��� (ָ��USART3) */
extern DMA_HandleTypeDef  g_uart3_dma_tx_handle;/* DMA���;�� (ԭ��) */
extern DMA_HandleTypeDef  g_hdma_usart_rx;      /* DMA���վ�� (����) */

/* ---------- ���������� ---------- */
extern uint8_t g_uart_rx_buf[FRAME_SIZE];       /* DMAѭ�����ջ����� (����) */
extern uint8_t g_usart_rx_buf[USART_REC_LEN];   /* ����ԭ�л����� (���ݾɴ���) */
extern uint16_t g_usart_rx_sta;                  /* ����ԭ��״̬ (���ݾɴ���) */
extern uint8_t g_rx_buffer[RXBUFFERSIZE];        /* ����ԭ�л��� (���ݾɴ���) */
extern uint8_t g_uart3_tx_buf[];
extern const uint16_t g_uart3_tx_buf_len;

/* ---------- �������� ---------- */
void usart_init(uint32_t baudrate);
void usart2_log_init(uint32_t baudrate);
void usart_poll(void);
void usart3_dma_send(uint8_t *pbuf, uint16_t len); /* DMA���� (ԭ��) */
void usart3_dma_stop_send(void);                 /* ֹͣDMA���� (ԭ��) */

#endif
