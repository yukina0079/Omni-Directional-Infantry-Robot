#ifndef _USART_H
#define _USART_H

#include "stdio.h"
#include "sys.h"

/*******************************************************************************************************/
/* 引脚 和 串口 定义 (保持原有 USART3 配置) */
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

/*******************************************************************************************************/
/* ---------- 新增：DMA循环接收相关配置 ---------- */
#define FRAME_SIZE              20      /* 帧长度固定 */
#define USART_EN_RX             1       /* 使能接收 */

/* ---------- 原有发送相关定义 ---------- */
#define USART_REC_LEN           200     /* 保留原有定义 (兼容旧代码) */
#define RXBUFFERSIZE            1       /* 保留原有定义 (兼容旧代码) */

/* ---------- 句柄声明 ---------- */
extern UART_HandleTypeDef g_uart1_handle;       /* UART句柄 (指向USART3) */
extern DMA_HandleTypeDef  g_uart3_dma_tx_handle;/* DMA发送句柄 (原有) */
extern DMA_HandleTypeDef  g_hdma_usart_rx;      /* DMA接收句柄 (新增) */

/* ---------- 缓冲区声明 ---------- */
extern uint8_t g_uart_rx_buf[FRAME_SIZE];       /* DMA循环接收缓冲区 (新增) */
extern uint8_t g_usart_rx_buf[USART_REC_LEN];   /* 保留原有缓冲区 (兼容旧代码) */
extern uint16_t g_usart_rx_sta;                  /* 保留原有状态 (兼容旧代码) */
extern uint8_t g_rx_buffer[RXBUFFERSIZE];        /* 保留原有缓存 (兼容旧代码) */
extern uint8_t g_uart3_tx_buf[];
extern const uint16_t g_uart3_tx_buf_len;

/* ---------- 函数声明 ---------- */
void usart_init(uint32_t baudrate);             /* 串口初始化 (已修改：增加DMA接收) */
void usart_poll(void);                           /* 轮询处理接收数据 (新增) */
void usart3_dma_send(uint8_t *pbuf, uint16_t len); /* DMA发送 (原有) */
void usart3_dma_stop_send(void);                 /* 停止DMA发送 (原有) */

#endif
