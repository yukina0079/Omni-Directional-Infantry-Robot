#ifndef __USART_H__
#define __USART_H__

#include "stdio.h"
#include "sys.h"

#define UART_EOK                     0
#define UART_ERROR                   1
#define UART_ETIMEOUT                2
#define UART_EINVAL                  3

#define UART1_RX_BUF_SIZE            128
#define UART1_TX_BUF_SIZE            64

void uart1_init(uint32_t bound);

#endif
