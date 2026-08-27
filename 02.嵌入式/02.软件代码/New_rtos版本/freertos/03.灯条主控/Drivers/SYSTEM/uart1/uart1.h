#ifndef __USART_H__
#define __USART_H__

#include "stdio.h"
#include "sys.h"

/* 错误代码 */
#define UART_EOK                     0      /* 没有错误 */
#define UART_ERROR                   1      /* 通用错误 */
#define UART_ETIMEOUT                2      /* 超时错误 */
#define UART_EINVAL                  3      /* 参数错误 */

/* UART收发缓冲大小 */
#define UART1_RX_BUF_SIZE            128
#define UART1_TX_BUF_SIZE            64


void uart1_init(uint32_t bound);            /* 串口初始化函数 */

#endif


