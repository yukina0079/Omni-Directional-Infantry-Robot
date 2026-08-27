#ifndef __UART1_H
#define __UART1_H

#include "sys.h"
#include <stdio.h>
#define FRAME_SIZE  20   /* 数据帧长度固定为20字节 */
extern UART_HandleTypeDef uart1_handle;
extern DMA_HandleTypeDef  hdma_usart2_rx;
extern uint8_t uart1_rx_buf[20];

void uart1_init(uint32_t baudrate);
void uart1_poll(void);

#endif
