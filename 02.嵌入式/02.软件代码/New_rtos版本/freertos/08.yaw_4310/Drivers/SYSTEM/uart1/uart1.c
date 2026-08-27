#include "uart1.h"
#include "string.h"
#include "data.h"
#include <stdio.h>

/* 句柄和全局变量定义 */
UART_HandleTypeDef uart1_handle;
DMA_HandleTypeDef  hdma_usart2_rx;
uint8_t uart1_rx_buf[FRAME_SIZE];

/* 上次处理时的DMA写入位置 */
static uint16_t last_dma_pos = 0;

/**
 * @brief  重定义fputc (printf输出到USART2)
 */
int fputc(int ch, FILE *f)
{
    while((USART2->SR & 0X40) == 0);
    USART2->DR = (uint8_t)ch;
    return ch;
}

/**
 * @brief  USART2初始化函数
 * @param  baudrate: 波特率 (如 115200)
 */
void uart1_init(uint32_t baudrate)
{
    /* 1. 配置UART基础参数 */
    uart1_handle.Instance = USART2;
    uart1_handle.Init.BaudRate = baudrate;
    uart1_handle.Init.WordLength = UART_WORDLENGTH_8B;
    uart1_handle.Init.StopBits = UART_STOPBITS_1;
    uart1_handle.Init.Parity = UART_PARITY_NONE;
    uart1_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart1_handle.Init.Mode = UART_MODE_TX_RX;

    /* 2. 初始化UART */
    HAL_UART_Init(&uart1_handle);

    /* 3. 启动DMA循环接收，缓冲区大小为帧长度 */
    HAL_UART_Receive_DMA(&uart1_handle, uart1_rx_buf, FRAME_SIZE);

    /* 4. 初始化上次位置，避免首次误判 */
    last_dma_pos = FRAME_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
}

/**
 * @brief  UART底层驱动初始化
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio_init_struct;

    if(huart->Instance == USART2)
    {
        /* 1. 使能时钟 */
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_DMA1_CLK_ENABLE();

        /* 2. 配置 TX (PA2) */
        gpio_init_struct.Pin = GPIO_PIN_2;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_PULLUP;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &gpio_init_struct);

        /* 3. 配置 RX (PA3) 为输入 */
        gpio_init_struct.Pin = GPIO_PIN_3;
        gpio_init_struct.Mode = GPIO_MODE_INPUT;
        gpio_init_struct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &gpio_init_struct);

        /* 4. 配置 DMA (USART2_RX -> DMA1_Channel6) 为循环模式 */
        hdma_usart2_rx.Instance = DMA1_Channel6;
        hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;      /* 循环模式 */
        hdma_usart2_rx.Init.Priority = DMA_PRIORITY_HIGH;
        HAL_DMA_Init(&hdma_usart2_rx);

        /* 5. 关联 DMA 句柄 */
        __HAL_LINKDMA(huart, hdmarx, hdma_usart2_rx);

        /* 不使能任何中断，纯轮询 */
    }
}

void uart1_poll(void)
{
    uint16_t current_pos = FRAME_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
    uint16_t new_bytes;

    /* 计算新收到的字节数（处理环形回绕） */
    if (current_pos >= last_dma_pos) {
        new_bytes = current_pos - last_dma_pos;
    } else {
        new_bytes = (FRAME_SIZE - last_dma_pos) + current_pos;
    }

    /* 逐个处理新字节 */
    for (uint16_t i = 0; i < new_bytes; i++) {
        uint16_t index = (last_dma_pos + i) % FRAME_SIZE;
        uint8_t byte = uart1_rx_buf[index];

        /* 调用逐字节同步函数，若返回1表示收到一帧 */
        if (frame_sync(byte, uart_recv)) {
            /* 打印新帧 */
//            for (uint16_t j = 0; j < FRAME_SIZE; j++) {
//                printf("%02X ", uart_recv[j]);
//            }
//            printf("\r\n");
        }
    }

    /* 更新上次位置 */
    last_dma_pos = current_pos;

    /* 清除溢出标志（可选） */
    if (__HAL_UART_GET_FLAG(&uart1_handle, UART_FLAG_ORE) != RESET) {
        __HAL_UART_CLEAR_OREFLAG(&uart1_handle);
    }
}
