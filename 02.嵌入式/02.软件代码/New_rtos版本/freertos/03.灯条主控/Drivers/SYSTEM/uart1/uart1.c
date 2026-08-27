#include "sys.h"
#include "uart1.h"
#include "string.h"

UART_HandleTypeDef uart1_handle;                                            /* UART1句柄 */

uint8_t uart1_rx_buf[UART1_RX_BUF_SIZE];                                    /* UART1接收缓冲区 */
uint16_t uart1_rx_len = 0;                                                  /* UART1接收字符长度 */

/**
 * @brief       重定义fputc函数
 * @note        printf函数最终会通过调用fputc输出字符串到串口
 */
int fputc(int ch, FILE *f)
{
    while ((USART1->SR & 0X40) == 0);                                       /* 等待上一个字符发送完成 */

    USART1->DR = (uint8_t)ch;                                               /* 将要发送的字符 ch 写入到DR寄存器 */
    return ch;
}

/**
 * @brief       串口1初始化函数
 * @param       baudrate: 波特率, 根据自己需要设置波特率值
 * @retval      无
 */
void uart1_init(uint32_t baudrate)
{
    /*UART1 初始化设置*/
    uart1_handle.Instance = USART1;                                         /* USART1 */
    uart1_handle.Init.BaudRate = baudrate;                                  /* 波特率 */
    uart1_handle.Init.WordLength = UART_WORDLENGTH_8B;                      /* 字长为8位数据格式 */
    uart1_handle.Init.StopBits = UART_STOPBITS_1;                           /* 一个停止位 */
    uart1_handle.Init.Parity = UART_PARITY_NONE;                            /* 无奇偶校验位 */
    uart1_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;                      /* 无硬件流控 */
    uart1_handle.Init.Mode = UART_MODE_TX_RX;                               /* 收发模式 */
    HAL_UART_Init(&uart1_handle);                                           /* HAL_UART_Init()会使能UART1 */
}

/**
 * @brief       UART底层初始化函数
 * @param       huart: UART句柄类型指针
 * @note        此函数会被HAL_UART_Init()调用
 *              完成时钟使能，引脚配置，中断配置
 * @retval      无
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio_init_struct;

    if (huart->Instance == USART1)                                          /* 如果是串口1，进行串口1 MSP初始化 */
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();                                       /* 使能串口TX脚时钟 */
        __HAL_RCC_USART1_CLK_ENABLE();                                      /* 使能串口时钟 */

        gpio_init_struct.Pin = GPIO_PIN_9;                                  /* 串口发送引脚号 */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                            /* 复用推挽输出 */
        gpio_init_struct.Pull = GPIO_PULLUP;                                /* 上拉 */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;                      /* IO速度设置为高速 */
        HAL_GPIO_Init(GPIOA, &gpio_init_struct);
                
        gpio_init_struct.Pin = GPIO_PIN_10;                                 /* 串口RX脚 模式设置 */
        gpio_init_struct.Mode = GPIO_MODE_AF_INPUT;    
        HAL_GPIO_Init(GPIOA, &gpio_init_struct);                            /* 串口RX脚 必须设置成输入模式 */
        
        HAL_NVIC_EnableIRQ(USART1_IRQn);                                    /* 使能USART1中断通道 */
        HAL_NVIC_SetPriority(USART1_IRQn, 3, 3);                            /* 组2，最低优先级:抢占优先级3，子优先级3 */

        __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);                          /* 使能UART1接收中断 */
        __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);                          /* 使能UART1总线空闲中断 */
    }
}

/**
 * @brief       UART1接收缓冲区清除
 * @param       无
 * @retval      无
 */
void uart1_rx_clear(void)
{
    memset(uart1_rx_buf, 0, sizeof(uart1_rx_buf));                          /* 清空接收缓冲区 */
    uart1_rx_len = 0;                                                       /* 接收计数器清零 */
}

/**
 * @brief       串口1中断服务函数
 * @note        在此使用接收中断及空闲中断，实现不定长数据收发
 * @param       无
 * @retval      无
 */
void USART1_IRQHandler(void)
{
    uint8_t receive_data = 0;   
    if(__HAL_UART_GET_FLAG(&uart1_handle, UART_FLAG_RXNE) != RESET){        /* 获取接收RXNE标志位是否被置位 */
        if(uart1_rx_len >= sizeof(uart1_rx_buf))                            /* 如果接收的字符数大于接收缓冲区大小， */
            uart1_rx_len = 0;                                               /* 则将接收计数器清零 */
        HAL_UART_Receive(&uart1_handle, &receive_data, 1, 1000);            /* 接收一个字符 */
        uart1_rx_buf[uart1_rx_len++] = receive_data;                        /* 将接收到的字符保存在接收缓冲区 */
    }

    if (__HAL_UART_GET_FLAG(&uart1_handle, UART_FLAG_IDLE) != RESET)        /* 获取接收空闲中断标志位是否被置位 */
    {
        printf("recv: %s\r\n", uart1_rx_buf);                               /* 将接收到的数据打印出来 */
        uart1_rx_clear();
        __HAL_UART_CLEAR_IDLEFLAG(&uart1_handle);                           /* 清除UART总线空闲中断 */
    }
}
