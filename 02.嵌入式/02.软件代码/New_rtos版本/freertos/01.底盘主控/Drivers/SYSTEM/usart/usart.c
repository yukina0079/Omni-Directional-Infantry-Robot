#include "sys.h"
#include "usart.h"
#include "data.h"          /* 包含 frame_sync 和 uart_recv_byte */

/* ---------------- 已注释：原生 FreeRTOS 不需要 os.h ---------------- */
/*
#if SYS_SUPPORT_OS
#include "os.h"
#endif
*/

/******************************************************************************************/
/* 加入以下代码, 支持printf函数, 而不需要选择use MicroLIB (保持原有 USART3 输出) */

#if 1
#if (__ARMCC_VERSION >= 6010050)                    /* 使用AC6编译器时 */
__asm(".global __use_no_semihosting\n\t");          /* 声明不使用半主机模式 */
__asm(".global __ARM_use_no_argv \n\t");            /* AC6下需要声明main函数为无参数格式 */

#else
/* 使用AC5编译器时, 要在这里定义__FILE 和 不使用半主机模式 */
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
};

#endif

int _ttywrch(int ch) { return ch; }
void _sys_exit(int x) { x = x; }
char *_sys_command_string(char *cmd, int len) { return NULL; }
FILE __stdout;

/* 重定义fputc函数, printf输出到USART3 */
int fputc(int ch, FILE *f)
{
    while ((USART3->SR & 0X40) == 0);               /* 等待发送完成 */
    USART3->DR = (uint8_t)ch;
    return ch;
}
#endif
/***********************************************END*******************************************/

/* ---------- 原有 DMA 发送句柄 ---------- */
DMA_HandleTypeDef g_uart3_dma_tx_handle;

/* ---------- 新增：DMA 接收相关全局变量 ---------- */
DMA_HandleTypeDef g_hdma_usart_rx;
uint8_t g_uart_rx_buf[FRAME_SIZE];
static uint16_t last_dma_pos = 0;

/* ---------- 原有测试数据 ---------- */
uint8_t g_uart3_tx_buf[] = "USART3 DMA Single Send: Hello F407!\r\n";
const uint16_t g_uart3_tx_buf_len = sizeof(g_uart3_tx_buf) - 1;

/* ---------- 保留原有变量定义 (兼容旧代码) ---------- */
#if USART_EN_RX
uint8_t g_usart_rx_buf[USART_REC_LEN];
uint16_t g_usart_rx_sta = 0;
uint8_t g_rx_buffer[RXBUFFERSIZE];
UART_HandleTypeDef g_uart1_handle;
#endif

/**
 * @brief  串口初始化函数 (已修改：增加 DMA 循环接收)
 * @param  baudrate: 波特率
 */
void usart_init(uint32_t baudrate)
{
    /* 1. 配置UART基础参数 (USART3) */
    g_uart1_handle.Instance = USART_UX;
    g_uart1_handle.Init.BaudRate = baudrate;
    g_uart1_handle.Init.WordLength = UART_WORDLENGTH_8B;
    g_uart1_handle.Init.StopBits = UART_STOPBITS_1;
    g_uart1_handle.Init.Parity = UART_PARITY_NONE;
    g_uart1_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    g_uart1_handle.Init.Mode = UART_MODE_TX_RX;
    HAL_UART_Init(&g_uart1_handle);

    /* 2. 原有：DMA发送配置 (单次模式) */
    g_uart3_dma_tx_handle.Instance = DMA1_Stream3;
    g_uart3_dma_tx_handle.Init.Channel = DMA_CHANNEL_4;
    g_uart3_dma_tx_handle.Init.Direction = DMA_MEMORY_TO_PERIPH;
    g_uart3_dma_tx_handle.Init.PeriphInc = DMA_PINC_DISABLE;
    g_uart3_dma_tx_handle.Init.MemInc = DMA_MINC_ENABLE;
    g_uart3_dma_tx_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    g_uart3_dma_tx_handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    g_uart3_dma_tx_handle.Init.Mode = DMA_NORMAL;
    g_uart3_dma_tx_handle.Init.Priority = DMA_PRIORITY_MEDIUM;
    g_uart3_dma_tx_handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&g_uart3_dma_tx_handle);
    __HAL_LINKDMA(&g_uart1_handle, hdmatx, g_uart3_dma_tx_handle);

    /* 3. 新增：DMA接收配置 (循环模式 CIRCULAR) */
    /* USART3_RX 对应 DMA1_Stream1, Channel4 */
    g_hdma_usart_rx.Instance = DMA1_Stream1;
    g_hdma_usart_rx.Init.Channel = DMA_CHANNEL_4;
    g_hdma_usart_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    g_hdma_usart_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    g_hdma_usart_rx.Init.MemInc = DMA_MINC_ENABLE;
    g_hdma_usart_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    g_hdma_usart_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    g_hdma_usart_rx.Init.Mode = DMA_CIRCULAR;      /* 循环模式 */
    g_hdma_usart_rx.Init.Priority = DMA_PRIORITY_HIGH;
    g_hdma_usart_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&g_hdma_usart_rx);
    __HAL_LINKDMA(&g_uart1_handle, hdmarx, g_hdma_usart_rx);

    /* 4. 启动 DMA 循环接收 */
    HAL_UART_Receive_DMA(&g_uart1_handle, g_uart_rx_buf, FRAME_SIZE);
    
    /* 5. 初始化上次位置 */
    last_dma_pos = FRAME_SIZE - __HAL_DMA_GET_COUNTER(&g_hdma_usart_rx);

    /* 6. 注释掉原有接收中断，改为纯轮询 */
    // HAL_UART_Receive_IT(&g_uart1_handle, (uint8_t *)g_rx_buffer, RXBUFFERSIZE);
}

/**
 * @brief  UART底层驱动初始化 (已修改：增加 DMA 接收引脚和时钟)
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio_init_struct;

    if (huart->Instance == USART_UX)
    {
        /* 1. 使能时钟 */
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_DMA1_CLK_ENABLE();        /* DMA1 时钟 */

        /* 2. 配置 TX (PC11) */
        gpio_init_struct.Pin = USART_TX_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_PULLUP;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct.Alternate = USART_TX_GPIO_AF;
        HAL_GPIO_Init(USART_TX_GPIO_PORT, &gpio_init_struct);

        /* 3. 配置 RX (PC10) */
        gpio_init_struct.Pin = USART_RX_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;      /* 复用推挽输入 */
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Alternate = USART_RX_GPIO_AF;
        HAL_GPIO_Init(USART_RX_GPIO_PORT, &gpio_init_struct);

        /* 4. 原有：DMA发送初始化已在上面统一处理，这里不需要重复 */
        
        /* 5. 不使能 UART 中断，纯轮询 */
    }
}

/**
 * @brief  新增：轮询处理接收到的数据 (需在主循环 while(1) 中调用)
 */
void usart_poll(void)
{
    uint16_t current_pos = FRAME_SIZE - __HAL_DMA_GET_COUNTER(&g_hdma_usart_rx);
    uint16_t new_bytes;

    /* 计算新收到的字节数 */
    if (current_pos >= last_dma_pos) {
        new_bytes = current_pos - last_dma_pos;
    } else {
        new_bytes = (FRAME_SIZE - last_dma_pos) + current_pos;
    }

    /* 逐个处理新字节 */
    for (uint16_t i = 0; i < new_bytes; i++) {
        uint16_t index = (last_dma_pos + i) % FRAME_SIZE;
        uint8_t byte = g_uart_rx_buf[index];

        /* 调用逐字节同步函数 */
        if (frame_sync(byte, Uart_recv_byte)) {
            // 帧处理完成，在此添加你的代码
        }
    }

    /* 更新位置 */
    last_dma_pos = current_pos;

    /* 清除溢出标志 */
    if (__HAL_UART_GET_FLAG(&g_uart1_handle, UART_FLAG_ORE) != RESET) {
        __HAL_UART_CLEAR_OREFLAG(&g_uart1_handle);
    }
}

/**
 * @brief  原有：终极版 USART3 DMA 连续发送函数 (保持不变)
 */
void usart3_dma_send(uint8_t *pbuf, uint16_t len)
{
    while(DMA1_Stream3->CR & DMA_SxCR_EN);
    DMA1->LIFCR = (DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3);
    USART3->SR = ~(USART_SR_TC);
    g_uart1_handle.gState = HAL_UART_STATE_READY;
    g_uart1_handle.hdmatx->State = HAL_DMA_STATE_READY;
    DMA1_Stream3->NDTR = len;
    DMA1_Stream3->M0AR = (uint32_t)pbuf;
    DMA1_Stream3->PAR  = (uint32_t)&(USART3->DR);
    USART3->CR3 |= USART_CR3_DMAT;
    DMA1_Stream3->CR |= DMA_SxCR_EN;
}

/**
 * @brief  原有：中止DMA发送 (保持不变)
 */
void usart3_dma_stop_send(void)
{
    HAL_UART_AbortTransmit(&g_uart1_handle);
}

/* ---------------- 已注释：原有接收中断相关函数 (因为改为纯轮询) ---------------- */
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) { ... }
void USART_UX_IRQHandler(void) { ... }
*/
