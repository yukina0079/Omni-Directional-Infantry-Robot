#include "sys.h"
#include "usart.h"
#include "data.h"          /* ���� frame_sync �� uart_recv_byte */

/* ---------------- ��ע�ͣ�ԭ�� FreeRTOS ����Ҫ os.h ---------------- */
/*
#if SYS_SUPPORT_OS
#include "os.h"
#endif
*/

/******************************************************************************************/
/* �������´���, ֧��printf����, ������Ҫѡ��use MicroLIB (����ԭ�� USART3 ���) */

#if 1
#if (__ARMCC_VERSION >= 6010050)                    /* ʹ��AC6������ʱ */
__asm(".global __use_no_semihosting\n\t");          /* ������ʹ�ð�����ģʽ */
__asm(".global __ARM_use_no_argv \n\t");            /* AC6����Ҫ����main����Ϊ�޲�����ʽ */

#else
/* ʹ��AC5������ʱ, Ҫ�����ﶨ��__FILE �� ��ʹ�ð�����ģʽ */
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

/* printf -> USART2 (PA2/PA3). USART3 is reserved for FOC frames. */
int fputc(int ch, FILE *f)
{
    (void)f;
    while ((USART2->SR & 0X40) == 0) { }
    USART2->DR = (uint8_t)ch;
    return ch;
}
#endif
/***********************************************END*******************************************/

/* ---------- ԭ�� DMA ���;�� ---------- */
DMA_HandleTypeDef g_uart3_dma_tx_handle;

/* ---------- ������DMA �������ȫ�ֱ��� ---------- */
DMA_HandleTypeDef g_hdma_usart_rx;
uint8_t g_uart_rx_buf[FRAME_SIZE];
static uint16_t last_dma_pos = 0;

/* ---------- ԭ�в������� ---------- */
uint8_t g_uart3_tx_buf[] = "USART3 DMA Single Send: Hello F407!\r\n";
const uint16_t g_uart3_tx_buf_len = sizeof(g_uart3_tx_buf) - 1;

/* ---------- ����ԭ�б������� (���ݾɴ���) ---------- */
#if USART_EN_RX
uint8_t g_usart_rx_buf[USART_REC_LEN];
uint16_t g_usart_rx_sta = 0;
uint8_t g_rx_buffer[RXBUFFERSIZE];
UART_HandleTypeDef g_uart1_handle;
#endif
UART_HandleTypeDef g_uart2_log;

void usart2_log_init(uint32_t baudrate)
{
    g_uart2_log.Instance = USART2;
    g_uart2_log.Init.BaudRate = baudrate;
    g_uart2_log.Init.WordLength = UART_WORDLENGTH_8B;
    g_uart2_log.Init.StopBits = UART_STOPBITS_1;
    g_uart2_log.Init.Parity = UART_PARITY_NONE;
    g_uart2_log.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    g_uart2_log.Init.Mode = UART_MODE_TX_RX;
    HAL_UART_Init(&g_uart2_log);
}

/**
 * @brief  ���ڳ�ʼ������ (���޸ģ����� DMA ѭ������)
 * @param  baudrate: ������
 */
void usart_init(uint32_t baudrate)
{
    usart2_log_init(baudrate);

    /* 1. USART3: FOC binary frames */
    g_uart1_handle.Instance = USART_UX;
    g_uart1_handle.Init.BaudRate = baudrate;
    g_uart1_handle.Init.WordLength = UART_WORDLENGTH_8B;
    g_uart1_handle.Init.StopBits = UART_STOPBITS_1;
    g_uart1_handle.Init.Parity = UART_PARITY_NONE;
    g_uart1_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    g_uart1_handle.Init.Mode = UART_MODE_TX_RX;
    HAL_UART_Init(&g_uart1_handle);

    /* 2. ԭ�У�DMA�������� (����ģʽ) */
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

    /* 3. ������DMA�������� (ѭ��ģʽ CIRCULAR) */
    /* USART3_RX ��Ӧ DMA1_Stream1, Channel4 */
    g_hdma_usart_rx.Instance = DMA1_Stream1;
    g_hdma_usart_rx.Init.Channel = DMA_CHANNEL_4;
    g_hdma_usart_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    g_hdma_usart_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    g_hdma_usart_rx.Init.MemInc = DMA_MINC_ENABLE;
    g_hdma_usart_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    g_hdma_usart_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    g_hdma_usart_rx.Init.Mode = DMA_CIRCULAR;      /* ѭ��ģʽ */
    g_hdma_usart_rx.Init.Priority = DMA_PRIORITY_HIGH;
    g_hdma_usart_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&g_hdma_usart_rx);
    __HAL_LINKDMA(&g_uart1_handle, hdmarx, g_hdma_usart_rx);

    /* 4. ���� DMA ѭ������ */
    HAL_UART_Receive_DMA(&g_uart1_handle, g_uart_rx_buf, FRAME_SIZE);
    
    /* 5. ��ʼ���ϴ�λ�� */
    last_dma_pos = FRAME_SIZE - __HAL_DMA_GET_COUNTER(&g_hdma_usart_rx);

    /* 6. ע�͵�ԭ�н����жϣ���Ϊ����ѯ */
    // HAL_UART_Receive_IT(&g_uart1_handle, (uint8_t *)g_rx_buffer, RXBUFFERSIZE);
}

/**
 * @brief  UART�ײ�������ʼ�� (���޸ģ����� DMA �������ź�ʱ��)
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio_init_struct;

    if (huart->Instance == USART2)
    {
        GPIO_InitTypeDef gpio2;

        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_USART2_CLK_ENABLE();
        gpio2.Pin = GPIO_PIN_2 | GPIO_PIN_3;
        gpio2.Mode = GPIO_MODE_AF_PP;
        gpio2.Pull = GPIO_PULLUP;
        gpio2.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio2.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &gpio2);
        return;
    }

    if (huart->Instance == USART_UX)
    {
        /* 1. ʹ��ʱ�� */
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_DMA1_CLK_ENABLE();        /* DMA1 ʱ�� */

        /* 2. ���� TX (PC11) */
        gpio_init_struct.Pin = USART_TX_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_PULLUP;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct.Alternate = USART_TX_GPIO_AF;
        HAL_GPIO_Init(USART_TX_GPIO_PORT, &gpio_init_struct);

        /* 3. ���� RX (PC10) */
        gpio_init_struct.Pin = USART_RX_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;      /* ������������ */
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Alternate = USART_RX_GPIO_AF;
        HAL_GPIO_Init(USART_RX_GPIO_PORT, &gpio_init_struct);

        /* 4. ԭ�У�DMA���ͳ�ʼ����������ͳһ���������ﲻ��Ҫ�ظ� */
        
        /* 5. ��ʹ�� UART �жϣ�����ѯ */
    }
}

/**
 * @brief  ��������ѯ�������յ������� (������ѭ�� while(1) �е���)
 */
void usart_poll(void)
{
    uint16_t current_pos = FRAME_SIZE - __HAL_DMA_GET_COUNTER(&g_hdma_usart_rx);
    uint16_t new_bytes;

    /* �������յ����ֽ��� */
    if (current_pos >= last_dma_pos) {
        new_bytes = current_pos - last_dma_pos;
    } else {
        new_bytes = (FRAME_SIZE - last_dma_pos) + current_pos;
    }

    /* ����������ֽ� */
    for (uint16_t i = 0; i < new_bytes; i++) {
        uint16_t index = (last_dma_pos + i) % FRAME_SIZE;
        uint8_t byte = g_uart_rx_buf[index];

        /* �������ֽ�ͬ������ */
        if (frame_sync(byte, Uart_recv_byte)) {
            // ֡������ɣ��ڴ�������Ĵ���
        }
    }

    /* ����λ�� */
    last_dma_pos = current_pos;

    /* ��������־ */
    if (__HAL_UART_GET_FLAG(&g_uart1_handle, UART_FLAG_ORE) != RESET) {
        __HAL_UART_CLEAR_OREFLAG(&g_uart1_handle);
    }
}

/**
 * @brief  ԭ�У��ռ��� USART3 DMA �������ͺ��� (���ֲ���)
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
 * @brief  ԭ�У���ֹDMA���� (���ֲ���)
 */
void usart3_dma_stop_send(void)
{
    HAL_UART_AbortTransmit(&g_uart1_handle);
}

/* ---------------- ��ע�ͣ�ԭ�н����ж���غ��� (��Ϊ��Ϊ����ѯ) ---------------- */
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) { ... }
void USART_UX_IRQHandler(void) { ... }
*/
