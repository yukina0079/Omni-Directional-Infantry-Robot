#include "spi.h"

SPI_HandleTypeDef g_spi2_handler;

void spi2_init(void)
{
    SPI2_SPI_CLK_ENABLE();

    g_spi2_handler.Instance = SPI2_SPI;
    g_spi2_handler.Init.Mode = SPI_MODE_MASTER;
    g_spi2_handler.Init.Direction = SPI_DIRECTION_2LINES;
    g_spi2_handler.Init.DataSize = SPI_DATASIZE_8BIT;

    g_spi2_handler.Init.CLKPolarity = SPI_POLARITY_LOW;
    g_spi2_handler.Init.CLKPhase = SPI_PHASE_1EDGE;

    g_spi2_handler.Init.NSS = SPI_NSS_SOFT;
    g_spi2_handler.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    g_spi2_handler.Init.FirstBit = SPI_FIRSTBIT_MSB;
    g_spi2_handler.Init.TIMode = SPI_TIMODE_DISABLE;
    g_spi2_handler.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    g_spi2_handler.Init.CRCPolynomial = 7;
    HAL_SPI_Init(&g_spi2_handler);

    __HAL_SPI_ENABLE(&g_spi2_handler);

}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef gpio_init_struct;

    if (hspi->Instance == SPI2_SPI)
    {
        SPI2_SCK_GPIO_CLK_ENABLE();
        SPI2_MISO_GPIO_CLK_ENABLE();
        SPI2_MOSI_GPIO_CLK_ENABLE();

        gpio_init_struct.Pin = SPI2_SCK_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_PULLUP;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(SPI2_SCK_GPIO_PORT, &gpio_init_struct);

        gpio_init_struct.Pin = SPI2_MISO_GPIO_PIN;
        HAL_GPIO_Init(SPI2_MISO_GPIO_PORT, &gpio_init_struct);

        gpio_init_struct.Pin = SPI2_MOSI_GPIO_PIN;
        HAL_GPIO_Init(SPI2_MOSI_GPIO_PORT, &gpio_init_struct);
    }
}

void spi2_set_speed(uint8_t speed)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(speed));
    __HAL_SPI_DISABLE(&g_spi2_handler);
    g_spi2_handler.Instance->CR1 &= 0XFFC7;
    g_spi2_handler.Instance->CR1 |= speed << 3;
    __HAL_SPI_ENABLE(&g_spi2_handler);
}

uint8_t spi2_read_write_byte(uint8_t txdata)
{
    uint8_t rxdata;
    HAL_SPI_TransmitReceive(&g_spi2_handler, &txdata, &rxdata, 1, 1000);
    return rxdata;
}
