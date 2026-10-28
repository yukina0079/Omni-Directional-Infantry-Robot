#include "iic_hw.h"
#include "delay.h"

I2C_HandleTypeDef hi2c2;

static void IIC_HW_GPIO_Config(void)
{
    GPIO_InitTypeDef gpio_init_struct = {0};

    IIC_HW_GPIO_CLK_ENABLE();

    gpio_init_struct.Pin = IIC_HW_SCL_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_OD;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init_struct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(IIC_HW_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = IIC_HW_SDA_PIN;
    gpio_init_struct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(IIC_HW_GPIO_PORT, &gpio_init_struct);
}

static void IIC_HW_I2C_Config(void)
{

    IIC_HW_CLK_ENABLE();

    hi2c2.Instance = IIC_HW_I2Cx;
    hi2c2.Init.ClockSpeed = 400000;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0x00;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0x00;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c2) != HAL_OK)
    {

        while (1);
    }
}

static uint8_t iic_hw_bus_ready(void)
{
    return (uint8_t)((IIC_HW_I2Cx->SR2 & I2C_SR2_BUSY) == 0);
}

static void iic_hw_recover_if_busy(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint8_t i;
    static uint32_t last_ms;

    if (iic_hw_bus_ready())
    {
        return;
    }

    if ((HAL_GetTick() - last_ms) < 20u)
    {
        return;
    }
    last_ms = HAL_GetTick();

    __HAL_I2C_DISABLE(&hi2c2);
    IIC_HW_I2Cx->CR1 |= I2C_CR1_SWRST;
    IIC_HW_I2Cx->CR1 &= (uint16_t)~I2C_CR1_SWRST;

    gpio.Pin = IIC_HW_SCL_PIN | IIC_HW_SDA_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(IIC_HW_GPIO_PORT, &gpio);

    HAL_GPIO_WritePin(IIC_HW_GPIO_PORT, IIC_HW_SDA_PIN, GPIO_PIN_SET);
    for (i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(IIC_HW_GPIO_PORT, IIC_HW_SCL_PIN, GPIO_PIN_SET);
        delay_us(5);
        HAL_GPIO_WritePin(IIC_HW_GPIO_PORT, IIC_HW_SCL_PIN, GPIO_PIN_RESET);
        delay_us(5);
    }
    HAL_GPIO_WritePin(IIC_HW_GPIO_PORT, IIC_HW_SDA_PIN, GPIO_PIN_RESET);
    delay_us(5);
    HAL_GPIO_WritePin(IIC_HW_GPIO_PORT, IIC_HW_SCL_PIN, GPIO_PIN_SET);
    delay_us(5);
    HAL_GPIO_WritePin(IIC_HW_GPIO_PORT, IIC_HW_SDA_PIN, GPIO_PIN_SET);
    delay_us(5);

    IIC_HW_GPIO_Config();
    IIC_HW_I2C_Config();
    (void)IIC_HW_I2Cx->SR1;
    (void)IIC_HW_I2Cx->SR2;
}

void iic_init(void)
{
    IIC_HW_GPIO_Config();
    IIC_HW_I2C_Config();
    iic_hw_recover_if_busy();
}

void iic_write_register(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data)
{

    iic_hw_recover_if_busy();
    HAL_I2C_Mem_Write(&hi2c2,
                      (uint16_t)(i2c_addr << 1),
                      reg_addr,
                      I2C_MEMADD_SIZE_8BIT,
                      &data,
                      1,
                      IIC_HW_TIMEOUT);
}

uint8_t iic_read_register(uint8_t i2c_addr, uint8_t reg_addr)
{
    uint8_t data = 0xFF;

    iic_hw_recover_if_busy();
    if (HAL_I2C_Mem_Read(&hi2c2,
                         (uint16_t)(i2c_addr << 1),
                         reg_addr,
                         I2C_MEMADD_SIZE_8BIT,
                         &data,
                         1,
                         IIC_HW_TIMEOUT) != HAL_OK)
    {
        data = 0xFF;
    }
    return data;
}

uint8_t iic_read_len(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buf, uint8_t len)
{
    iic_hw_recover_if_busy();
    if (HAL_I2C_Mem_Read(&hi2c2,
                         (uint16_t)(i2c_addr << 1),
                         reg_addr,
                         I2C_MEMADD_SIZE_8BIT,
                         buf,
                         len,
                         IIC_HW_TIMEOUT) == HAL_OK)
    {
        return 0;
    }
    return 1;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == IIC_HW_I2Cx)
    {

        HAL_I2C_DeInit(hi2c);
        IIC_HW_I2C_Config();
    }
}
