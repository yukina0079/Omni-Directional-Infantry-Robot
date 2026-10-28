#include "iic.h"

I2C_HandleTypeDef hi2c2;

void i2c_init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0x00;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0x00;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c2) != HAL_OK){}
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C2)
  {

    __HAL_RCC_I2C2_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

void i2c_write_register(uint8_t i2c_address, uint8_t address, uint8_t data)
{

  HAL_I2C_Mem_Write(&hi2c2, (i2c_address << 1), address, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

uint8_t i2c_read_register(uint8_t i2c_address, uint8_t address)
{
  uint8_t data = 0;

  HAL_I2C_Mem_Read(&hi2c2, (i2c_address << 1), address, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
  return data;
}

void i2c_write_len(uint8_t i2c_address, uint8_t reg_addr, uint8_t len, uint8_t *data)
{
  HAL_I2C_Mem_Write(&hi2c2, (i2c_address << 1), reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

void i2c_read_len(uint8_t i2c_address, uint8_t reg_addr, uint8_t len, uint8_t *data)
{
  HAL_I2C_Mem_Read(&hi2c2, (i2c_address << 1), reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}
