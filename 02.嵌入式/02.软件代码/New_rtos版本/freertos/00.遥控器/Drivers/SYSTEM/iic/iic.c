#include "iic.h"

/* 硬件I2C句柄定义 */
I2C_HandleTypeDef hi2c2;

/**
 * @brief 硬件I2C2初始化（替代原模拟I2C初始化）
 * @note  适配GPIOB10(SCL)/GPIOB11(SDA)，复用为I2C2功能
 */
void i2c_init(void)
{
  hi2c2.Instance = I2C2;                    // 选择I2C2外设
  hi2c2.Init.ClockSpeed = 400000;           // 400KHz高速模式（也可设100000为标准模式）
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;   // 占空比2:1
  hi2c2.Init.OwnAddress1 = 0x00;            // 主机模式，自身地址设为0
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT; // 7位地址模式（兼容绝大多数传感器）
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0x00;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  
  // 初始化I2C外设
  if (HAL_I2C_Init(&hi2c2) != HAL_OK){}
}

/**
 * @brief HAL库I2C底层硬件初始化（自动被HAL_I2C_Init调用）
 * @note  配置GPIO复用、时钟、上拉等
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C2)
  {
    /* 1. 使能I2C2时钟 */
    __HAL_RCC_I2C2_CLK_ENABLE();

    /* 2. 使能GPIOB时钟 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /**I2C2 GPIO配置
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;    // 开漏输出（I2C必须）
    GPIO_InitStruct.Pull = GPIO_PULLUP;        // 上拉（I2C推荐）
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

/**
 * @brief 单寄存器写（保持原接口）
 * @param i2c_address: 从机7位地址（无需左移）
 * @param address: 寄存器地址
 * @param data: 写入的数据
 */
void i2c_write_register(uint8_t i2c_address, uint8_t address, uint8_t data)
{
  // HAL_I2C_Mem_Write：主机写从机寄存器
  // 参数：I2C句柄、从机地址(7位)、寄存器地址、地址长度(8位=1)、数据指针、数据长度、超时时间
  HAL_I2C_Mem_Write(&hi2c2, (i2c_address << 1), address, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

/**
 * @brief 单寄存器读（保持原接口）
 * @param i2c_address: 从机7位地址（无需左移）
 * @param address: 寄存器地址
 * @return 读取到的数据
 */
uint8_t i2c_read_register(uint8_t i2c_address, uint8_t address)
{
  uint8_t data = 0;
  // HAL_I2C_Mem_Read：主机读从机寄存器
  HAL_I2C_Mem_Read(&hi2c2, (i2c_address << 1), address, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
  return data;
}

/**
 * @brief 多字节写（保持原接口）
 * @param i2c_address: 从机7位地址
 * @param reg_addr: 起始寄存器地址
 * @param len: 写入字节数
 * @param data: 待写入数据指针
 */
void i2c_write_len(uint8_t i2c_address, uint8_t reg_addr, uint8_t len, uint8_t *data)
{
  HAL_I2C_Mem_Write(&hi2c2, (i2c_address << 1), reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

/**
 * @brief 多字节读（保持原接口）
 * @param i2c_address: 从机7位地址
 * @param reg_addr: 起始寄存器地址
 * @param len: 读取字节数
 * @param data: 接收数据缓冲区指针
 */
void i2c_read_len(uint8_t i2c_address, uint8_t reg_addr, uint8_t len, uint8_t *data)
{
  HAL_I2C_Mem_Read(&hi2c2, (i2c_address << 1), reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

