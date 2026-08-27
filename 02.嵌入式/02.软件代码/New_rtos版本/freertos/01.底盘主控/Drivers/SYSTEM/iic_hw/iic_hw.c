#include "iic_hw.h"
#include "delay.h"
#include "usart.h"

#define IIC_TEST_CLOCK_HZ   400000U
#define IIC_HW_CLOCK_HZ     IIC_TEST_CLOCK_HZ
#define IIC_SCAN_DELAY_US   ((IIC_TEST_CLOCK_HZ <= 10000U) ? 50U : 5U)

/* I2C句柄定义 */
I2C_HandleTypeDef hi2c2;
volatile iic_scan_log_t iic_scan_log;
volatile iic_line_test_t iic_line_test;
volatile uint8_t iic_last_hal_status;
volatile uint32_t iic_last_error_code;

/**
 * @brief 手动配置GPIO引脚（硬件IIC专用）
 * @note  PB10->I2C2_SCL, PB11->I2C2_SDA
 */
static void IIC_HW_GPIO_Config(void)
{
    GPIO_InitTypeDef gpio_init_struct = {0};

    /* 1. 使能GPIO时钟 */
    IIC_HW_GPIO_CLK_ENABLE();

    /* 2. 配置SCL引脚：复用开漏 + 上拉 + 高速 */
    gpio_init_struct.Pin = IIC_HW_SCL_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_OD;          // 硬件IIC必须开漏复用
    gpio_init_struct.Pull = GPIO_PULLUP;              // 上拉（IIC总线必备）
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init_struct.Alternate = GPIO_AF4_I2C2;       // PB10复用为I2C2_SCL（查STM32手册）
    HAL_GPIO_Init(IIC_HW_GPIO_PORT, &gpio_init_struct);

    /* 3. 配置SDA引脚：复用开漏 + 上拉 + 高速 */
    gpio_init_struct.Pin = IIC_HW_SDA_PIN;
    gpio_init_struct.Alternate = GPIO_AF4_I2C2;       // PB11复用为I2C2_SDA
    HAL_GPIO_Init(IIC_HW_GPIO_PORT, &gpio_init_struct);
}

/**
 * @brief 手动配置I2C外设寄存器（HAL库底层）
 */
static void IIC_HW_I2C_Config(void)
{
    /* 1. 使能I2C时钟 */
    IIC_HW_CLK_ENABLE();

    /* 2. 初始化I2C句柄 */
    hi2c2.Instance = IIC_HW_I2Cx;
    hi2c2.Init.ClockSpeed = IIC_HW_CLOCK_HZ;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;           // 占空比2:1
    hi2c2.Init.OwnAddress1 = 0x00;                    // 主机模式，自身地址设为0
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT; // 7位地址模式
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0x00;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    /* 3. 初始化I2C外设（HAL库核心函数） */
    if (HAL_I2C_Init(&hi2c2) != HAL_OK)
    {
        /* 初始化失败处理（可根据需求加断言/重启） */
        while (1);
    }
}

/**
 * @brief Recover I2C2 after a NACK / timeout.
 *
 * STM32F4 I2C leaves BUSY set after a failed polling transfer. The next
 * HAL_I2C_Mem_Read then times out immediately, so a miss at 0x69 makes a
 * perfectly good device at 0x68 look absent. A peripheral reset clears that.
 */
void iic_bus_recover(void)
{
    HAL_I2C_DeInit(&hi2c2);
    __HAL_RCC_I2C2_CLK_ENABLE();
    __HAL_RCC_I2C2_FORCE_RESET();
    __HAL_RCC_I2C2_RELEASE_RESET();
    IIC_HW_I2C_Config();
}

uint8_t iic_device_ready(uint8_t i2c_addr)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(i2c_addr << 1), 2U, IIC_HW_TIMEOUT);
    iic_last_hal_status = (uint8_t)status;
    iic_last_error_code = hi2c2.ErrorCode;
    if (status == HAL_OK)
    {
        return 1U;
    }

    iic_bus_recover();
    return 0U;
}

/* Bit-bang on PB10/PB11. Hardware I2C2 is turned off first so a stuck BUSY
 * flag cannot hide a live slave. Open-drain + write-1 releases the line. */
#define IIC_SCAN_SCL(x)  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define IIC_SCAN_SDA(x)  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define IIC_SCAN_SDA_IN() HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)

static void iic_scan_delay(void)
{
    delay_us(IIC_SCAN_DELAY_US);
}

static void iic_scan_gpio_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    HAL_I2C_DeInit(&hi2c2);
    __HAL_RCC_I2C2_CLK_DISABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
    IIC_SCAN_SCL(1);
    IIC_SCAN_SDA(1);
    delay_us(20U);
}

static void iic_scan_line_test(void)
{
    GPIO_InitTypeDef gpio = {0};

    IIC_SCAN_SCL(1);
    IIC_SCAN_SDA(1);
    delay_us(20U);
    iic_line_test.pullup_20us_scl = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
    iic_line_test.pullup_20us_sda = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);

    delay_us(1000U);
    iic_line_test.pullup_1ms_scl = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
    iic_line_test.pullup_1ms_sda = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);

    /* Remove only the MCU's pull-up. An externally pulled-up bus stays high. */
    gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
    IIC_SCAN_SCL(1);
    IIC_SCAN_SDA(1);
    delay_us(1000U);
    iic_line_test.nopull_1ms_scl = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
    iic_line_test.nopull_1ms_sda = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);

    /* Restore the normal scan configuration. */
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpio);
    IIC_SCAN_SCL(1);
    IIC_SCAN_SDA(1);
}

static void iic_scan_start(void)
{
    IIC_SCAN_SDA(1);
    IIC_SCAN_SCL(1);
    iic_scan_delay();
    IIC_SCAN_SDA(0);
    iic_scan_delay();
    IIC_SCAN_SCL(0);
    iic_scan_delay();
}

static void iic_scan_stop(void)
{
    IIC_SCAN_SDA(0);
    iic_scan_delay();
    IIC_SCAN_SCL(1);
    iic_scan_delay();
    IIC_SCAN_SDA(1);
    iic_scan_delay();
}

static uint8_t iic_scan_write_byte(uint8_t data)
{
    uint8_t i;
    uint8_t ack;

    for (i = 0U; i < 8U; i++)
    {
        IIC_SCAN_SDA((data & 0x80U) != 0U);
        iic_scan_delay();
        IIC_SCAN_SCL(1);
        iic_scan_delay();
        IIC_SCAN_SCL(0);
        data <<= 1;
    }

    IIC_SCAN_SDA(1);
    iic_scan_delay();
    IIC_SCAN_SCL(1);
    iic_scan_delay();
    ack = (IIC_SCAN_SDA_IN() == GPIO_PIN_RESET) ? 1U : 0U;
    IIC_SCAN_SCL(0);
    iic_scan_delay();
    return ack;
}

static uint8_t iic_scan_read_byte(uint8_t send_ack)
{
    uint8_t i;
    uint8_t data = 0U;

    IIC_SCAN_SDA(1);
    for (i = 0U; i < 8U; i++)
    {
        data <<= 1;
        IIC_SCAN_SCL(1);
        iic_scan_delay();
        if (IIC_SCAN_SDA_IN() != GPIO_PIN_RESET)
        {
            data |= 1U;
        }
        IIC_SCAN_SCL(0);
        iic_scan_delay();
    }

    IIC_SCAN_SDA(send_ack ? 0U : 1U);
    iic_scan_delay();
    IIC_SCAN_SCL(1);
    iic_scan_delay();
    IIC_SCAN_SCL(0);
    IIC_SCAN_SDA(1);
    iic_scan_delay();
    return data;
}

static uint8_t iic_scan_read_reg0(uint8_t addr)
{
    uint8_t id = 0xFFU;

    iic_scan_start();
    if (iic_scan_write_byte((uint8_t)(addr << 1)) == 0U)
    {
        iic_scan_stop();
        return 0xFFU;
    }
    if (iic_scan_write_byte(0x00U) == 0U)
    {
        iic_scan_stop();
        return 0xFFU;
    }
    iic_scan_start();
    if (iic_scan_write_byte((uint8_t)((addr << 1) | 1U)) == 0U)
    {
        iic_scan_stop();
        return 0xFFU;
    }
    id = iic_scan_read_byte(0U);
    iic_scan_stop();
    return id;
}

static const char *iic_scan_hint(uint8_t addr, uint8_t id)
{
    if ((addr == 0x68U) || (addr == 0x69U))
    {
        if (id == 0x24U) { return "IMU660RA/BMI270"; }
        if (id == 0x0FU) { return "BMI088/BMI055 gyro"; }
        return "possible IMU (0x68/0x69)";
    }
    if (addr == 0x18U)
    {
        if (id == 0x1EU) { return "BMI088 accel"; }
        if (id == 0xFAU) { return "BMI055 accel"; }
        return "possible BMI accel";
    }
    if ((addr == 0x3CU) || (addr == 0x3DU)) { return "OLED SSD1306"; }
    if ((addr >= 0x0CU) && (addr <= 0x0FU)) { return "possible IST8310"; }
    return "";
}

void iic_scan(void)
{
    uint8_t addr;
    uint8_t found = 0U;
    uint8_t idle_scl;
    uint8_t idle_sda;

    printf("\r\n======== I2C scan PB10=SCL PB11=SDA ========\r\n");
    iic_scan_log.magic = 0U;
    iic_scan_log.done = 0U;
    iic_scan_log.found = 0U;
    iic_scan_gpio_init();
    iic_scan_line_test();

    idle_scl = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
    idle_sda = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);
    iic_scan_log.idle_scl = idle_scl;
    iic_scan_log.idle_sda = idle_sda;
    printf("idle SCL=%u SDA=%u (both should be 1; 0 = no pull-up or bus stuck)\r\n",
           idle_scl, idle_sda);
    printf("scan 0x08 -> 0x77\r\n");

    for (addr = 0x08U; addr <= 0x77U; addr++)
    {
        iic_scan_start();
        if (iic_scan_write_byte((uint8_t)(addr << 1)) != 0U)
        {
            uint8_t id;
            const char *hint;

            iic_scan_stop();
            id = iic_scan_read_reg0(addr);
            hint = iic_scan_hint(addr, id);
            printf("  ACK 0x%02X  reg0=0x%02X  %s\r\n", addr, id, hint);
            if (found < IIC_SCAN_LOG_MAX)
            {
                iic_scan_log.addr[found] = addr;
                iic_scan_log.id[found] = id;
            }
            found++;
        }
        else
        {
            iic_scan_stop();
        }
    }

    if (found == 0U)
    {
        printf("  no ACK. IMU660RA still in SPI, or CS held low, or no pull-up.\r\n");
    }
    else
    {
        printf("found %u device(s)\r\n", found);
    }
    printf("============================================\r\n");

    iic_scan_log.found = found;
    iic_scan_log.magic = IIC_SCAN_LOG_MAGIC;
    iic_scan_log.done = 1U;

    iic_init();
}

/**
 * @brief 硬件IIC初始化（替换原模拟IIC的iic_init函数）
 */
void iic_init(void)
{
    /* Clock + reset the peripheral BEFORE the pins go AF. Enabling AF while
     * I2C2 is still gated can glitch SDA/SCL low and leave BUSY stuck. */
    IIC_HW_GPIO_CLK_ENABLE();
    IIC_HW_CLK_ENABLE();
    __HAL_RCC_I2C2_FORCE_RESET();
    __HAL_RCC_I2C2_RELEASE_RESET();

    IIC_HW_GPIO_Config();
    IIC_HW_I2C_Config();
}

/**
 * @brief 硬件IIC写入单个寄存器（兼容原函数名）
 * @param i2c_addr: I2C设备7位地址（无需左移）
 * @param reg_addr: 寄存器地址
 * @param data: 要写入的数据
 */
void iic_write_register(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data)
{

    /* HAL库硬件IIC写寄存器：核心函数 */
    HAL_I2C_Mem_Write(&hi2c2,
                      (i2c_addr << 1),  // 7位地址左移1位（HAL自动补写位0）
                      reg_addr,
                      I2C_MEMADD_SIZE_8BIT,  // 寄存器地址8位
                      &data,
                      1,
                      IIC_HW_TIMEOUT);

}

uint8_t iic_write_len(uint8_t i2c_addr, uint8_t reg_addr, const uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef status;

    if ((buf == NULL) || (len == 0U))
    {
        return 1U;
    }

    status = HAL_I2C_Mem_Write(&hi2c2,
                               (i2c_addr << 1),
                               reg_addr,
                               I2C_MEMADD_SIZE_8BIT,
                               (uint8_t *)buf,
                               len,
                               IIC_HW_TIMEOUT);
    iic_last_hal_status = (uint8_t)status;
    iic_last_error_code = hi2c2.ErrorCode;
    if (status != HAL_OK)
    {
        iic_bus_recover();
        return 1U;
    }
    return 0U;
}

/**
 * @brief 硬件IIC读取单个寄存器（兼容原函数名）
 * @param i2c_addr: I2C设备7位地址
 * @param reg_addr: 寄存器地址
 * @retval 读取到的数据（失败返回0xFF）
 */
uint8_t iic_read_register(uint8_t i2c_addr, uint8_t reg_addr)
{
    uint8_t data = 0xFF;
    HAL_StatusTypeDef status;

    /* HAL库硬件IIC读寄存器：核心函数 */
    status = HAL_I2C_Mem_Read(&hi2c2,
                         (i2c_addr << 1),  // 7位地址左移1位（HAL自动补读位1）
                         reg_addr,
                         I2C_MEMADD_SIZE_8BIT,
                         &data,
                         1,
                         IIC_HW_TIMEOUT);
    iic_last_hal_status = (uint8_t)status;
    iic_last_error_code = hi2c2.ErrorCode;
    if (status != HAL_OK)
    {
        iic_bus_recover();
        data = 0xFF;  // 读取失败返回0xFF（兼容原模拟IIC逻辑）
    }

    return data;
}

/**
 * @brief 硬件IIC读取多字节（兼容原函数名）
 * @param i2c_addr: I2C设备7位地址
 * @param reg_addr: 起始寄存器地址
 * @param buf: 数据缓冲区
 * @param len: 读取长度
 * @retval 0:成功, 1:失败（兼容原模拟IIC返回值）
 */
uint8_t iic_read_len(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buf, uint8_t len)
{
    uint8_t ret = 1;  // 默认失败
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(&hi2c2,
                         (i2c_addr << 1),
                         reg_addr,
                         I2C_MEMADD_SIZE_8BIT,
                         buf,
                         len,
                         IIC_HW_TIMEOUT_FAST);
    iic_last_hal_status = (uint8_t)status;
    iic_last_error_code = hi2c2.ErrorCode;
    if (status == HAL_OK)
    {
        ret = 0;  // 读取成功
    }
    else
    {
        iic_bus_recover();
    }

    return ret;
}

/************************ HAL库底层回调（可选） ************************/
/**
 * @brief I2C错误回调函数（调试用）
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == IIC_HW_I2Cx)
    {
        /* 可添加错误处理：重启IIC、打印日志等 */
        HAL_I2C_DeInit(hi2c);
        IIC_HW_I2C_Config();
    }
}

