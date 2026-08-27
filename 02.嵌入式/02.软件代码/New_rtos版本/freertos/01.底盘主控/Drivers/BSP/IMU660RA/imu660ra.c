#include "imu660ra.h"
#include "delay.h"
#include "iic_hw.h"
#include "usart.h"
#include "led.h"

extern const unsigned char imu660ra_config_file[IMU660RA_CONFIG_SIZE];

#define IMU660RA_ID_RETRY           32U
#define IMU660RA_ID_RETRY_DELAY_MS  10U
#define IMU660RA_CONFIG_SINGLE_BURST 1U

static uint8_t imu660ra_i2c_addr = IMU660RA_I2C_ADDR_HIGH;
static uint8_t imu660ra_ready = 0U;
volatile imu660ra_diag_t imu660ra_diag;

static void imu660ra_diag_capture_i2c(void)
{
    imu660ra_diag.i2c_status = iic_last_hal_status;
    imu660ra_diag.i2c_error = iic_last_error_code;
}

static void imu660ra_diag_reset(void)
{
    imu660ra_diag.magic = IMU660RA_DIAG_MAGIC;
    imu660ra_diag.i2c_error = 0U;
    imu660ra_diag.config_offset = 0U;
    imu660ra_diag.stage = 0U;
    imu660ra_diag.result = IMU660RA_ERROR_CONFIG;
    imu660ra_diag.chip_id = 0xFFU;
    imu660ra_diag.i2c_addr = IMU660RA_I2C_ADDR_HIGH;
    imu660ra_diag.internal_status = 0xFFU;
    imu660ra_diag.i2c_status = 0xFFU;
    imu660ra_diag.probe_low_chip_id = 0xFFU;
    imu660ra_diag.probe_high_chip_id = 0xFFU;
    imu660ra_diag.probe_found_mask = 0U;
}

static uint8_t imu660ra_read(uint8_t reg, uint8_t *data, uint8_t len)
{
    return iic_read_len(imu660ra_i2c_addr, reg, data, len);
}

static uint8_t imu660ra_write(uint8_t reg, uint8_t data)
{
    return iic_write_len(imu660ra_i2c_addr, reg, &data, 1U);
}

#if !IMU660RA_CONFIG_SINGLE_BURST
static uint8_t imu660ra_set_init_addr(uint16_t address)
{
    uint8_t low = (uint8_t)(address & 0xFFU);
    uint8_t high = (uint8_t)((address >> 8) & 0x0FU);

    if (imu660ra_write(IMU660RA_INIT_ADDR_0_REG, low) != 0U)
    {
        return 1U;
    }
    return imu660ra_write(IMU660RA_INIT_ADDR_1_REG, high);
}
#endif

static uint8_t imu660ra_load_config(void)
{
    uint8_t status;
    uint8_t i;
#if !IMU660RA_CONFIG_SINGLE_BURST
    uint16_t offset;
#endif

    /* Datasheet: APS is on after POR / soft reset. Disable it and wait >=450 us
     * before any further write, or the next access hits a sleeping interface. */
    imu660ra_diag.stage = IMU660RA_DIAG_STAGE_PWR_CONF;
    if (imu660ra_write(IMU660RA_PWR_CONF_REG, 0x00U) != 0U)
    {
        imu660ra_diag_capture_i2c();
        return 1U;
    }
    delay_ms(1U);

    imu660ra_diag.stage = IMU660RA_DIAG_STAGE_INIT_CTRL_START;
    if (imu660ra_write(IMU660RA_INIT_CTRL_REG, 0x00U) != 0U)
    {
        imu660ra_diag_capture_i2c();
        return 1U;
    }

    /* The IMU660RA module's supplied I2C example streams all 8192 bytes in one
     * transaction to INIT_DATA; it does not use BMI270 INIT_ADDR_0/1 paging. */
#if IMU660RA_CONFIG_SINGLE_BURST
    imu660ra_diag.config_offset = 0U;
    imu660ra_diag.stage = IMU660RA_DIAG_STAGE_INIT_DATA;
    if (iic_write_len(imu660ra_i2c_addr,
                      IMU660RA_INIT_DATA_REG,
                      imu660ra_config_file,
                      IMU660RA_CONFIG_SIZE) != 0U)
    {
        imu660ra_diag_capture_i2c();
        return 1U;
    }
#else
    for (offset = 0U; offset < IMU660RA_CONFIG_SIZE; offset += IMU660RA_CONFIG_CHUNK)
    {
        imu660ra_diag.config_offset = offset;
        imu660ra_diag.stage = IMU660RA_DIAG_STAGE_INIT_ADDR;
        if (imu660ra_set_init_addr((uint16_t)(offset >> 1)) != 0U)
        {
            imu660ra_diag_capture_i2c();
            return 1U;
        }
        imu660ra_diag.stage = IMU660RA_DIAG_STAGE_INIT_DATA;
        if (iic_write_len(imu660ra_i2c_addr,
                          IMU660RA_INIT_DATA_REG,
                          &imu660ra_config_file[offset],
                          IMU660RA_CONFIG_CHUNK) != 0U)
        {
            imu660ra_diag_capture_i2c();
            return 1U;
        }
    }
#endif

    imu660ra_diag.stage = IMU660RA_DIAG_STAGE_INIT_CTRL_END;
    if (imu660ra_write(IMU660RA_INIT_CTRL_REG, 0x01U) != 0U)
    {
        imu660ra_diag_capture_i2c();
        return 1U;
    }

    /* INTERNAL_STATUS[3:0] == 0x01 is "init OK". Bit0-only would also pass
     * error codes 0x03/0x05. Poll up to ~150 ms as in the Bosch BMI2 API. */
    for (i = 0U; i < 30U; i++)
    {
        delay_ms(5U);
        imu660ra_diag.stage = IMU660RA_DIAG_STAGE_INTERNAL_STATUS;
        status = iic_read_register(imu660ra_i2c_addr, IMU660RA_INTERNAL_STATUS_REG);
        imu660ra_diag.internal_status = status;
        imu660ra_diag_capture_i2c();
        if ((status & 0x0FU) == 0x01U)
        {
            return 0U;
        }
    }
    printf("IMU660RA INTERNAL_STATUS=0x%02X (want 0x01)\r\n", status);
    return 1U;
}

uint8_t IMU660RA_get_chip_id(void)
{
    return iic_read_register(imu660ra_i2c_addr, IMU660RA_CHIP_ID_REG);
}

uint8_t IMU660RA_get_i2c_addr(void)
{
    return imu660ra_i2c_addr;
}

uint8_t IMU660RA_is_ready(void)
{
    return imu660ra_ready;
}

static uint8_t imu660ra_probe_addr(uint8_t addr)
{
    uint8_t chip_id = 0xFFU;
    uint8_t i;

    imu660ra_i2c_addr = addr;

    /* BMI270 powers up with Advanced Power Save on. The first I2C access only
     * wakes the interface and often returns 0x00 / NACK. Official Seekfree
     * code retries CHIP_ID up to 32 times; one shot here is why a live module
     * was reported as "not found". */
    (void)IMU660RA_get_chip_id();
    delay_ms(1U);

    for (i = 0U; i < IMU660RA_ID_RETRY; i++)
    {
        chip_id = IMU660RA_get_chip_id();
        if (chip_id == IMU660RA_CHIP_ID)
        {
            return chip_id;
        }
        delay_ms(IMU660RA_ID_RETRY_DELAY_MS);
    }

    printf("IMU660RA probe 0x%02X last_id=0x%02X ready=%u\r\n",
           addr, chip_id, iic_device_ready(addr));
    return chip_id;
}

IMU660RA_Status IMU660RA_init(void)
{
    uint8_t chip_id_high;
    uint8_t chip_id_low;

    imu660ra_ready = 0U;
    imu660ra_diag_reset();
    LED1(0); /* PD3: 0x68 probe result */
    LED2(0); /* PD4: 0x69 probe result */
    delay_ms(20U);

    imu660ra_diag.stage = IMU660RA_DIAG_STAGE_PROBE_HIGH;
    chip_id_high = imu660ra_probe_addr(IMU660RA_I2C_ADDR_HIGH);
    imu660ra_diag.probe_high_chip_id = chip_id_high;
    if (chip_id_high == IMU660RA_CHIP_ID)
    {
        imu660ra_diag.probe_found_mask |= IMU660RA_PROBE_HIGH_FOUND;
    }
    else
    {
        iic_bus_recover();
    }
    LED2(chip_id_high == IMU660RA_CHIP_ID);

    /* Always test both SA0 choices. A successful 0x69 probe must not skip
     * the 0x68 self-check, because SA0 may be floating on the fitted module. */
    imu660ra_diag.stage = IMU660RA_DIAG_STAGE_PROBE_LOW;
    chip_id_low = imu660ra_probe_addr(IMU660RA_I2C_ADDR_LOW);
    imu660ra_diag.probe_low_chip_id = chip_id_low;
    if (chip_id_low == IMU660RA_CHIP_ID)
    {
        imu660ra_diag.probe_found_mask |= IMU660RA_PROBE_LOW_FOUND;
    }
    else
    {
        iic_bus_recover();
    }
    LED1(chip_id_low == IMU660RA_CHIP_ID);

    printf("IMU660RA self-check: 0x68 id=0x%02X %s, 0x69 id=0x%02X %s\r\n",
           chip_id_low, (chip_id_low == IMU660RA_CHIP_ID) ? "OK" : "FAIL",
           chip_id_high, (chip_id_high == IMU660RA_CHIP_ID) ? "OK" : "FAIL");

    /* Preserve the previous address priority when both addresses respond. */
    if (chip_id_high == IMU660RA_CHIP_ID)
    {
        imu660ra_i2c_addr = IMU660RA_I2C_ADDR_HIGH;
        imu660ra_diag.chip_id = chip_id_high;
    }
    else if (chip_id_low == IMU660RA_CHIP_ID)
    {
        imu660ra_i2c_addr = IMU660RA_I2C_ADDR_LOW;
        imu660ra_diag.chip_id = chip_id_low;
    }
    else
    {
        imu660ra_diag.chip_id = chip_id_low;
        imu660ra_diag.i2c_addr = imu660ra_i2c_addr;
        imu660ra_diag_capture_i2c();
        imu660ra_diag.result = IMU660RA_ERROR_NOT_FOUND;
        return IMU660RA_ERROR_NOT_FOUND;
    }

    imu660ra_diag.i2c_addr = imu660ra_i2c_addr;
    imu660ra_diag_capture_i2c();

    /* CMD=0xB6 soft-reset. Datasheet min wait 1 ms; Bosch API uses 2 ms.
     * Soft reset does NOT re-sample CSB, so SPI/I2C stays as latched at POR. */
    imu660ra_diag.stage = IMU660RA_DIAG_STAGE_SOFT_RESET;
    if (imu660ra_write(IMU660RA_CMD_REG, IMU660RA_SOFT_RESET_CMD) != 0U)
    {
        imu660ra_diag_capture_i2c();
        LED1(0);
        LED2(0);
        imu660ra_diag.result = IMU660RA_ERROR_CONFIG;
        return IMU660RA_ERROR_CONFIG;
    }
    delay_ms(2U);
    imu660ra_diag.stage = IMU660RA_DIAG_STAGE_RESET_CHIP_ID;
    imu660ra_diag.chip_id = IMU660RA_get_chip_id();
    imu660ra_diag_capture_i2c();
    delay_ms(1U);

    if (imu660ra_load_config() != 0U)
    {
        LED1(0);
        LED2(0);
        imu660ra_diag.result = IMU660RA_ERROR_CONFIG;
        return IMU660RA_ERROR_CONFIG;
    }

    /* Configure ODR/range first, then enable sensors (Bosch BMI2 order).
     * Seekfree uses ACC_CONF=0xA7 (50 Hz) for a print loop. The chassis AHRS
     * runs at 1 kHz, so keep 0xAB = perf + NORM + 800 Hz. Range macros match
     * Seekfree IMU660RA_ACC_SAMPLE / IMU660RA_GYR_SAMPLE. */
    imu660ra_diag.stage = IMU660RA_DIAG_STAGE_SENSOR_CONFIG;
    if ((imu660ra_write(IMU660RA_ACC_CONF_REG, 0xABU) != 0U) ||
        (imu660ra_write(IMU660RA_ACC_RANGE_REG, IMU660RA_ACC_SAMPLE) != 0U) ||
        (imu660ra_write(IMU660RA_GYR_CONF_REG, 0xABU) != 0U) ||
        (imu660ra_write(IMU660RA_GYR_RANGE_REG, IMU660RA_GYR_SAMPLE) != 0U) ||
        (imu660ra_write(IMU660RA_PWR_CTRL_REG, 0x0EU) != 0U))
    {
        imu660ra_diag_capture_i2c();
        LED1(0);
        LED2(0);
        imu660ra_diag.result = IMU660RA_ERROR_CONFIG;
        return IMU660RA_ERROR_CONFIG;
    }
    /* Gyro typical startup is ~45 ms after gyr_en. */
    delay_ms(50U);

    imu660ra_ready = 1U;
    imu660ra_diag.stage = IMU660RA_DIAG_STAGE_DONE;
    imu660ra_diag.result = IMU660RA_OK;
    imu660ra_diag_capture_i2c();
    return IMU660RA_OK;
}

IMU660RA_Status IMU660RA_get_data(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                                  int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ,
                                  int16_t *Temperature)
{
    uint8_t buf[12];

    if ((imu660ra_ready == 0U) || (AccX == NULL) || (AccY == NULL) || (AccZ == NULL) ||
        (GyroX == NULL) || (GyroY == NULL) || (GyroZ == NULL) || (Temperature == NULL))
    {
        return IMU660RA_ERROR_DATA;
    }

    if (imu660ra_read(IMU660RA_ACC_DATA_REG, buf, sizeof(buf)) != 0U)
    {
        return IMU660RA_ERROR_DATA;
    }

    *AccX = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    *AccY = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    *AccZ = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
    *GyroX = (int16_t)(((uint16_t)buf[7] << 8) | buf[6]);
    *GyroY = (int16_t)(((uint16_t)buf[9] << 8) | buf[8]);
    *GyroZ = (int16_t)(((uint16_t)buf[11] << 8) | buf[10]);

    if (imu660ra_read(IMU660RA_TEMP_DATA_REG, buf, 2U) == 0U)
    {
        *Temperature = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    }
    else
    {
        *Temperature = 0;
    }
    return IMU660RA_OK;
}

float IMU660RA_acc_transition(int16_t acc_value)
{
    switch (IMU660RA_ACC_SAMPLE)
    {
        case 0x00U: return (float)acc_value / 16384.0f;
        case 0x01U: return (float)acc_value / 8192.0f;
        case 0x02U: return (float)acc_value / 4096.0f;
        case 0x03U: return (float)acc_value / 2048.0f;
        default:    return 0.0f;
    }
}

float IMU660RA_gyro_transition(int16_t gyro_value)
{
    switch (IMU660RA_GYR_SAMPLE)
    {
        case 0x00U: return (float)gyro_value / 16.4f;
        case 0x01U: return (float)gyro_value / 32.8f;
        case 0x02U: return (float)gyro_value / 65.6f;
        case 0x03U: return (float)gyro_value / 131.2f;
        case 0x04U: return (float)gyro_value / 262.4f;
        default:    return 0.0f;
    }
}

IMU660RA_Status IMU660RA_update(float *AX, float *AY, float *AZ,
                                float *GX, float *GY, float *GZ,
                                float *Temperature)
{
    int16_t ax, ay, az, gx, gy, gz, temp;

    if ((AX == NULL) || (AY == NULL) || (AZ == NULL) ||
        (GX == NULL) || (GY == NULL) || (GZ == NULL) || (Temperature == NULL))
    {
        return IMU660RA_ERROR_DATA;
    }

    if (IMU660RA_get_data(&ax, &ay, &az, &gx, &gy, &gz, &temp) != IMU660RA_OK)
    {
        return IMU660RA_ERROR_DATA;
    }

    *AX = IMU660RA_acc_transition(ax);
    *AY = IMU660RA_acc_transition(ay);
    *AZ = IMU660RA_acc_transition(az);
    *GX = IMU660RA_gyro_transition(gx);
    *GY = IMU660RA_gyro_transition(gy);
    *GZ = IMU660RA_gyro_transition(gz);
    /* BMI270: temperature_C = raw / 512 + 23 */
    *Temperature = ((float)temp / 512.0f) + 23.0f;
    return IMU660RA_OK;
}
