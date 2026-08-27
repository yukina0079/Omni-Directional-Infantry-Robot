#include "bmi055.h"
#include "usart.h"
#include "delay.h"
#include "iic.h"
#include "math.h"
#include <stdio.h>

/* Official datasheet sensitivities */
#define BMI055_ACCEL_4G_SEN  0.001953125f
#define BMI055_GYRO_2000_SEN 0.06103515625f

static uint8_t bmi055_write_check(uint8_t addr, uint8_t reg, uint8_t val, uint8_t mask)
{
    uint8_t rd;
    uint8_t try_n;

    for (try_n = 0; try_n < 3u; try_n++)
    {
        iic_write_register(addr, reg, val);
        delay_ms(2);
        rd = iic_read_register(addr, reg);
        /* Reserved bits (e.g. gyro 0x10 bit7) may read back as 1. */
        if ((rd & mask) == (val & mask))
        {
            return 0;
        }
        delay_ms(2);
    }
    printf("BMI055 cfg mismatch addr=0x%02X reg=0x%02X wr=0x%02X rd=0x%02X\r\n",
           addr, reg, val, rd);
    return 1;
}

bmi055_status_t BMI055_update(float *AX, float *AY, float *AZ,
                              float *GX, float *GY, float *GZ,
                              float *Temperature)
{
    static float last_a[3] = {0.0f, 0.0f, 0.0f};
    static float last_g[3] = {0.0f, 0.0f, 0.0f};
    static float last_t = 0.0f;
    int16_t ax = 0, ay = 0, az = 0;
    int16_t gx = 0, gy = 0, gz = 0;
    int16_t temp_raw = 0;
    uint8_t flags;

    flags = BMI055_get_data(&ax, &ay, &az, &gx, &gy, &gz, &temp_raw);

    if (flags & BMI055_FLAG_ACCEL)
    {
        /* 12-bit left-aligned in 16-bit register: arithmetic shift keeps sign */
        last_a[0] = (float)(ax >> 4) * BMI055_ACCEL_4G_SEN;
        last_a[1] = (float)(ay >> 4) * BMI055_ACCEL_4G_SEN;
        last_a[2] = (float)(az >> 4) * BMI055_ACCEL_4G_SEN;
    }

    if (flags & BMI055_FLAG_GYRO)
    {
        last_g[0] = (float)gx * BMI055_GYRO_2000_SEN;
        last_g[1] = (float)gy * BMI055_GYRO_2000_SEN;
        last_g[2] = (float)gz * BMI055_GYRO_2000_SEN;
    }

    if (flags & BMI055_FLAG_TEMP)
    {
        /* 0.5 K/LSB, 0x00 = 23 C; cast to int8_t for two's complement */
        last_t = (float)((int8_t)temp_raw) * 0.5f + 23.0f;
    }

    *AX = last_a[0];
    *AY = last_a[1];
    *AZ = last_a[2];
    *GX = last_g[0];
    *GY = last_g[1];
    *GZ = last_g[2];
    *Temperature = last_t;

    if (((flags & BMI055_FLAG_ACCEL) == 0) && ((flags & BMI055_FLAG_GYRO) == 0))
    {
        return BMI055_BOTH_READ_ERROR;
    }
    if ((flags & BMI055_FLAG_ACCEL) == 0)
    {
        return BMI055_ACCEL_READ_ERROR;
    }
    if ((flags & BMI055_FLAG_GYRO) == 0)
    {
        return BMI055_GYRO_READ_ERROR;
    }
    return BMI055_OK;
}

uint8_t BMI055_get_data(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                        int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ, int16_t *Tmp)
{
    uint8_t buf[6] = {0};
    uint8_t flags = 0;

    /* Accel data 0x02-0x07, LSB first */
    if (iic_read_len(BMI055_ACCEL_ADDRESS, 0x02, buf, 6) == 0)
    {
        *AccX = (int16_t)((buf[1] << 8) | buf[0]);
        *AccY = (int16_t)((buf[3] << 8) | buf[2]);
        *AccZ = (int16_t)((buf[5] << 8) | buf[4]);
        flags |= BMI055_FLAG_ACCEL;
    }

    /* Gyro data 0x02-0x07 */
    if (iic_read_len(BMI055_GYRO_ADDRESS, 0x02, buf, 6) == 0)
    {
        *GyroX = (int16_t)((buf[1] << 8) | buf[0]);
        *GyroY = (int16_t)((buf[3] << 8) | buf[2]);
        *GyroZ = (int16_t)((buf[5] << 8) | buf[4]);
        flags |= BMI055_FLAG_GYRO;
    }

    /* Temperature is a single byte at accel 0x08 */
    if (iic_read_len(BMI055_ACCEL_ADDRESS, 0x08, buf, 1) == 0)
    {
        *Tmp = (int16_t)buf[0];
        flags |= BMI055_FLAG_TEMP;
    }

    return flags;
}

void BMI055_init(void)
{
    uint8_t gyro_id;
    uint8_t acc_id;
    uint8_t mismatch = 0;

    iic_write_register(BMI055_ACCEL_ADDRESS, 0x14, 0xB6);  /* Accel soft reset */
    delay_ms(10);

    mismatch |= bmi055_write_check(BMI055_ACCEL_ADDRESS, 0x0F, BMI055_ACCEL_RANGE_4G, 0x0F);
    mismatch |= bmi055_write_check(BMI055_ACCEL_ADDRESS, 0x10, BMI055_ACCEL_BW_1000HZ, 0x1F);
    mismatch |= bmi055_write_check(BMI055_ACCEL_ADDRESS, 0x11, BMI055_ACCEL_NORMAL_MODE, 0xE0);
    delay_ms(1);

    iic_write_register(BMI055_GYRO_ADDRESS, 0x14, 0xB6);  /* Gyro soft reset */
    delay_ms(50);

    mismatch |= bmi055_write_check(BMI055_GYRO_ADDRESS, 0x0F, BMI055_GYRO_RANGE_2000, 0x07);
    mismatch |= bmi055_write_check(BMI055_GYRO_ADDRESS, 0x10, BMI055_GYRO_BW_1000HZ, 0x07);
    mismatch |= bmi055_write_check(BMI055_GYRO_ADDRESS, 0x11, BMI055_GYRO_NORMAL_MODE, 0xE0);
    delay_ms(100);

    gyro_id = iic_read_register(BMI055_GYRO_ADDRESS, BMI055_WHO_AM_I);
    acc_id  = iic_read_register(BMI055_ACCEL_ADDRESS, BMI055_WHO_AM_I);
    printf("BMI055 gyro=0x%02X accel=0x%02X\r\n", gyro_id, acc_id);

    if ((gyro_id != BMI055_GYRO_CHIP_ID) || (acc_id != BMI055_ACCEL_CHIP_ID))
    {
        printf("BMI055 WHO_AM_I unexpected (expect gyro=0x0F accel=0xFA)\r\n");
    }
    if (mismatch)
    {
        printf("BMI055 config register verify failed\r\n");
    }
}
