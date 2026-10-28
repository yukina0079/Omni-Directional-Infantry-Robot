#include "bmi055.h"
#include "usart.h"
#include "delay.h"
#include "iic.h"
#include "math.h"

#define BMI055_ACCEL_2G_SEN  0.0009765625f
#define BMI055_ACCEL_4G_SEN  0.001953125f
#define BMI055_ACCEL_8G_SEN  0.00390625f
#define BMI055_ACCEL_16G_SEN 0.0078125f

#define BMI055_GYRO_2000_SEN 0.06103515625f
#define BMI055_GYRO_1000_SEN 0.030517578125f
#define BMI055_GYRO_500_SEN  0.0152587890625f
#define BMI055_GYRO_250_SEN  0.00762939453125f
#define BMI055_GYRO_125_SEN  0.003814697265625f

void BMI055_update(float *AX,float *AY,float *AZ,float *GX,float *GY,float *GZ,float *Temperature)
{
    int16_t ax = 0, ay = 0, az = 0;
    int16_t gx = 0, gy = 0, gz = 0;
    int16_t temp_raw = 0;

    BMI055_get_data(&ax, &ay, &az, &gx, &gy, &gz, &temp_raw);

    *AX = (float)(ax >> 4) * BMI055_ACCEL_4G_SEN;
    *AY = (float)(ay >> 4) * BMI055_ACCEL_4G_SEN;
    *AZ = (float)(az >> 4) * BMI055_ACCEL_4G_SEN;

    *GX = (float)gx * BMI055_GYRO_2000_SEN;
    *GY = (float)gy * BMI055_GYRO_2000_SEN;
    *GZ = (float)gz * BMI055_GYRO_2000_SEN;

    *Temperature = (float)((int8_t)temp_raw) * 0.5f + 23.0f;
}

void BMI055_get_data(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                      int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ, int16_t *Tmp)
{
    uint8_t buf[6] = {0};

    if (iic_read_len(BMI055_ACCEL_ADDRESS, 0x02, buf, 6) == 0)
    {
        *AccX = (int16_t)((buf[1] << 8) | buf[0]);
        *AccY = (int16_t)((buf[3] << 8) | buf[2]);
        *AccZ = (int16_t)((buf[5] << 8) | buf[4]);
    }

    if (iic_read_len(BMI055_GYRO_ADDRESS, 0x02, buf, 6) == 0)
    {
        *GyroX = (int16_t)((buf[1] << 8) | buf[0]);
        *GyroY = (int16_t)((buf[3] << 8) | buf[2]);
        *GyroZ = (int16_t)((buf[5] << 8) | buf[4]);
    }

    if (iic_read_len(BMI055_ACCEL_ADDRESS, 0x08, buf, 1) == 0)
    {
        *Tmp = (int16_t)buf[0];
    }
}

void BMI055_init(void)
{

    iic_write_register(BMI055_ACCEL_ADDRESS, 0x7E, 0xB6);
    delay_ms(10);

    iic_write_register(BMI055_ACCEL_ADDRESS, 0x0F, 0x05);
    iic_write_register(BMI055_ACCEL_ADDRESS, 0x10, 0x0F);
    iic_write_register(BMI055_ACCEL_ADDRESS, 0x11, 0x00);
    delay_ms(1);

    iic_write_register(BMI055_GYRO_ADDRESS, 0x14, 0xB6);
    delay_ms(50);

    iic_write_register(BMI055_GYRO_ADDRESS, 0x0F, 0x00);

    iic_write_register(BMI055_GYRO_ADDRESS, 0x10, 0x01);
    iic_write_register(BMI055_GYRO_ADDRESS, 0x11, 0x00);
    delay_ms(10);

}
