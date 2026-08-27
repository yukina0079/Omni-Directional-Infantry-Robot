#ifndef __BMI055_H__
#define __BMI055_H__

#include "sys.h"

#define BMI055_ACCEL_ADDRESS 0x18
#define BMI055_GYRO_ADDRESS  0x68
#define BMI055_WHO_AM_I      0x00

#define BMI055_GYRO_CHIP_ID   0x0F
#define BMI055_ACCEL_CHIP_ID  0xFA

#define BMI055_ACCEL_RANGE_4G     0x05
#define BMI055_ACCEL_BW_1000HZ    0x0F
#define BMI055_ACCEL_NORMAL_MODE  0x00
#define BMI055_GYRO_RANGE_2000    0x00
#define BMI055_GYRO_BW_1000HZ     0x02  /* 1000 Hz ODR, 116 Hz filter */
#define BMI055_GYRO_NORMAL_MODE   0x00

#define BMI055_FLAG_ACCEL  (1u << 0)
#define BMI055_FLAG_GYRO   (1u << 1)
#define BMI055_FLAG_TEMP   (1u << 2)

typedef enum
{
    BMI055_OK = 0,
    BMI055_ACCEL_READ_ERROR,
    BMI055_GYRO_READ_ERROR,
    BMI055_BOTH_READ_ERROR
} bmi055_status_t;

void BMI055_init(void);
bmi055_status_t BMI055_update(float *AX, float *AY, float *AZ,
                              float *GX, float *GY, float *GZ,
                              float *Temperature);
uint8_t BMI055_get_data(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                        int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ,
                        int16_t *Tmp);
float calculateTemperature(void);

#endif
