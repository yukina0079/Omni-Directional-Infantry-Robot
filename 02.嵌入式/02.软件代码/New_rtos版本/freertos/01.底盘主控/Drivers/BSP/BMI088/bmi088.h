#ifndef __BMI088_H__
#define __BMI088_H__

#include "sys.h"

#define BMI088_ACCEL_ADDRESS 0x18	//加速度计
#define BMI088_GYRO_ADDRESS  0x68	//陀螺仪

#define BMI088_WHO_AM_I      0x00

void BMI088_init(void);
void BMI088_update(float *AX,float *AY,float *AZ,float *GX,float *GY,float *GZ,float *Temperature);
void BMI088_get_angle(void);
void BMI088_get_data(int16_t *AccX, int16_t *AccY, int16_t *AccZ,\
                      int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ, int16_t *Tmp);
float calculateTemperature(void);
#endif

