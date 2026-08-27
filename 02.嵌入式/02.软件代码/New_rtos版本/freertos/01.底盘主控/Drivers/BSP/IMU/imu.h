#ifndef __IMU_H
#define __IMU_H

#include "sys.h"

extern float imu_dt;
extern uint8_t imu_healthy;

#define IMU_TELEM_MAGIC  0x41545431u

typedef struct
{
    uint32_t magic;
    float yaw;
    float pitch;
    float roll;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float dt;
    uint8_t healthy;
    uint8_t ready;
} imu_telem_t;

extern volatile imu_telem_t imu_telem;

void AHRS_init(float quat[4], float accel[3], float mag[3]);
void AHRS_update(float quat[4], float time, float gyro[3], float accel[3], float mag[3]);
void get_angle(float q[4], float *yaw, float *pitch, float *roll);
uint8_t imu_updata(void);
void imu_fault_reset(void);
void imu_init(void);
uint8_t imu_is_healthy(void);

#endif
