#include "imu.h"
#include "math.h"
#include "MahonyAHRS.h"
#include "imu660ra.h"
#include "ist8310.h"
#include "usart.h"
#include "data.h"
#include "pid.h"
#include "delay.h"

extern pid_type_def yaw_gimble_pid;
extern pid_type_def yaw_chassis_pid;
extern float yaw_output[2];

#define IMU_DT_MIN            0.0005f
#define IMU_DT_MAX            0.005f
#define IMU_DT_DEFAULT        0.001f
#define GYRO_CALIB_SAMPLES    400u
#define GYRO_CALIB_DISCARD    80u
#define GYRO_CALIB_MAX_TRY    800u
#define ACCEL_STILL_MIN       0.85f
#define ACCEL_STILL_MAX       1.15f
#define GYRO_STILL_DPS        8.0f
/*
 * Consecutive failed reads before the IMU is declared unhealthy.
 *
 * Each failure now costs IIC_HW_TIMEOUT_FAST (5 ms), so 20 retries would be
 * 100 ms of degraded scheduling -- exactly the yaw board's comms timeout, i.e.
 * long enough to drop the gimbal on the way to noticing the sensor is gone.
 * 5 retries is 25 ms, which leaves margin, and 5 ms without attitude is far
 * more tolerance than a working IMU ever needs.
 */
#define IMU_FAIL_FREEZE_N     5u

float imu_dt = IMU_DT_DEFAULT;
uint8_t imu_healthy = 1U;
volatile imu_telem_t imu_telem;

static float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
static uint32_t dwt_last;
static uint16_t imu_fail_streak = 0;

static void imu_dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    dwt_last = DWT->CYCCNT;
}

static void imu_dwt_reset(void)
{
    dwt_last = DWT->CYCCNT;
}

static float imu_measure_dt(void)
{
    uint32_t now;
    uint32_t delta;
    float dt;

    now = DWT->CYCCNT;
    delta = now - dwt_last;
    dwt_last = now;

    if (SystemCoreClock == 0u)
    {
        return IMU_DT_DEFAULT;
    }

    dt = (float)delta / (float)SystemCoreClock;
    if (dt < IMU_DT_MIN)
    {
        dt = IMU_DT_MIN;
    }
    else if (dt > IMU_DT_MAX)
    {
        dt = IMU_DT_MAX;
    }
    return dt;
}

static uint8_t imu_gyro_calibrate(void)
{
    float sum[3] = {0.0f, 0.0f, 0.0f};
    uint16_t good = 0;
    uint16_t tries;

    printf("IMU gyro calib: keep still\r\n");

    for (tries = 0; tries < GYRO_CALIB_DISCARD; tries++)
    {
        (void)IMU660RA_update(&accel[0], &accel[1], &accel[2],
                              &gyro[0], &gyro[1], &gyro[2], &temp[0]);
        delay_ms(1);
    }

    tries = 0;
    while ((good < GYRO_CALIB_SAMPLES) && (tries < GYRO_CALIB_MAX_TRY))
    {
        float acc_norm;
        float gyr_norm;

        tries++;
        if (IMU660RA_update(&accel[0], &accel[1], &accel[2],
                            &gyro[0], &gyro[1], &gyro[2], &temp[0]) != IMU660RA_OK)
        {
            delay_ms(1);
            continue;
        }

        acc_norm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
        gyr_norm = sqrtf(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);
        if ((acc_norm >= ACCEL_STILL_MIN) && (acc_norm <= ACCEL_STILL_MAX) &&
            (gyr_norm <= GYRO_STILL_DPS))
        {
            sum[0] += gyro[0];
            sum[1] += gyro[1];
            sum[2] += gyro[2];
            good++;
        }
        delay_ms(1);
    }

    if (good < GYRO_CALIB_SAMPLES)
    {
        printf("IMU calib aborted: still %u/%u acc=%.3f g gyro=%.2f\r\n",
               good, (unsigned)GYRO_CALIB_SAMPLES,
               sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]),
               sqrtf(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]));
        gyro_bias[0] = gyro_bias[1] = gyro_bias[2] = 0.0f;
        return 0U;
    }

    gyro_bias[0] = sum[0] / (float)GYRO_CALIB_SAMPLES;
    gyro_bias[1] = sum[1] / (float)GYRO_CALIB_SAMPLES;
    gyro_bias[2] = sum[2] / (float)GYRO_CALIB_SAMPLES;
    printf("IMU gyro_bias: %.4f %.4f %.4f dps\r\n",
           gyro_bias[0], gyro_bias[1], gyro_bias[2]);
    return 1U;
}

void imu_init(void)
{
    imu_dwt_init();
    AHRS_init(INS_quat, gyro, mag);
    if (IMU660RA_is_ready() != 0U)
    {
        if (imu_gyro_calibrate() == 0U)
        {
            printf("IMU running without gyro bias\r\n");
        }
    }
    imu_fail_streak = 0;
    imu_healthy = 1U;
    imu_dwt_reset();
}

uint8_t imu_updata(void)
{
    float gyro_rad[3];

    imu_dt = imu_measure_dt();

    if (IMU660RA_update(&accel[0], &accel[1], &accel[2],
                        &gyro[0], &gyro[1], &gyro[2], &temp[0]) != IMU660RA_OK)
    {
        if (imu_fail_streak < 0xFFFFu)
        {
            imu_fail_streak++;
        }
        if (imu_fail_streak >= IMU_FAIL_FREEZE_N)
        {
            imu_healthy = 0U;
        }
        return 0U;
    }

    imu_fail_streak = 0U;
    imu_healthy = 1U;

    gyro[0] -= gyro_bias[0];
    gyro[1] -= gyro_bias[1];
    gyro[2] -= gyro_bias[2];

    gyro_rad[0] = gyro[0] * (PI / 180.0f);
    gyro_rad[1] = gyro[1] * (PI / 180.0f);
    gyro_rad[2] = gyro[2] * (PI / 180.0f);

    AHRS_update(INS_quat, imu_dt, gyro_rad, accel, mag);
    get_angle(INS_quat, &INS_angle[0], &INS_angle[1], &INS_angle[2]);

    INS_continuous_angle[0] = process_continuous_angle(0, INS_angle[0]);
    INS_continuous_angle[1] = process_continuous_angle(1, INS_angle[1]);
    INS_continuous_angle[2] = process_continuous_angle(2, INS_angle[2]);

    INS_angle_filtered[0] = low_pass_filter(INS_continuous_angle[0], &angle_lpf[0]);
    INS_angle_filtered[1] = low_pass_filter(INS_continuous_angle[1], &angle_lpf[1]);
    INS_angle_filtered[2] = low_pass_filter(INS_continuous_angle[2], &angle_lpf[2]);

    imu_telem.yaw = INS_angle[0];
    imu_telem.pitch = INS_angle[1];
    imu_telem.roll = INS_angle[2];
    imu_telem.ax = accel[0];
    imu_telem.ay = accel[1];
    imu_telem.az = accel[2];
    imu_telem.gx = gyro[0];
    imu_telem.gy = gyro[1];
    imu_telem.gz = gyro[2];
    imu_telem.dt = imu_dt;
    imu_telem.healthy = imu_healthy;
    imu_telem.ready = IMU660RA_is_ready();
    imu_telem.magic = IMU_TELEM_MAGIC;
    return 1U;
}

void imu_fault_reset(void)
{
    PID_clear(&yaw_gimble_pid);
    PID_clear(&yaw_chassis_pid);
    yaw_output[0] = 0.0f;
    yaw_output[1] = 0.0f;
}

void AHRS_init(float quat[4], float accel[3], float mag[3])
{
    (void)accel;
    (void)mag;
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;
}

void AHRS_update(float quat[4], float time, float gyro[3], float accel[3], float mag[3])
{
    (void)mag;
    MahonyAHRSupdateIMU(quat, time, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
}

void get_angle(float q[4], float *yaw, float *pitch, float *roll)
{
    float sinp;

    *yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                  2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);

    sinp = -2.0f * (q[1] * q[3] - q[0] * q[2]);
    if (sinp > 1.0f)
    {
        sinp = 1.0f;
    }
    else if (sinp < -1.0f)
    {
        sinp = -1.0f;
    }
    *pitch = asinf(sinp);

    *roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                   2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
}

uint8_t imu_is_healthy(void)
{
    return imu_healthy;
}

