#ifndef __IMU660RA_H
#define __IMU660RA_H

#include "sys.h"

/* IMU660RA uses the Bosch BMI270-compatible register map. */
#define IMU660RA_I2C_ADDR_LOW       0x68U
#define IMU660RA_I2C_ADDR_HIGH      0x69U
#define IMU660RA_CHIP_ID_REG        0x00U
#define IMU660RA_CHIP_ID            0x24U

#define IMU660RA_ACC_DATA_REG       0x0CU
#define IMU660RA_GYR_DATA_REG       0x12U
#define IMU660RA_PWR_CONF_REG       0x7CU
#define IMU660RA_PWR_CTRL_REG       0x7DU
#define IMU660RA_INIT_CTRL_REG      0x59U
#define IMU660RA_INIT_ADDR_0_REG    0x5BU
#define IMU660RA_INIT_ADDR_1_REG    0x5CU
#define IMU660RA_INIT_DATA_REG      0x5EU
#define IMU660RA_INTERNAL_STATUS_REG 0x21U  /* bits[3:0]==0x01 means config OK */
#define IMU660RA_CMD_REG            0x7EU
#define IMU660RA_SOFT_RESET_CMD     0xB6U
#define IMU660RA_TEMP_DATA_REG      0x22U
#define IMU660RA_ACC_CONF_REG       0x40U
#define IMU660RA_ACC_RANGE_REG      0x41U
#define IMU660RA_GYR_CONF_REG       0x42U
#define IMU660RA_GYR_RANGE_REG      0x43U

/* Seekfree IMU660RA_ACC_SAMPLE / IMU660RA_GYR_SAMPLE. */
#define IMU660RA_ACC_SAMPLE         0x01U   /* +/-4 g, raw / 8192 -> g */
#define IMU660RA_GYR_SAMPLE         0x00U   /* +/-2000 dps, raw / 16.4 -> deg/s */

#define IMU660RA_CONFIG_SIZE        8192U
#define IMU660RA_CONFIG_CHUNK       32U

typedef enum
{
    IMU660RA_OK = 0,
    IMU660RA_ERROR_NOT_FOUND,
    IMU660RA_ERROR_CONFIG,
    IMU660RA_ERROR_DATA
} IMU660RA_Status;

#define IMU660RA_DIAG_MAGIC                 0x494D5531u
#define IMU660RA_DIAG_STAGE_PROBE_HIGH      0x10u
#define IMU660RA_DIAG_STAGE_PROBE_LOW       0x11u
#define IMU660RA_DIAG_STAGE_SOFT_RESET      0x20u
#define IMU660RA_DIAG_STAGE_RESET_CHIP_ID   0x21u
#define IMU660RA_DIAG_STAGE_PWR_CONF        0x30u
#define IMU660RA_DIAG_STAGE_INIT_CTRL_START 0x31u
#define IMU660RA_DIAG_STAGE_INIT_ADDR       0x32u
#define IMU660RA_DIAG_STAGE_INIT_DATA       0x33u
#define IMU660RA_DIAG_STAGE_INIT_CTRL_END   0x34u
#define IMU660RA_DIAG_STAGE_INTERNAL_STATUS 0x35u
#define IMU660RA_DIAG_STAGE_SENSOR_CONFIG   0x40u
#define IMU660RA_DIAG_STAGE_DONE            0x7Fu

#define IMU660RA_PROBE_LOW_FOUND             0x01u
#define IMU660RA_PROBE_HIGH_FOUND            0x02u

typedef struct
{
    uint32_t magic;
    uint32_t i2c_error;
    uint16_t config_offset;
    uint8_t stage;
    uint8_t result;
    uint8_t chip_id;
    uint8_t i2c_addr;
    uint8_t internal_status;
    uint8_t i2c_status;
    uint8_t probe_low_chip_id;
    uint8_t probe_high_chip_id;
    uint8_t probe_found_mask;
} imu660ra_diag_t;

extern volatile imu660ra_diag_t imu660ra_diag;

IMU660RA_Status IMU660RA_update(float *AX, float *AY, float *AZ,
                                float *GX, float *GY, float *GZ,
                                float *Temperature);

IMU660RA_Status IMU660RA_init(void);
IMU660RA_Status IMU660RA_get_data(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                                  int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ,
                                  int16_t *Temperature);
uint8_t IMU660RA_get_chip_id(void);
uint8_t IMU660RA_get_i2c_addr(void);
uint8_t IMU660RA_is_ready(void);
float IMU660RA_acc_transition(int16_t acc_value);
float IMU660RA_gyro_transition(int16_t gyro_value);

#endif
