#ifndef __DATA_H
#define __DATA_H
#include "sys.h"
#include "pid.h"

#define PI          3.1415926f

extern uint8_t B_rx_A_buf[32];
extern uint8_t Uart_sand_byte[20];
extern uint8_t Uart_recv_byte[20];

extern float INS_angle_filtered[3];
extern float angle_lpf[3];
extern float last_angle[3];
extern float angle_offset[3];
extern float INS_continuous_angle[3];

extern float gyro[3];
extern float accel[3];
extern float mag[3];
extern float temp[2];
extern float INS_quat[4];
extern float INS_angle[3];

extern pid_type_def pitch_pos_pid;
extern float pitch_position[3];
extern volatile uint32_t nrf_last_ms;

void nrf_mark_rx(void);
void float_to_two_uint8_signed(float num, uint8_t* high_byte, uint8_t* low_byte);
float two_uint8_to_float_signed(uint8_t high_byte, uint8_t low_byte);
void data_change(void);
void pid_calculate(void);
void data_print(void);
uint8_t frame_sync(uint8_t byte, uint8_t *frame_out);
float process_continuous_angle(uint8_t axis, float curr);
float low_pass_filter(float input, float *prev_output, float dt);
double normalizeAngleRad(double angle);
void data_init(void);
void uart_sand(void);
#endif

