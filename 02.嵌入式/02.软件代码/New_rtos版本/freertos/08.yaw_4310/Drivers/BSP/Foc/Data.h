#ifndef DATA_H
#define DATA_H

#include "Pid.h"
#include "frame13.h"

#define PI 3.1415926f
#define _PI 3.14159265359f
#define _3PI_2 4.71238898038f
#define _PI_3 1.0471975512f
#define _PI_2 1.57079632679f
#define _2PI 6.28318530718f
#define	_1_SQRT3 0.57735026919f
#define _2_SQRT3 1.1547005383f
#define _SQRT3 1.73205080757f

extern float yaw_num;
extern float joy_num;
extern float yaw_pid_num;

extern uint8_t uart_recv[FRAME13_LEN];
extern uint8_t send_angal[FRAME13_LEN];

extern float sensor_direction;
extern float pole_pairs;
extern float voltage_power_supply ;
extern float zero_electric_angle;

extern uint16_t adc_result[2];
extern PID PID_Pos;
extern PID PID_Vel;
extern PID PID_Cur;

extern float gain_a,gain_c;
extern float offset_ia,offset_ic;
extern float volts_to_amps_ratio;
extern float shunt_resistor ;
extern float amp_gain;
extern float current_a,current_c;

#define COMMS_TIMEOUT_US  100000u
extern uint32_t last_frame_us;

#define YAW_CMD_CENTER   0.5f
#define YAW_CMD_SCALE    1.0f

#define YAW_MAX_LEAD_RAD         0.15f

#define YAW_DCMD_MAX_RAD         0.05f

extern float last_iq;

uint32_t overcurrent_trip_count(void);

uint32_t reply_skip_count(void);

void closedloop_bench_test(void);

void float_to_two_uint8_signed(float num, uint8_t* high_byte, uint8_t* low_byte);
float two_uint8_to_float_signed(uint8_t high_byte, uint8_t low_byte);
uint8_t frame_sync(uint8_t byte, uint8_t *frame_out);
void data_change(void);
void data_print(void);
void data_init(void);
#endif
