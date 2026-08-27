#ifndef DATA_H
#define DATA_H

#include "Pid.h"

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

extern uint8_t uart_recv[20];
extern uint8_t send_angal[20];

extern float sensor_direction;//编码器方向
extern float pole_pairs;//电机极对数
extern float voltage_power_supply ;//供电电压
extern float zero_electric_angle;

extern uint16_t adc_result[2];
extern PID PID_Pos;//位置PID
extern PID PID_Vel;//速度PID
extern PID PID_Cur;//电流PID

extern float gain_a,gain_c;//欧姆定律系数
extern float offset_ia,offset_ic;//采集的2路电流
extern float volts_to_amps_ratio;
extern float shunt_resistor ;//采样电阻值
extern float amp_gain;//运放增益
extern float current_a,current_c;//A相和C相电流

/*
 * micros() stamp of the last valid frame from the chassis MCU.
 *
 * Stored in microseconds, not milliseconds, so that the elapsed-time test can
 * be a plain unsigned subtraction. millis() is micros()/1000, which rolls over
 * at a value that is NOT a power of two -- an unsigned difference across that
 * roll would produce a huge bogus interval and trip the timeout once every
 * ~71 minutes. Comparing raw microseconds wraps cleanly.
 */
#define COMMS_TIMEOUT_US  100000u   /* 100 ms */
extern uint32_t last_frame_us;

/*
 * Mapping from the chassis MCU's yaw command (frame bytes 14/15) to this
 * axis's mechanical position setpoint, in radians.
 *
 * Units, traced through the link (01.底盘主控/Drivers/BSP/DATA/data.c):
 *   yaw_output[0] = PID_calc(&yaw_gimble_pid, joy_yaw_num, INS_angle_filtered[0])
 * where yaw_gimble_pid has P = 1.0 and INS_angle_filtered[0] is a continuous
 * yaw angle in RADIANS. With unity proportional gain the PID output is
 * therefore already an angular quantity in radians, which is why the scale here
 * is 1.0 and not a conversion factor. This matches the original firmware's
 * intent, `Position_VelocityCloseloop(0.5f + yaw_pid_num)`.
 *
 * YAW_CMD_CENTER is the mechanical "gimbal centred" angle. It matches
 * gimble_angal = 0.5f on the chassis side, which is the value the chassis's own
 * follow loop drives this axis toward, so the two ends agree on where centre is.
 *
 * There is deliberately NO excursion clamp. This yaw axis rotates a full 360
 * degrees continuously, so there is no mechanical end stop for a clamp to
 * protect, and an earlier +-1.5 rad clamp did real harm: it confined the gimbal
 * to a +-86 degree sector of an axis built to spin freely.
 *
 * What bounds a corrupt frame instead is the velocity limit of the position
 * loop. PID_Pos.limit caps the velocity setpoint at 31.4 rad/s, and the chassis
 * sends a fresh frame every 2 ms, so the furthest a single bad command can
 * carry the shaft before the next good one overrides it is 31.4 * 0.002 = 0.06
 * rad, about 3.6 degrees. A clamp is not needed to bound that; the frame rate
 * and the velocity limit already do, and they do it without giving up travel.
 *
 * The command is an absolute multi-turn angle, and Get_Angel() tracks turns, so
 * the axis follows it across rotation boundaries without special handling.
 */
#define YAW_CMD_CENTER   0.5f
#define YAW_CMD_SCALE    1.0f

/*
 * How far the setpoint may lead the shaft. Extra chassis/remote
 * integrator travel past this is thrown away so a held stick cannot
 * bank a multi-turn debt. No mechanical clamp: this axis is continuous.
 */
#define YAW_MAX_LEAD_RAD         0.15f  /* commanded direction only, ~8.6 deg */

/*
 * One-cycle increment cap. A dropped UART/NRF frame then a catch-up
 * jump used to look like a huge dcmd; the velocity feedforward
 * turned that into a voltage spike (the "sudden shake"). FF is gone;
 * this still rate-limits the setpoint so a glitch cannot yank it.
 * 0.05 rad ≈ full-stick increment over a 2 ms chassis frame.
 */
#define YAW_DCMD_MAX_RAD         0.05f

/* Most recent filtered q-axis current, amps. Written by the supervisor in
 * data_change(); exposed for telemetry and bench printing. */
extern float last_iq;

/* Number of times the overcurrent latch has closed since reset. */
uint32_t overcurrent_trip_count(void);

/*
 * Number of reply-frame updates skipped because the TX DMA was mid-way through
 * the angle field. See reply_publish() in Data.c -- skipping is how P1-9 (torn
 * reply frames) is avoided. A steadily rising count is normal (~15% of cycles);
 * a count that tracks the cycle count 1:1 would mean the window never opens,
 * i.e. the DMA is not running.
 */
uint32_t reply_skip_count(void);

/*
 * Closes the position loop against an internal setpoint so the control chain
 * can be exercised with no chassis MCU attached — data_change()'s comms
 * interlock would otherwise keep the axis de-energised on a bench.
 *
 * Steps the shaft +-0.3 rad and reports settling error and peak current, with
 * a runaway guard that aborts if the error grows instead of shrinking.
 * Barrel off, supply current-limited. Runs ~14 s.
 */
void closedloop_bench_test(void);

void float_to_two_uint8_signed(float num, uint8_t* high_byte, uint8_t* low_byte);
float two_uint8_to_float_signed(uint8_t high_byte, uint8_t low_byte);
uint8_t frame_sync(uint8_t byte, uint8_t *frame_out);
void data_change(void);
void data_print(void);
void data_init(void);
#endif
