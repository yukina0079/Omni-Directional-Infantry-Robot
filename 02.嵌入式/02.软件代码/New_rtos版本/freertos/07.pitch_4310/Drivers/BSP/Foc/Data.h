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

extern float pitch_num;
extern float joy_num;
extern float pitch_pid_num;

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
 * Encoder position mode. Bytes 14/15 are the remote's integrated ly
 * angle in radians. The first good frame captures the current shaft
 * angle as the session origin, so the barrel is not yanked to a
 * stored horizon. Stick centre holds; stick deflection slews the
 * setpoint. BMI055 is not required -- that is only for world-frame
 * hold once the gimbal MCU rides with the axis.
 *
 * PITCH_ZERO_RAD remains the last measured horizon for bench tests.
 */
#define PITCH_ZERO_RAD            6.2326f
#define PITCH_CMD_SCALE           1.0f
#define PITCH_CLOSED_LOOP_ENABLE  1

/*
 * Calibrated 2026-08-25:
 *   Horizon (0 deg)      : 6.2326 rad (raw=4063)
 *   Upper stop (+24.70 deg): 5.8015 rad (raw=3782, delta = +0.4311 rad)
 *   Lower stop (-26.45 deg): 0.4111 rad (raw=268,  delta = -0.4617 rad)
 *   Total travel span    : 51.15 deg
 */
#define PITCH_MAX_ELEV_RAD        0.4311f
#define PITCH_MIN_ELEV_RAD       -0.4617f
#define PITCH_LIMIT_INSET_RAD     0.0175f

#define PITCH_SOFTWARE_LIMIT_ENABLE  1

/*
 * How far a live stick increment may lead the shaft. Extra integrator
 * travel past this is thrown away so a held stick cannot bank a 20 deg
 * debt and then slam. ~7 deg is enough error for P + friction to
 * break away. Applied ONLY in the direction of the current increment:
 * a two-sided clamp would drag the setpoint after a gravity sag and
 * ratchet the barrel from the upper stop down to the lower one.
 */
#define PITCH_MAX_LEAD_RAD        0.12f

/*
 * Voltage cap / hold torque. 10.7 ohm winding: 6.5 V ≈ 0.61 A stall,
 * enough to hold against gravity without sitting on the SVPWM ceiling.
 */
#define PITCH_PHASE_RESISTANCE_OHM  10.7f
#define PITCH_TORQUE_LIMIT_V        6.5f

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
