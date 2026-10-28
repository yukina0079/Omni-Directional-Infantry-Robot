#include "Data.h"
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "sys.h"
#include "uart1.h"
#include "delay.h"
#include "systime.h"
#include "LED.h"
#include "iic.h"
#include "key.h"
#include "as5600.h"
#include "FOC.h"
#include "pwm.h"
#include "Lowpass.h"
#include "Motor.h"
#include "Pid.h"
#include "adc.h"

uint8_t uart_recv[FRAME13_LEN] = {0};
uint8_t send_angal[FRAME13_LEN] = {0x55,0,0,0,0,0,0,0,0,0,0,0,0xff};

float yaw_num = 0.0f;
float joy_num = 0.0f;
float yaw_pid_num = 0.0f;

uint32_t last_frame_us = 0;

float sensor_direction = -1;
float pole_pairs = 14;
float voltage_power_supply = 11.7f;
float zero_electric_angle = 0;

PID PID_Pos;
PID PID_Vel;
PID PID_Cur;

uint16_t adc_result[2] = {0};
float gain_a, gain_c;
float offset_ia, offset_ic;
float volts_to_amps_ratio;

float shunt_resistor = 0.01f;
float amp_gain = 50.0f;
float current_a, current_c;

static uint8_t s_motor_enabled = 0;
static uint8_t s_pos_locked = 0;
static float   s_cmd = 0.0f;
static float   s_yaw_prev = 0.0f;

static void motor_set_enabled(uint8_t enable)
{
	if (enable == s_motor_enabled) {
		return;
	}

	if (enable) {
		Motor_start();
	} else {
		Motor_stop();
	}
	s_motor_enabled = enable;
}

#define OC_LIMIT_A     1.20f
#define OC_TRIP_US     200000u
#define OC_COOLDOWN_US 2000000u

static uint32_t s_oc_since_us  = 0;
static uint32_t s_oc_trip_us   = 0;
static uint8_t  s_oc_active    = 0;
static uint8_t  s_oc_latched   = 0;
static uint32_t s_oc_trip_count = 0;

float last_iq = 0.0f;

#define MONITOR_MAGIC   0x59415731u

typedef struct {
    uint32_t magic;
    uint32_t seq;
    uint32_t loop_us;
    uint32_t frames;
    uint32_t oc_trips;
    uint32_t flags;

    uint32_t reply_skips;
    float    cmd_rad;
    float    angle_rad;
    float    err_rad;
    float    velocity;
    float    iq_amps;
    float    yaw_pid_num;
} yaw_monitor_t;

volatile yaw_monitor_t g_yaw_monitor = { MONITOR_MAGIC, 0, 0, 0, 0, 0, 0,
                                         0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

static uint32_t s_frame_count = 0;

static uint32_t s_reply_skipped = 0;

static void reply_publish(float angle_rad)
{
	uint8_t high, low;
	uint32_t next_idx;

	float_to_two_uint8_signed(angle_rad, &high, &low);

	next_idx = sizeof(send_angal) - DMA1_Channel7->CNDTR;

	if (next_idx > 2u && next_idx <= sizeof(send_angal)) {
		send_angal[1] = high;
		send_angal[2] = low;
	} else {
		s_reply_skipped++;
	}
}

uint32_t reply_skip_count(void)
{
	return s_reply_skipped;
}

static float normalize_angle_rad(float angle)
{
	angle = fmodf(angle, _2PI);
	if (angle > _PI) {
		angle -= _2PI;
	} else if (angle < -_PI) {
		angle += _2PI;
	}
	return angle;
}

static void current_sample(void)
{
	last_iq = Get_Current();
}

static uint8_t overcurrent_ok(void)
{
	float mag = (last_iq < 0.0f) ? -last_iq : last_iq;
	uint32_t now = micros();

	if (s_oc_latched) {
		if (micros_since(s_oc_trip_us) < OC_COOLDOWN_US) {
			return 0;
		}

		s_oc_latched = 0;
		s_oc_active  = 0;
	}

	if (mag > OC_LIMIT_A) {
		if (!s_oc_active) {
			s_oc_active   = 1;
			s_oc_since_us = now;
		} else if (micros_since(s_oc_since_us) > OC_TRIP_US) {
			s_oc_latched = 1;
			s_oc_trip_us = now;
			s_oc_trip_count++;
			return 0;
		}
	} else {
		s_oc_active = 0;
	}

	return 1;
}

uint32_t overcurrent_trip_count(void)
{
	return s_oc_trip_count;
}

static void monitor_update(uint8_t energised, float cmd, float err)
{
	static uint32_t s_last_loop_us = 0;
	uint32_t flags = 0;

	if (energised)                                       { flags |= 0x01u; }
	if (encoder_fault)                                   { flags |= 0x02u; }
	if (micros_since(last_frame_us) > COMMS_TIMEOUT_US)  { flags |= 0x04u; }
	if (s_oc_latched)                                    { flags |= 0x08u; }

	g_yaw_monitor.loop_us     = micros_since(s_last_loop_us);
	s_last_loop_us            = micros();

	g_yaw_monitor.seq++;
	g_yaw_monitor.frames      = s_frame_count;
	g_yaw_monitor.oc_trips    = s_oc_trip_count;
	g_yaw_monitor.flags       = flags;
	g_yaw_monitor.reply_skips = s_reply_skipped;
	g_yaw_monitor.cmd_rad     = cmd;
	g_yaw_monitor.angle_rad   = sensor_direction * Get_Angel();
	g_yaw_monitor.err_rad     = err;
	g_yaw_monitor.velocity    = sensor_direction * Get_Velocity();
	g_yaw_monitor.iq_amps     = last_iq;
	g_yaw_monitor.yaw_pid_num = yaw_pid_num;
}

void data_change(void)
{
	uint8_t sensor_ok;

	uart1_poll();

	joy_num     = two_uint8_to_float_signed(uart_recv[6],  uart_recv[7]);
	yaw_num     = two_uint8_to_float_signed(uart_recv[F13_ATT_H], uart_recv[F13_ATT_L]);
	yaw_pid_num = two_uint8_to_float_signed(uart_recv[F13_CMD_H], uart_recv[F13_CMD_L]);

	sensor_ok = as5600_update();

	current_sample();

	reply_publish(normalize_angle_rad(sensor_direction * Get_Angel_Notrack()));

	if (!sensor_ok || encoder_fault) {

		s_pos_locked = 0;
		motor_set_enabled(0);
		monitor_update(0, 0.0f, 0.0f);
		return;
	}

	if (micros_since(last_frame_us) > COMMS_TIMEOUT_US) {

		s_pos_locked = 0;
		motor_set_enabled(0);
		monitor_update(0, 0.0f, 0.0f);
		return;
	}

	if (!overcurrent_ok()) {

		s_pos_locked = 0;
		motor_set_enabled(0);
		monitor_update(0, 0.0f, 0.0f);
		return;
	}

	{
		float ang;

		ang = sensor_direction * Get_Angel();

		if (!s_pos_locked) {
			s_cmd = ang;
			s_yaw_prev = yaw_pid_num * YAW_CMD_SCALE;
			s_pos_locked = 1;
		} else {
			float dcmd = (yaw_pid_num * YAW_CMD_SCALE) - s_yaw_prev;
			s_yaw_prev = yaw_pid_num * YAW_CMD_SCALE;

			if (dcmd > YAW_DCMD_MAX_RAD) {
				dcmd = YAW_DCMD_MAX_RAD;
			} else if (dcmd < -YAW_DCMD_MAX_RAD) {
				dcmd = -YAW_DCMD_MAX_RAD;
			}
			s_cmd += dcmd;

			if (dcmd > 0.0f) {
				if (s_cmd > (ang + YAW_MAX_LEAD_RAD)) {
					s_cmd = ang + YAW_MAX_LEAD_RAD;
				}
			} else if (dcmd < 0.0f) {
				if (s_cmd < (ang - YAW_MAX_LEAD_RAD)) {
					s_cmd = ang - YAW_MAX_LEAD_RAD;
				}
			}
		}

		motor_set_enabled(1);
		PositionCloseloop(s_cmd);
		monitor_update(1, s_cmd, position_error_rad(s_cmd));
	}
}

void data_print(void)
{
	printf("%.2f\r\n", as5600_read_angal());
}

#define BENCH_STEP_RAD    0.30f
#define BENCH_HOLD_MS     3500u
#define BENCH_PRINT_MS    250u

void closedloop_bench_test(void)
{

	static const float steps[4] = { 0.0f, +BENCH_STEP_RAD, -BENCH_STEP_RAD, 0.0f };

	float origin, target, err, peak_iq, peak_err;
	uint32_t phase, t_phase, t_print;
	uint8_t  aborted = 0;

	printf("\r\n=== closed-loop bench test ===\r\n");
	printf("sensor_direction = %+.0f, pole_pairs = %.0f, zero_el = %.4f\r\n",
	       sensor_direction, pole_pairs, zero_electric_angle);
	printf("PID_Pos limit = %.2f V, overcurrent trip = %.2f A\r\n",
	       PID_Pos.limit, OC_LIMIT_A);

	if (!as5600_update()) {
		printf("ABORT: encoder not responding.\r\n");
		return;
	}
	origin = sensor_direction * Get_Angel();

	printf("\r\n  t(ms)  target   actual    err     Iq(mA)\r\n");

	motor_set_enabled(1);

	for (phase = 0; phase < 4u && !aborted; phase++) {
		target  = origin + steps[phase];
		t_phase = micros();
		t_print = micros();
		peak_iq  = 0.0f;
		peak_err = 0.0f;

		while (micros_since(t_phase) < BENCH_HOLD_MS * 1000u) {
			float iq_mag;

			if (!as5600_update()) {
				printf("  ABORT: encoder dropped out.\r\n");
				aborted = 1;
				break;
			}

			current_sample();

			if (!overcurrent_ok()) {
				printf("  ABORT: overcurrent latch tripped (|Iq| > %.2f A).\r\n",
				       OC_LIMIT_A);
				aborted = 1;
				break;
			}

			PositionCloseloop(target);
			monitor_update(1, target, position_error_rad(target));

			err = position_error_rad(target);
			if (err < 0.0f) { err = -err; }
			if (err > peak_err) { peak_err = err; }

			iq_mag = (last_iq < 0.0f) ? -last_iq : last_iq;
			if (iq_mag > peak_iq) { peak_iq = iq_mag; }

			if (err > 2.0f) {
				printf("  ABORT: error %.2f rad and growing -- RUNAWAY.\r\n", err);
				printf("  sensor_direction is probably wrong. Re-run the\r\n");
				printf("  direction test (COMMISSION_DIRECTION 1).\r\n");
				aborted = 1;
				break;
			}

			if (micros_since(t_print) >= BENCH_PRINT_MS * 1000u) {
				t_print = micros();
				printf("  %5u  %+.3f  %+.3f  %+.3f   %6.0f\r\n",
				       (unsigned)(micros_since(t_phase) / 1000u),
				       target,
				       sensor_direction * Get_Angel(),
				       target - sensor_direction * Get_Angel(),
				       last_iq * 1000.0f);
			}
		}

		if (!aborted) {
			printf("  -- step %u done: peak err %.3f rad, peak |Iq| %.0f mA\r\n",
			       (unsigned)phase, peak_err, peak_iq * 1000.0f);
		}
	}

	motor_set_enabled(0);
	printf("=== bench test complete (overcurrent trips: %u) ===\r\n",
	       (unsigned)overcurrent_trip_count());
}

void data_init(void)
{
}

void float_to_two_uint8_signed(float num, uint8_t* high_byte, uint8_t* low_byte)
{

    if(num > 327.67f) num = 327.67f;
    if(num < -327.68f) num = -327.68f;

    int16_t scaled = (int16_t)(roundf(num * 100.0f));

    *high_byte = (scaled >> 8) & 0xFF;
    *low_byte = scaled & 0xFF;
}

float two_uint8_to_float_signed(uint8_t high_byte, uint8_t low_byte)
{

    int16_t scaled = (int16_t)((high_byte << 8) | low_byte);

    return (float)scaled / 100.0f;
}

uint8_t frame_sync(uint8_t byte, uint8_t *frame_out)
{
    static uint8_t ring[FRAME13_LEN];
    static uint8_t w_idx = 0;
    static uint8_t count = 0;

    ring[w_idx] = byte;
    w_idx = (w_idx + 1) % FRAME13_LEN;
    if (count < FRAME13_LEN) count++;

    if (count >= FRAME13_LEN) {
        uint8_t start = w_idx;
        if (ring[start] == FRAME13_HEAD && ring[(start + F13_TAIL_IDX) % FRAME13_LEN] == FRAME13_TAIL) {
            for (uint8_t i = 0; i < FRAME13_LEN; i++) {
                frame_out[i] = ring[(start + i) % FRAME13_LEN];
            }
            count = 0;
            return 1;
        }
    }
    return 0;
}
