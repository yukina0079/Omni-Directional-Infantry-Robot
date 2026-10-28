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

float pitch_num = 0.0f;
float joy_num = 0.0f;
float pitch_pid_num = 0.0f;

uint32_t last_frame_us = 0;

float sensor_direction = -1.0f;
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
static float   s_ly_prev = 0.0f;
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

#define MONITOR_MAGIC   0x50495431u

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
    float    pitch_pid_num;
    float    angle_abs_rad;
    float    cal_min_rad;
    float    cal_max_rad;
    uint32_t raw_count;
} pitch_monitor_t;

volatile pitch_monitor_t g_pitch_monitor = { MONITOR_MAGIC };

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
	static uint8_t  s_cal_inited   = 0;
	uint32_t flags = 0;
	float angle;

	if (energised)                                       { flags |= 0x01u; }
	if (encoder_fault)                                   { flags |= 0x02u; }
	if (micros_since(last_frame_us) > COMMS_TIMEOUT_US)  { flags |= 0x04u; }
	if (s_oc_latched)                                    { flags |= 0x08u; }

	angle = sensor_direction * Get_Angel();

	if (!s_cal_inited) {
		g_pitch_monitor.cal_min_rad = angle;
		g_pitch_monitor.cal_max_rad = angle;
		s_cal_inited = 1;
	} else {
		if (angle < g_pitch_monitor.cal_min_rad) {
			g_pitch_monitor.cal_min_rad = angle;
		}
		if (angle > g_pitch_monitor.cal_max_rad) {
			g_pitch_monitor.cal_max_rad = angle;
		}
	}

	g_pitch_monitor.loop_us     = micros_since(s_last_loop_us);
	s_last_loop_us            = micros();

	g_pitch_monitor.seq++;
	g_pitch_monitor.frames      = s_frame_count;
	g_pitch_monitor.oc_trips    = s_oc_trip_count;
	g_pitch_monitor.flags       = flags;
	g_pitch_monitor.reply_skips = s_reply_skipped;
	g_pitch_monitor.cmd_rad     = cmd;
	g_pitch_monitor.angle_rad   = angle;
	g_pitch_monitor.err_rad     = err;
	g_pitch_monitor.velocity    = sensor_direction * Get_Velocity();
	g_pitch_monitor.iq_amps     = last_iq;
	g_pitch_monitor.pitch_pid_num = pitch_pid_num;
	g_pitch_monitor.angle_abs_rad = Get_Angel_Notrack();
	g_pitch_monitor.raw_count     = (uint32_t)as5600_read_angal();
}

static float clamp_pitch_cmd(float desired)
{
	#if !PITCH_SOFTWARE_LIMIT_ENABLE
	return desired;
	#else
	float now_ctrl = sensor_direction * Get_Angel();
	float now_nt = Get_Angel_Notrack();

	float elev_now = now_nt - PITCH_ZERO_RAD;
	while (elev_now > _PI)  elev_now -= _2PI;
	while (elev_now < -_PI) elev_now += _2PI;

	float elev_pred = elev_now + sensor_direction * (desired - now_ctrl);

	float max_safe_elev = PITCH_MAX_ELEV_RAD - PITCH_LIMIT_INSET_RAD;
	float min_safe_elev = PITCH_MIN_ELEV_RAD + PITCH_LIMIT_INSET_RAD;

	if (elev_pred > max_safe_elev) {
		elev_pred = max_safe_elev;
	} else if (elev_pred < min_safe_elev) {
		elev_pred = min_safe_elev;
	}

	return now_ctrl + sensor_direction * (elev_pred - elev_now);
	#endif
}

void data_change(void)
{
	uint8_t sensor_ok;

	uart1_poll();

	joy_num     = two_uint8_to_float_signed(uart_recv[6],  uart_recv[7]);
	pitch_num     = two_uint8_to_float_signed(uart_recv[F13_ATT_H], uart_recv[F13_ATT_L]);

	pitch_pid_num = two_uint8_to_float_signed(uart_recv[F13_CMD_H], uart_recv[F13_CMD_L]);

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

		motor_set_enabled(0);
		monitor_update(0, 0.0f, 0.0f);
		return;
	}

	{
		float ang;
		float err;

		ang = sensor_direction * Get_Angel();

		if (!s_pos_locked) {
			s_cmd = ang;
			s_ly_prev = pitch_pid_num * PITCH_CMD_SCALE;
			s_pos_locked = 1;
		} else {
			float dly = (pitch_pid_num * PITCH_CMD_SCALE) - s_ly_prev;
			s_ly_prev = pitch_pid_num * PITCH_CMD_SCALE;
			s_cmd += dly;

			if (dly > 0.0f) {
				if (s_cmd > (ang + PITCH_MAX_LEAD_RAD)) {
					s_cmd = ang + PITCH_MAX_LEAD_RAD;
				}
			} else if (dly < 0.0f) {
				if (s_cmd < (ang - PITCH_MAX_LEAD_RAD)) {
					s_cmd = ang - PITCH_MAX_LEAD_RAD;
				}
			}
		}

		s_cmd = clamp_pitch_cmd(s_cmd);
		err = position_error_rad(s_cmd);

#if PITCH_CLOSED_LOOP_ENABLE
		motor_set_enabled(1);
		PositionCloseloop(s_cmd);
		monitor_update(1, s_cmd, err);
#else
		motor_set_enabled(0);
		monitor_update(0, s_cmd, err);
#endif
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

#if 0

#define PITCH_SWEEP_RATE_RAD_S     0.30f
#define PITCH_SWEEP_HOLD_US        250000u
#define PITCH_SWEEP_ARRIVE_RAD     0.06f
#define PITCH_SWEEP_SEG_TIMEOUT_US 5000000u
#define PITCH_SWEEP_WRONG_WAY_RAD  0.10f

#define PITCH_SWEEP_MIN_DRIVE_V         4.00f
#define PITCH_SWEEP_MIN_DRIVE_ERR_RAD   0.035f
#define PITCH_SWEEP_NO_MOTION_CMD_RAD   0.060f
#define PITCH_SWEEP_NO_MOTION_RAD       0.020f
#define PITCH_SWEEP_NO_MOTION_US        1500000u

#define PITCH_SWEEP_INSET_RAD      0.087f
#define PITCH_SWEEP_MIN  (PITCH_ZERO_RAD + PITCH_MIN_OFF_RAD + PITCH_SWEEP_INSET_RAD)
#define PITCH_SWEEP_MAX  (PITCH_ZERO_RAD + PITCH_MAX_OFF_RAD - PITCH_SWEEP_INSET_RAD)

static uint8_t sweep_move_to(float target)
{
	uint32_t t_seg  = micros();
	uint32_t t_prev = micros();
	uint32_t t_hold = 0;
	uint8_t  holding = 0;
	float    cmd;
	float    start_angle;
	float    expect_sign;

	s_selftest_reason = SELFTEST_REASON_NONE;
	s_selftest_target_rad = target;
	s_selftest_uq_volts = 0.0f;
	if (!as5600_update()) {
		s_selftest_reason = SELFTEST_REASON_ENCODER;
		return 0;
	}

	start_angle = sensor_direction * Get_Angel();
	s_selftest_start_rad = start_angle;
	cmd = start_angle;
	expect_sign = (target >= start_angle) ? 1.0f : -1.0f;

	PID_Reset(&PID_Pos);

	printf("  -> %.3f rad (now %.3f)\r\n", target, start_angle);

	for (;;) {
		float dt;
		float err;
		float angle;
		float moved;
		float max_step;
		float d;
		float iq_mag;
		uint32_t now = micros();

		dt = (float)micros_since(t_prev) * 1.0e-6f;
		t_prev = now;
		if (dt < 0.00005f) {
			dt = 0.00005f;
		}
		if (dt > 0.005f) {
			dt = 0.005f;
		}

		if (!as5600_update()) {
			printf("  ABORT: encoder dropped out.\r\n");
			s_selftest_reason = SELFTEST_REASON_ENCODER;
			return 0;
		}
		current_sample();
		iq_mag = (last_iq < 0.0f) ? -last_iq : last_iq;
		if (iq_mag > s_selftest_peak_iq_amps) {
			s_selftest_peak_iq_amps = iq_mag;
		}
		if (!overcurrent_ok()) {
			printf("  ABORT: overcurrent.\r\n");
			s_selftest_reason = SELFTEST_REASON_OVERCURRENT;
			return 0;
		}

		max_step = PITCH_SWEEP_RATE_RAD_S * dt;
		d = target - cmd;
		if (d > max_step) {
			cmd += max_step;
		} else if (d < -max_step) {
			cmd -= max_step;
		} else {
			cmd = target;
		}

		motor_set_enabled(1);
		s_selftest_uq_volts = PositionCloseloopMinDrive(cmd,
		                                               PITCH_SWEEP_MIN_DRIVE_V,
		                                               PITCH_SWEEP_MIN_DRIVE_ERR_RAD);
		err = position_error_rad(cmd);
		monitor_update(1, cmd, err);

		if ((err > PITCH_RUNAWAY_RAD) || (err < -PITCH_RUNAWAY_RAD)) {
			printf("  ABORT: runaway err=%.3f rad.\r\n", err);
			s_selftest_reason = SELFTEST_REASON_RUNAWAY;
			return 0;
		}

		angle = sensor_direction * Get_Angel();
		moved = angle - start_angle;
		if (((cmd - start_angle) * expect_sign) > 0.12f) {
			if ((moved * expect_sign) < -PITCH_SWEEP_WRONG_WAY_RAD) {
				printf("  ABORT: moved the wrong way (sensor_direction?).\r\n");
				s_selftest_reason = SELFTEST_REASON_WRONG_WAY;
				return 0;
			}
		}

		if ((fabsf(cmd - start_angle) >= PITCH_SWEEP_NO_MOTION_CMD_RAD) &&
		    (micros_since(t_seg) >= PITCH_SWEEP_NO_MOTION_US) &&
		    (fabsf(moved) < PITCH_SWEEP_NO_MOTION_RAD)) {
			printf("  ABORT: no motion after %u ms (cmd %.3f, moved %.3f).\r\n",
			       (unsigned)(micros_since(t_seg) / 1000u), cmd, moved);
			s_selftest_reason = SELFTEST_REASON_NO_MOTION;
			return 0;
		}

		if ((cmd == target) && (fabsf(err) < PITCH_SWEEP_ARRIVE_RAD)) {
			if (!holding) {
				holding = 1;
				t_hold = now;
			} else if (micros_since(t_hold) >= PITCH_SWEEP_HOLD_US) {
				return 1;
			}
		} else {
			holding = 0;
		}

		if (micros_since(t_seg) > PITCH_SWEEP_SEG_TIMEOUT_US) {
			printf("  ABORT: timeout target=%.3f angle=%.3f err=%.3f\r\n",
			       target, angle, err);
			s_selftest_reason = SELFTEST_REASON_TIMEOUT;
			return 0;
		}
	}
}

void pitch_travel_selftest(void)
{
	float start_angle;
	s_selftest_reason = SELFTEST_REASON_NONE;
	s_selftest_target_rad = 0.0f;
	s_selftest_start_rad = 0.0f;
	s_selftest_uq_volts = 0.0f;
	s_selftest_peak_iq_amps = 0.0f;

	printf("\r\n=== pitch travel self-test ===\r\n");
	printf("  sweep min=%.3f  zero=%.3f  max=%.3f\r\n",
	       PITCH_SWEEP_MIN, PITCH_ZERO_RAD, PITCH_SWEEP_MAX);

	if (!as5600_update() || encoder_fault) {
		printf("  ABORT: encoder not responding.\r\n");
		s_selftest_reason = SELFTEST_REASON_ENCODER;
		s_selftest_ok = 0;
		motor_set_enabled(0);
		return;
	}

	start_angle = sensor_direction * Get_Angel();
	if (((start_angle > PITCH_ZERO_RAD) &&
	     (!sweep_move_to(PITCH_SWEEP_MIN) ||
	      !sweep_move_to(PITCH_SWEEP_MAX) ||
	      !sweep_move_to(PITCH_ZERO_RAD))) ||
	    ((start_angle <= PITCH_ZERO_RAD) &&
	     (!sweep_move_to(PITCH_SWEEP_MAX) ||
	      !sweep_move_to(PITCH_SWEEP_MIN) ||
	      !sweep_move_to(PITCH_ZERO_RAD)))) {
		s_selftest_ok = 0;
		motor_set_enabled(0);
		printf("=== travel self-test FAILED - motor latched off ===\r\n");
		return;
	}

	motor_set_enabled(0);
	s_selftest_ok = 1;
	printf("=== travel self-test OK ===\r\n");
}

uint8_t pitch_selftest_ok(void)
{
	return s_selftest_ok;
}
#endif

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
