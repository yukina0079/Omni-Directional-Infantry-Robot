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

uint8_t uart_recv[20] = {0};
uint8_t send_angal[20] = {0x55,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xff};

float yaw_num = 0.0f;
float joy_num = 0.0f;
float yaw_pid_num = 0.0f;

/* micros() stamp of the last well-formed frame. See Data.h for why this is
 * microseconds rather than milliseconds. */
uint32_t last_frame_us = 0;

//电机相关

/*
 * Encoder orientation relative to the phase order. MEASURED, not assumed.
 *
 * motor_direction_test() swept the electrical angle 4 electrical revolutions
 * forward (+25.13 el.rad) and watched the shaft:
 *
 *     commanded  +25.13 el.rad
 *     shaft moved  -1.8162 rad      (expected +1.7952 rad if direction were +1)
 *     implied pole_pairs = 13.84    (declared 14, confirmed)
 *
 * The shaft moved the RIGHT amount but the WRONG way, so the encoder counts
 * opposite to the direction the field rotates. This value was previously +1,
 * inherited from the pitch board without ever being verified on this axis.
 *
 * Why it matters: sensor_direction multiplies the mechanical angle inside
 * _electricalAngle(), so with the wrong sign the commutation angle advances
 * backwards. The position loop then pushes the shaft AWAY from the setpoint --
 * positive feedback, and the axis runs away as soon as the loop is closed.
 * Nothing in the static tests (encoder read, current polarity) can catch this,
 * because it only shows up once the rotor moves.
 *
 * Re-run motor_direction_test() (main.c: COMMISSION_DIRECTION 1) after any
 * change to the motor phase wiring or the encoder magnet mounting.
 */
float sensor_direction = -1;     //编码器方向 (measured 2026-08-09)
float pole_pairs = 14;           //电极对数 (confirmed: implied 13.84)
float voltage_power_supply = 11.7f; //供电电压
float zero_electric_angle = 0;

//PID相关
PID PID_Pos;//位置PID
PID PID_Vel;//速度PID
PID PID_Cur;//电流PID

//电流采集相关
uint16_t adc_result[2] = {0};
float gain_a, gain_c;
float offset_ia, offset_ic;      //采集的2路电流 (A相 和 C相)
float volts_to_amps_ratio;

float shunt_resistor = 0.01f;    //采样电阻值 R80/R81 = 10 mOhm
float amp_gain = 50.0f;          //运放增益 INA240A2 = 50 V/V
float current_a, current_c;      //A相和C相电流

/*
	帧格式
	0x55 按键 enl enr lxh lxl lyh lyl
	rxh  rxl  ryh ryl yaw yaw pid pid
	0    0    0   0xff
*/

/*
 * Tracks whether the output stage is currently energised, so the interlocks
 * below only call Motor_stop()/Motor_start() on an actual state change.
 * Without this the loop would hammer HAL_TIM_PWM_Stop() thousands of times a
 * second whenever comms are absent -- which is the normal state at power-up,
 * before the chassis MCU starts talking.
 */
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

/*
 * ---------------------------------------------------------------------------
 * Overcurrent supervisor
 * ---------------------------------------------------------------------------
 *
 * The PID_Pos limit set in main.c is the PRIMARY current limit, and at low speed
 * it is a hard one: with negligible back-EMF the phase current is just Uq/R, so
 * capping Uq at 4.0 V against the measured 5.5 ohm winding caps the stall
 * current near 0.73 A. That is the case that matters thermally, because a gimbal
 * spends most of its life holding position rather than slewing.
 *
 * This supervisor is the backstop for everything that reasoning does not cover:
 * a bus voltage higher than the declared 11.7 V, a winding that is colder (and
 * therefore lower resistance) than when it was measured, or a genuine fault such
 * as a shorted phase. It watches the real current and latches the output stage
 * off if the limit is exceeded continuously.
 *
 * Why "continuously" rather than a single sample: current transients during a
 * fast direction reversal legitimately exceed the steady-state figure for a few
 * hundred microseconds. Tripping on one sample would shut the axis down during
 * normal aggressive moves, so the trip needs to persist for OC_TRIP_US before it
 * counts. A short spike decays the counter instead of latching.
 */
#define OC_LIMIT_A     1.20f      /* amps, q-axis magnitude                */
#define OC_TRIP_US     200000u    /* 200 ms continuously above the limit   */
#define OC_COOLDOWN_US 2000000u   /* stay off 2 s before trying again      */

static uint32_t s_oc_since_us  = 0;    /* when the present overload began  */
static uint32_t s_oc_trip_us   = 0;    /* when the latch closed            */
static uint8_t  s_oc_active    = 0;    /* currently above OC_LIMIT_A       */
static uint8_t  s_oc_latched   = 0;
static uint32_t s_oc_trip_count = 0;

float last_iq = 0.0f;      /* exposed for telemetry / debug printing */

/*
 * ---------------------------------------------------------------------------
 * SWD live-monitor block
 * ---------------------------------------------------------------------------
 *
 * A fixed struct in RAM that a debugger can poll over SWD while the firmware
 * runs, with no halt and no serial traffic.
 *
 * This exists because of a hardware constraint: this board has exactly one
 * usable UART (USART2 on PA2/PA3). Once the chassis MCU is wired to it, that
 * wire carries the 20-byte binary protocol in both directions and printf() can
 * no longer be used -- debug text would corrupt the link, and the link's binary
 * frames would bury the text. So the moment the axis is in its real
 * configuration is exactly the moment the serial console stops being available.
 *
 * SWD is already connected for flashing, costs no pins beyond that, and reading
 * target RAM does not disturb the running CPU. So the debug channel moves here.
 *
 * `magic` lets the reader confirm it is looking at the right address, which
 * matters because the address is resolved from the .map file and would
 * otherwise silently change on every rebuild.
 */
#define MONITOR_MAGIC   0x59415731u   /* "YAW1" */

typedef struct {
    uint32_t magic;
    uint32_t seq;             /* increments every control cycle          */
    uint32_t loop_us;         /* measured control period, microseconds   */
    uint32_t frames;          /* well-formed frames received             */
    uint32_t oc_trips;        /* overcurrent latch closures              */
    uint32_t flags;           /* bit0 energised, bit1 enc fault,
                                 bit2 comms timeout, bit3 oc latched     */
    uint32_t reply_skips;     /* reply updates deferred, see reply_publish */
    float    cmd_rad;         /* position setpoint                       */
    float    angle_rad;       /* measured shaft angle (direction-applied) */
    float    err_rad;         /* cmd - angle                             */
    float    velocity;        /* rad/s                                   */
    float    iq_amps;         /* filtered q-axis current                 */
    float    yaw_pid_num;     /* raw command from the chassis, bytes 14/15 */
} yaw_monitor_t;

/* Not static: the linker must emit a symbol so the .map file can be searched
 * for its address. Volatile so the compiler cannot cache the fields across the
 * update, which would leave the debugger reading stale values. */
volatile yaw_monitor_t g_yaw_monitor = { MONITOR_MAGIC, 0, 0, 0, 0, 0, 0,
                                         0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

/* Frame counter, bumped by frame_sync() on every good frame. */
static uint32_t s_frame_count = 0;

/*
 * ---------------------------------------------------------------------------
 * Reply publication (fixes P1-9)
 * ---------------------------------------------------------------------------
 *
 * send_angal is both the circular TX DMA's source buffer and the buffer the
 * control loop rewrites, with nothing synchronising the two. DMA1_Channel7
 * fetches one byte every 8.7 us at 115200 baud and never pauses, so a write
 * that lands between its fetch of byte 1 and its fetch of byte 2 puts the high
 * byte of one sample next to the low byte of the next -- a value the shaft was
 * never at.
 *
 * That is not theoretical. Measured with fake_chassis.py --glitch-stats,
 * sweeping the axis across 0 rad for 40 s:
 *
 *     decoded 23076 reply frames
 *     TORN FRAMES: 2 of 23076 (0.009%)
 *       t= 24.63s  -0.010 -> -2.560  (jump +2.550)
 *
 * -2.560 rad is 0xFF00: high byte 0xFF from the -0.01 sample, low byte 0x00
 * from the 0.00 sample that replaced it. The zero crossing is the dangerous
 * spot precisely because that is where the high byte flips (0xFFFF -> 0x0000),
 * making the two halves maximally inconsistent. Rare, but the chassis feeds
 * this straight into its follow PID, where one frame reporting a 147 deg error
 * is a torque kick.
 *
 * The fix is to publish only when the DMA controller is not about to read the
 * bytes being written. CNDTR counts down from 20 and reloads, so the index of
 * the next byte it will fetch is (20 - CNDTR). Writing while that index is
 * already past 2 is safe: the two stores complete in nanoseconds, and DMA
 * cannot reach byte 1 again until it wraps, at least 8.7 us away.
 *
 * When the window is closed the update is skipped rather than waited for.
 * Spinning here would stall a 2 kHz control loop for up to 26 us; skipping
 * costs one cycle of staleness (~0.5 ms, during which the shaft moves well
 * under a milliradian) and the next cycle almost certainly succeeds -- the
 * window is open 85% of the time, and the loop and frame periods are not
 * harmonically locked, so consecutive misses are ~2% likely.
 *
 * Note this also protects against a torn *pair*, not just a torn 16-bit field:
 * both bytes are written inside one open window, so a receiver can never see
 * halves from different control cycles.
 */
static uint32_t s_reply_skipped = 0;

static void reply_publish(float angle_rad)
{
	uint8_t high, low;
	uint32_t next_idx;

	float_to_two_uint8_signed(angle_rad, &high, &low);

	/* DMA1_Channel7 is USART2_TX on this part. Reading CNDTR is a single
	 * 32-bit load with no side effects, safe to do at any time. */
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

/*
 * Folds an angle into [-PI, PI]. Mirrors normalizeAngleRad() on the chassis
 * side (01.底盘主控/Drivers/BSP/DATA/data.c) so both ends describe the gimbal's
 * position with the same number rather than one that differs by a whole turn.
 *
 * Note this is NOT correcting a fault in the link. The chassis feeds our reply
 * to two consumers and both already tolerate a full-turn offset: its follow PID
 * uses normalizeAngleRad(gimble_yaw), and gimbal_to_chassis_speed_compute() is
 * a pure rotation matrix built from cosf/sinf, which are 2*PI periodic -- so
 * cosf(-6.29) and cosf(-0.007) are the same number to float precision.
 *
 * The reason to do it here anyway is that this board has no console (see the
 * SWD monitor block above): the reply frame is the only thing about this axis
 * an outside observer can see directly. Sending -5.79 when the shaft is parked
 * at the +0.5 rad centre forces every reader -- a scope, fake_chassis.py, or a
 * person -- to do the fold in their head before the value means anything, and
 * an off-by-one-turn reading is indistinguishable from a genuinely misplaced
 * axis. Sending +0.49 makes "is the gimbal centred?" answerable at a glance.
 */
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

/*
 * Refreshes last_iq. Split out of overcurrent_ok() so the current the monitor
 * reports is sampled on EVERY control cycle, including the ones that de-energise
 * the axis and return before the supervisor ever runs.
 *
 * Without this the monitor keeps displaying whatever was measured on the last
 * cycle the motor was actually driven -- observed on the bench as "-18 mA"
 * printed next to a COMMS_TIMEOUT flag, which reads as current still flowing
 * through a de-energised axis. Every other field in the block is refreshed on
 * all paths for exactly this reason; the current was the one that was not.
 */
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
		/* Cooldown expired: clear the latch and let the loop try again. If the
		 * fault is still present the trip simply happens a second time. */
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

/*
 * Refreshes the SWD monitor block. Called on EVERY exit path from
 * data_change(), including the ones that de-energise the axis.
 *
 * Updating only on the healthy path would be actively misleading: an encoder
 * dropout or comms timeout would freeze the block at its last good values, so a
 * debugger would show a happy, tracking axis while the motor was actually
 * stopped. The flags word is the whole point -- it says WHY the axis is off.
 */
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

	/* --- 1. communications --- */
	uart1_poll();

	joy_num     = two_uint8_to_float_signed(uart_recv[6],  uart_recv[7]);
	yaw_num     = two_uint8_to_float_signed(uart_recv[12], uart_recv[13]);
	yaw_pid_num = two_uint8_to_float_signed(uart_recv[14], uart_recv[15]);

	/*
	 * --- 2. sample the encoder ONCE for this cycle ---
	 * Everything below (position error, velocity, electrical angle) reads the
	 * cache this fills in. Previously each of those triggered its own I2C
	 * transaction.
	 */
	sensor_ok = as5600_update();

	/* Sample the phase current before any interlock can return early, so the
	 * monitor's Iq field is honest on every path. See current_sample(). */
	current_sample();

	/*
	 * Reply to the chassis master: OUR mechanical angle, in the SAME sign
	 * convention the control loop uses, folded to a single turn.
	 *
	 * The sign matters: the loop compares its setpoint against
	 * sensor_direction * Get_Angel(), and with sensor_direction = -1 the raw
	 * encoder count is the negative of the angle the loop (and the chassis's
	 * follow loop) thinks the gimbal is at. The chassis reads our reply as
	 * gimble_yaw and chases gimble_yaw - gimble_angal to zero, so a reply with
	 * the wrong sign makes the whole base chase a constant offset. Measured on
	 * the bench: cmd = +0.5 parked the shaft at direction-applied +0.5 while
	 * the raw count read -0.5 -- the raw value would report a -1.0 rad offset.
	 *
	 * The single-turn fold matters for range: float_to_two_uint8_signed()
	 * saturates at +-327.67, and the continuous multi-turn angle leaves that
	 * range after about half a revolution. Any manual turn of the axis while
	 * powered off (routine during setup) would then pin the reply at 3.27 rad
	 * and the chassis would read a huge fake offset. Get_Angel_Notrack() stays
	 * in [0, 2PI) forever.
	 *
	 * normalize_angle_rad() then centres that on zero, so the transmitted value
	 * reads as a signed offset from straight-ahead the way a human and the
	 * chassis both expect. See the function's comment for why this is about
	 * readability rather than correctness.
	 */
	reply_publish(normalize_angle_rad(sensor_direction * Get_Angel_Notrack()));

	/* --- 3. safety interlocks --- */
	if (!sensor_ok || encoder_fault) {
		/* Encoder is not answering: the cached angle is stale and commutating
		 * from it would run the gimbal away. */
		s_pos_locked = 0;
		motor_set_enabled(0);
		monitor_update(0, 0.0f, 0.0f);
		return;
	}

	if (micros_since(last_frame_us) > COMMS_TIMEOUT_US) {
		/* Chassis MCU has gone quiet. De-energise rather than staying latched
		 * onto the last commanded angle. Also covers the power-up window
		 * before the first frame ever arrives. */
		s_pos_locked = 0;
		motor_set_enabled(0);
		monitor_update(0, 0.0f, 0.0f);
		return;
	}

	/* --- 4. overcurrent supervision --- */
	if (!overcurrent_ok()) {
		/*
		 * Latched on overcurrent. De-energise and wait out the cooldown.
		 *
		 * Drop the position lock, exactly as the encoder and comms paths above
		 * do -- this path is the one that was missing it. The cooldown is 2 s
		 * (OC_COOLDOWN_US), long enough for the shaft to be moved by hand or by
		 * the very bind that caused the overload, so s_cmd is no longer a safe
		 * setpoint to resume from. Clearing s_pos_locked forces the next healthy
		 * cycle through the !s_pos_locked branch below, which recaptures the
		 * current shaft angle so the loop re-engages at ~0 error instead of
		 * snapping the shaft back to the stale pre-trip setpoint (up to a 180 deg
		 * error after wrapping -- a full-torque kick). Motor_start()'s PID_Reset
		 * clears the integrator, but s_cmd is not PID state and would otherwise
		 * survive the trip.
		 */
		s_pos_locked = 0;
		motor_set_enabled(0);
		monitor_update(0, 0.0f, 0.0f);
		return;
	}

	/* --- 5. control ---
	 *
	 * Follow increments of the chassis yaw command (remote lx integrator
	 * when CHASSIS_IMU_ENABLE is 0). First good frame captures the current
	 * shaft angle -- no yank to YAW_CMD_CENTER. Extra increments past
	 * YAW_MAX_LEAD_RAD are discarded so a held stick cannot bank a
	 * multi-turn debt. No mechanical clamp: the axis is continuous.
	 */
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

			/* Lead only in the commanded direction -- never drag
			 * the setpoint toward a disturbed shaft. */
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

/*
 * ---------------------------------------------------------------------------
 * Closed-loop bench test
 * ---------------------------------------------------------------------------
 *
 * Closes the position loop against an internally generated setpoint, so the
 * control chain can be validated with no chassis MCU attached.
 *
 * This test exists because of an ordering problem: data_change() refuses to
 * energise unless a frame arrived within COMMS_TIMEOUT_US, which is exactly
 * right in the field but means the loop can never be exercised on a bench where
 * nothing is transmitting. Rather than weaken the interlock for testing (and
 * risk shipping the weakened version), this drives the same control call behind
 * its own entry point.
 *
 * What it proves, in order of importance:
 *   1. Commutation works    -- the shaft moves toward the setpoint, not away.
 *      With sensor_direction wrong this instead runs away, which is why the
 *      excursion is bounded and the test aborts if the error grows.
 *   2. The loop is stable   -- the reported error settles instead of ringing.
 *   3. Current is modest    -- the peak |Iq| column is the number that decides
 *      whether the axis will cook itself holding position.
 *
 * Barrel off, supply current-limited. Runs ~14 s.
 */
#define BENCH_STEP_RAD    0.30f
#define BENCH_HOLD_MS     3500u
#define BENCH_PRINT_MS    250u

void closedloop_bench_test(void)
{
	/* Setpoint profile, relative to wherever the shaft is when we start. */
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

			/* Mirrors data_change(): the supervisor now judges the sample this
			 * takes, so it must run first here too. */
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

			/*
			 * Runaway guard. If the shaft ends up further from the setpoint than
			 * any legitimate transient could explain, the loop is pushing the
			 * wrong way -- stop before it winds up to a hard stop. This is the
			 * failure mode a wrong sensor_direction produces.
			 */
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
    // 范围检查（可选）
    if(num > 327.67f) num = 327.67f;
    if(num < -327.68f) num = -327.68f;

    // 放大100倍并四舍五入
    int16_t scaled = (int16_t)(roundf(num * 100.0f));

    // 将int16_t拆分为两个字节
    *high_byte = (scaled >> 8) & 0xFF;
    *low_byte = scaled & 0xFF;
}

/**
  * @brief  两个uint8_t还原为带符号浮点数
  * @param  high_byte: 高位字节
  * @param  low_byte: 低位字节
  * @retval 还原的浮点数（范围-327.68~327.67）
  */
float two_uint8_to_float_signed(uint8_t high_byte, uint8_t low_byte)
{
    // 组合为int16_t（注意：高位需要符号扩展）
    int16_t scaled = (int16_t)((high_byte << 8) | low_byte);

    // 缩小100倍
    return (float)scaled / 100.0f;
}

//串口字符排序
uint8_t frame_sync(uint8_t byte, uint8_t *frame_out)
{
    static uint8_t ring[20];
    static uint8_t w_idx = 0;
    static uint8_t count = 0;

    ring[w_idx] = byte;
    w_idx = (w_idx + 1) % 20;
    if (count < 20) count++;

    if (count >= 20) {
        uint8_t start = w_idx;
        if (ring[start] == 0x55 && ring[(start + 19) % 20] == 0xFF) {
            for (uint8_t i = 0; i < 20; i++) {
                frame_out[i] = ring[(start + i) % 20];
            }
            count = 0;   // 清空计数器，避免重复输出同一帧
            /* Feed the comms watchdog: this is the only place a frame is
             * confirmed well-formed. */
            last_frame_us = micros();
            s_frame_count++;
            return 1;
        }
    }
    return 0;
}
