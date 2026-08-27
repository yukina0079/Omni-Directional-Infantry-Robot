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

float pitch_num = 0.0f;
float joy_num = 0.0f;
float pitch_pid_num = 0.0f;

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
/*
 * 24N28P = 24 slots, 28 poles => 14 pole pairs. One electrical turn is
 * only 360/14 ≈ 25.7 deg mechanical. If sensor_direction is the wrong
 * sign, torque mode kicks the rotor about that far and then the field
 * walks backwards relative to the magnets -- exactly "电角度转了一圈
 * 就再也转不动". Yaw on the same 4310 family measured -1; pitch was
 * still at the untested +1 default.
 */
float sensor_direction = -1.0f;
float pole_pairs = 14;           /* 24N28P: 28 poles / 2 */
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

/*
 * ---------------------------------------------------------------------------
 * Overcurrent supervisor
 * ---------------------------------------------------------------------------
 *
 * The voltage cap is the PRIMARY current limit: stall current is Uq/R.
 * R = 10.7 ohm, Uq <= 6.5 V => about 0.61 A. This supervisor is only the
 * backstop if the bus is high, the winding is cold, or a phase is shorted.
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
#define MONITOR_MAGIC   0x50495431u   /* "PIT1" */

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
    float    pitch_pid_num;   /* raw command from the gimbal, bytes 14/15 */
    float    angle_abs_rad;   /* Get_Angel_Notrack(), [0, 2PI)           */
    float    cal_min_rad;     /* session min of angle_rad (hand sweep)   */
    float    cal_max_rad;     /* session max of angle_rad                */
    uint32_t raw_count;       /* AS5600 0..4095                          */
} pitch_monitor_t;

/* Not static: the linker must emit a symbol so the .map file can be searched
 * for its address. Volatile so the compiler cannot cache the fields across the
 * update, which would leave the debugger reading stale values. */
volatile pitch_monitor_t g_pitch_monitor = { MONITOR_MAGIC };

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
	static uint8_t  s_cal_inited   = 0;
	uint32_t flags = 0;
	float angle;

	if (energised)                                       { flags |= 0x01u; }
	if (encoder_fault)                                   { flags |= 0x02u; }
	if (micros_since(last_frame_us) > COMMS_TIMEOUT_US)  { flags |= 0x04u; }
	if (s_oc_latched)                                    { flags |= 0x08u; }

	angle = sensor_direction * Get_Angel();

	/* Session travel extrema. Reset only happens on MCU reset, so a
	 * hand sweep of the hard stops fills these without any host logic. */
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

/*
 * Clamp a control-frame command so the predicted AS5600 single-turn
 * angle stays inside the mechanical window. sensor_direction is applied
 * because the position loop lives in (direction * encoder) while the
 * stops are recorded on the magnet's own turn.
 */
static float clamp_pitch_cmd(float desired)
{
	#if !PITCH_SOFTWARE_LIMIT_ENABLE
	return desired;
	#else
	float now_ctrl = sensor_direction * Get_Angel();
	float now_nt = Get_Angel_Notrack();

	/* Elevation relative to calibrated horizon, wrapped to [-PI, PI] */
	float elev_now = now_nt - PITCH_ZERO_RAD;
	while (elev_now > _PI)  elev_now -= _2PI;
	while (elev_now < -_PI) elev_now += _2PI;

	/* Predicted elevation from the incoming desired angle change */
	float elev_pred = elev_now + sensor_direction * (desired - now_ctrl);

	/* Safe elevation limits with 2 deg inset buffer */
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

	/* --- 1. communications --- */
	uart1_poll();

	joy_num     = two_uint8_to_float_signed(uart_recv[6],  uart_recv[7]);
	pitch_num     = two_uint8_to_float_signed(uart_recv[12], uart_recv[13]);
	/* ly is 0.001 rad/LSB (remote floatToTwoSint8Milli). */
	{
		int16_t ly_scaled = (int16_t)((uart_recv[14] << 8) | uart_recv[15]);
		pitch_pid_num = (float)ly_scaled / 1000.0f;
	}

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
		 * before the first frame ever arrives. Recapture origin on the
		 * next good frame so a reconnect does not yank to an old zero. */
		s_pos_locked = 0;
		motor_set_enabled(0);
		monitor_update(0, 0.0f, 0.0f);
		return;
	}

	/* --- 4. overcurrent supervision --- */
	if (!overcurrent_ok()) {
		/* Latched on overcurrent. De-energise and wait out the cooldown. */
		motor_set_enabled(0);
		monitor_update(0, 0.0f, 0.0f);
		return;
	}

	/* --- 5. control ---
	 *
	 * Follow the remote ly INTEGRATOR's increments, not its absolute
	 * value. If the shaft cannot keep up (stiction), extra increments
	 * past PITCH_MAX_LEAD_RAD are discarded so error cannot bank up.
	 * Mechanical ends still clamp in AS5600 coordinates.
	 */
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

			/*
			 * Lead discards extra stick travel only. A two-sided
			 * clamp here used to drag s_cmd after a gravity sag
			 * (s_cmd := ang +/- lead) and the setpoint ratcheted
			 * from the upper stop down to the lower one.
			 */
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

/* Disabled by design: startup travel self-test is no longer part of the
 * pitch bring-up. Keep the old bench implementation here for reference only.
 */
#if 0
/*
 * Power-on travel sweep.
 *
 * Why closed-loop, not another electrical-angle jog: we already know the
 * mechanical stops from the hand calibration. Commanding inside
 * PITCH_CMD_MIN/MAX lets the position loop carry the barrel through the
 * real travel, which also proves commutation, sensor_direction and the
 * clamp in one motion. An open-loop electrical sweep does not know where
 * the hard stops are.
 *
 * Rate is bounded in the *setpoint*, not just the voltage ramp, so a
 * 45 deg axis does not slam from one end to the other on the first tick.
 */
#define PITCH_SWEEP_RATE_RAD_S     0.30f    /* ~17 deg/s, easier on the stops */
#define PITCH_SWEEP_HOLD_US        250000u
#define PITCH_SWEEP_ARRIVE_RAD     0.06f    /* ~3.4 deg                      */
#define PITCH_SWEEP_SEG_TIMEOUT_US 5000000u
#define PITCH_SWEEP_WRONG_WAY_RAD  0.10f
/* The normal P controller only produces 0.52 V for the measured 4.3 deg
 * boot-to-zero error, below this pitch assembly's breakaway torque. Apply a
 * self-test-only floor once the error is meaningful; 4.0 V is about 730 mA
 * with the measured 5.5 ohm phase resistance and matches the conservative
 * commissioning ceiling used elsewhere in this firmware. */
#define PITCH_SWEEP_MIN_DRIVE_V         4.00f
#define PITCH_SWEEP_MIN_DRIVE_ERR_RAD   0.035f   /* 2 deg */
#define PITCH_SWEEP_NO_MOTION_CMD_RAD   0.060f   /* 3.4 deg */
#define PITCH_SWEEP_NO_MOTION_RAD       0.020f   /* 1.1 deg */
#define PITCH_SWEEP_NO_MOTION_US        1500000u
/* Sweep targets sit further inside the hard stops than the command clamp.
 * First enable overshot the 3 deg command margin and sat on the lower
 * stop, then never "arrived" at CMD_MIN. */
#define PITCH_SWEEP_INSET_RAD      0.087f   /* ~5 deg from measured stop     */
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

	/* Drop any I accumulated while the previous segment was stuck.
	 * That leftover is what flung the barrel into the far stop after
	 * the first self-test finally broke stiction. */
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

	/* Start with the farther endpoint. A boot position near horizon only has a
	 * few degrees of error, which is below this gravity-loaded axis's breakaway
	 * torque. Going to the farther end first guarantees a meaningful command;
	 * then traverse the opposite end and return to horizon. */
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
