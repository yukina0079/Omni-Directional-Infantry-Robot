#include "Motor.h"
#include <math.h>          /* fmodf, used by position_error_deg() */
#include "delay.h"
#include "systime.h"
#include "uart1.h"
#include "adc.h"
#include "pwm.h"
#include "as5600.h"

static float shaft_angle = 0.0f;
static uint32_t open_loop_timestamp = 0;

/*
 * Rail-fill state for PositionCloseloop(), at file scope so Motor_stop() can
 * clear it.
 *
 * These were function-level statics, which put them out of reach of
 * Motor_stop() -- and that mattered. s_rail_v is slewed at 80 V/s and can sit
 * near PID_Pos.limit (5.5 V) when an interlock de-energises the axis. On
 * re-engagement the loop starts from ~0 error (data_change() recaptures the
 * shaft angle), so the rail term SHOULD be zero, but s_rail_v resumes from its
 * stale value and only walks back down at max_step per cycle. With the dt
 * fallback pinning max_step at 0.08 V, unwinding 5.5 V takes ~69 cycles, i.e.
 * ~32 ms of full-rail voltage bearing no relation to the current error, in a
 * direction set by whatever the axis was doing before it stopped.
 *
 * Note this became a pure artefact only once the overcurrent path started
 * clearing s_pos_locked (Data.c): before that the axis re-engaged against a
 * large stale error, so a high s_rail_v looked plausible. The two fixes belong
 * together.
 *
 * Motor_stop() already clears the three PID structs for exactly this reason --
 * "an integrator that wound up before the stop would still be loaded when the
 * loop is re-engaged". This is the same class of cross-engagement state; it was
 * simply invisible to that cleanup.
 */
static float    s_rail_v  = 0.0f;
static uint32_t s_rail_us = 0;

/*
 * Note on the removed E_EN and M_EN helpers:
 * they toggled PA6 and PB5, which are unconnected on this board (new_foc,
 * DRV8313). The DRV8313's EN1/EN2/EN3 pins are tied active in hardware, so
 * there is no software enable line -- Motor_stop() disabling the PWM outputs
 * is the only available way to de-energise the motor.
 */

void Motor_init(void)
{
	float angle;
	int i;

	motor1_pwm_start();

	/*
	 * Electrical zero calibration.
	 *
	 * Sweep the electrical angle one revolution forward and one back to unstick
	 * the rotor from any cogging detent, then park it at a known electrical
	 * angle and record what the encoder reads there.
	 *
	 * IMPORTANT: this physically drives the rotor. On an assembled gimbal it
	 * swings the barrel for ~2 s. If the axis is against a hard stop during
	 * this sweep the recorded zero will be wrong and every subsequent
	 * commutation angle will be offset -- which is a runaway on the first
	 * closed-loop enable. Consider recording a fixed value once and skipping
	 * the sweep in production.
	 */
	for(i = 0; i <= 500; i++)
	{
		angle = _3PI_2 + _2PI * (float)i / 500.0f;
		setPhaseVoltage(MOTOR_ALIGN_VOLTAGE, 0, angle);
		delay_ms(2);
	}

	for(i = 500; i >= 0; i--)
	{
		angle = _3PI_2 + _2PI * (float)i / 500.0f;
		setPhaseVoltage(MOTOR_ALIGN_VOLTAGE, 0, angle);
		delay_ms(2);
	}

	setPhaseVoltage(0, 0, angle);
	delay_ms(100);

	/* Park at electrical _3PI_2 and let the rotor settle before sampling. */
	setPhaseVoltage(MOTOR_ALIGN_VOLTAGE, 0, _3PI_2);
	delay_ms(300);

	/* Refresh the encoder cache, then derive the electrical zero from it. */
	as5600_update();
	zero_electric_angle = 0.0f;              /* must be zero for this reading */
	zero_electric_angle = _electricalAngle();

	setPhaseVoltage(0, 0, _3PI_2);
	Motor_stop();

	printf("zero_electric_angle = %f\r\n", zero_electric_angle);
}

/*
 * ---------------------------------------------------------------------------
 * Commissioning test: encoder direction AND pole pair count
 * ---------------------------------------------------------------------------
 *
 * Determines sensor_direction without any external instrument, and validates
 * pole_pairs as a free by-product.
 *
 * Method: drive the electrical angle open-loop through a known span and measure
 * how far the shaft actually moved. The rotor follows the commanded field, so
 *
 *     mechanical_delta = electrical_span / pole_pairs
 *
 * with the SIGN telling us how the encoder is oriented relative to the phase
 * order, and the MAGNITUDE telling us whether pole_pairs is right.
 *
 * Why this matters more than any other single check: sensor_direction enters
 * _electricalAngle(), so getting it wrong makes the commutation angle run
 * backwards. A position loop built on that is positive feedback -- the axis
 * accelerates away from the setpoint instead of toward it. That is the classic
 * gimbal runaway, and it cannot be diagnosed from a schematic.
 *
 * Separating the two failure modes is the point of checking magnitude as well
 * as sign: a wrong sign is a wiring/mounting convention, a wrong magnitude
 * means pole_pairs is misdeclared, and the two need completely different fixes.
 *
 * Safety: the rotor turns ~0.29 revolutions forward, slowly, at
 * MOTOR_ALIGN_VOLTAGE. Barrel off, supply current-limited.
 */
void motor_direction_test(void)
{
	const int   steps    = 400;
	const float el_span  = 4.0f * _2PI;    /* 4 electrical revolutions */
	float angle_start, angle_end, mech_delta, expected, implied_pp;
	float el;
	int   i;

	printf("\r\n=== encoder direction / pole pair test ===\r\n");

	motor1_pwm_start();

	/* Park on a known electrical angle and let the rotor settle, so the start
	 * sample is taken from a rotor that is actually at rest. */
	setPhaseVoltage(MOTOR_ALIGN_VOLTAGE, 0, 0.0f);
	delay_ms(500);
	as5600_update();
	angle_start = Get_Angel();

	/*
	 * Sweep forward. as5600_update() runs every step, not just at the ends:
	 * Get_Angel() is a multi-turn value maintained by detecting wraps between
	 * consecutive samples, so it stays valid only if we keep sampling. Reading
	 * just the endpoints would risk missing a wrap entirely.
	 */
	for (i = 0; i <= steps; i++) {
		el = el_span * (float)i / (float)steps;
		setPhaseVoltage(MOTOR_ALIGN_VOLTAGE, 0, el);
		delay_ms(5);
		as5600_update();
	}

	delay_ms(300);
	as5600_update();
	angle_end = Get_Angel();

	setPhaseVoltage(0, 0, el);
	Motor_stop();

	mech_delta = angle_end - angle_start;
	expected   = el_span / pole_pairs;

	printf("  commanded %.2f el.rad over %d steps\r\n", el_span, steps);
	printf("  shaft moved %.4f rad, expected %+.4f rad\r\n",
	       mech_delta, expected);

	if (mech_delta > 0.0f) {
		sensor_direction = 1.0f;
		printf("  -> sensor_direction = +1 (encoder agrees with phase order)\r\n");
	} else {
		sensor_direction = -1.0f;
		printf("  -> sensor_direction = -1 (encoder INVERTED vs phase order)\r\n");
	}

	/*
	 * Magnitude cross-check. A shaft that barely moved means the rotor did not
	 * track the field at all (stalled against a stop, or the drive voltage is
	 * too low), which would make the sign verdict above meaningless -- so say so
	 * rather than reporting a confident direction.
	 */
	mech_delta = (mech_delta < 0.0f) ? -mech_delta : mech_delta;
	if (mech_delta < 0.1f) {
		printf("  -> WARNING shaft barely moved: rotor did not follow the field.\r\n");
		printf("     Direction verdict above is NOT trustworthy. Check that the\r\n");
		printf("     axis is free, and raise MOTOR_ALIGN_VOLTAGE.\r\n");
		return;
	}

	implied_pp = el_span / mech_delta;
	printf("  implied pole_pairs = %.2f (declared %.0f)\r\n",
	       implied_pp, pole_pairs);
	if (implied_pp > pole_pairs * 0.85f && implied_pp < pole_pairs * 1.15f) {
		printf("  -> pole_pairs confirmed\r\n");
	} else {
		printf("  -> pole_pairs MISMATCH. Commutation will be wrong; set\r\n");
		printf("     pole_pairs = %.0f in Data.c and re-run.\r\n", implied_pp);
	}
}

void Motor_start(void)
{
	PID_Reset(&PID_Pos);
	PID_Reset(&PID_Vel);
	PID_Reset(&PID_Cur);
	motor1_pwm_start();
}

void Motor_stop(void)
{
	motor1_pwm_stop();
	/*
	 * Clear the loop state too. Without this, an integrator that wound up
	 * before the stop would still be loaded when the loop is re-engaged, and
	 * the motor would jump on the first cycle.
	 *
	 * s_rail_v is part of that state even though it lives outside the PID
	 * structs: it is slewed, not recomputed, so it carries across an
	 * engagement boundary the same way an integrator does. Clearing s_rail_us
	 * too keeps the timestamp meaningful -- micros_since(0) lands in the dt
	 * fallback, which is the right answer for a first cycle.
	 */
	s_rail_v  = 0.0f;
	s_rail_us = 0;
	PID_Reset(&PID_Pos);
	PID_Reset(&PID_Vel);
	PID_Reset(&PID_Cur);
}

//开环速度控制
/*
 * Open-loop (sensorless) velocity: integrate a commanded rate into a synthetic
 * shaft angle and commutate from that, ignoring the encoder entirely.
 *
 * Bench/bring-up only -- nothing in the live control path calls this. Useful for
 * proving the three phases and the modulator work before trusting the encoder,
 * because a wrong sensor_direction cannot affect it.
 *
 * The drive voltage was a bare 6.0f. That is inside the modulator's ceiling at
 * the declared 11.7 V bus (FOC_voltage_limit() = 0.577 * 11.7 = 6.75 V), so it
 * worked -- but it is a constant that silently stops being valid if the bus
 * voltage is ever revised. Below about 10.4 V, 6.0 V exceeds the linear range
 * and setPhaseVoltage() clamps the modulation index instead of reporting
 * anything, so the waveform distorts and the axis loses torque with no
 * indication of why. Taking the smaller of the two keeps the intent (a fixed,
 * generous open-loop drive) while making the ceiling follow the bus.
 *
 * Worth knowing what this current level is: 6.0 V into the measured 5.5 ohm
 * winding is about 1.09 A, which sits just under the OC_LIMIT_A of 1.20 A in
 * Data.c. The overcurrent supervisor does not run on this path, so nothing here
 * will trip -- run it current-limited at the supply, and briefly.
 */
float velocityOpenloop(float target_velocity)
{
	float Ts, Uq, Ulim;

	Ts = systime_delta_s(&open_loop_timestamp);

	shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts);

	Uq   = 6.0f;
	Ulim = FOC_voltage_limit();
	if (Uq > Ulim) {
		Uq = Ulim;
	}

	setPhaseVoltage(Uq, 0, shaft_angle * pole_pairs);
	return Uq;
}

/*
 * A note on units for all the closed loops below:
 * Get_Angel() and Get_Velocity() are in radians and rad/s, but the error fed
 * to the PID is converted to DEGREES (the *180/PI factor). So the P/I/D gains
 * are "volts per degree of error", and the limit is in volts.
 */

/*
 * Position error, wrapped to the shortest path, in DEGREES.
 *
 * Both the command and Get_Angel() are continuous multi-turn angles, so their
 * raw difference grows without bound: after the axis has made three turns the
 * error can legitimately read 6*PI even though the shaft is pointing exactly
 * where it was asked to. Feeding that to the PID would drive a full-speed slew
 * to unwind turns that do not need unwinding.
 *
 * This matters most for the chassis-spin (小陀螺) case that is coming. There the
 * command is a world-frame heading, which is inherently a mod-2*PI quantity: as
 * the chassis spins past its own zero the command steps from +PI to -PI while
 * the shaft angle keeps accumulating. Without wrapping, that 2*PI step reads as
 * a huge error and the axis lunges a full turn backwards at exactly the moment
 * it should hold still.
 *
 * Wrapping to [-PI, PI] makes the loop always take the short way round, which
 * is both the correct behaviour and the only one the mechanics like.
 */
float position_error_rad(float target)
{
    float err = target - sensor_direction * Get_Angel();

    err = fmodf(err, _2PI);
    if (err > PI) {
        err -= _2PI;
    } else if (err < -PI) {
        err += _2PI;
    }

    return err;
}

/* Same error in DEGREES, which is the unit the PID gains are expressed in. */
static float position_error_deg(float target)
{
    return position_error_rad(target) * 180.0f / PI;
}

//位置闭环
void PID_Pos_Set(float P, float I, float D, float limit)
{
    PID_Pos.P = P;
    PID_Pos.I = I;
    PID_Pos.D = D;
    PID_Pos.limit = limit;
}

void PositionCloseloop(float position)
{
    float error_deg = position_error_deg(position);
    float output = PID_operator(&PID_Pos, error_deg);

    /*
     * Smooth Coulomb assist on commanded effort (P * error), not sign(error).
     * tanhf is continuous: full +/-COMP on a real move, ~0 at hold, no 0.5 deg
     * switch to chatter across.  Not an integrator -- a wrapped yaw error at
     * +/-PI must not wind up while the bearing is stuck.
     */
    {
        float cmd = PID_Pos.P * error_deg;
        float eabs = (error_deg < 0.0f) ? -error_deg : error_deg;
        float live = tanhf((eabs - 0.5f) / YAW_FRICTION_ERR_ON_DEG);
        if (live < 0.0f) {
            live = 0.0f;
        }
        output += live * YAW_FRICTION_COMP_VOLTAGE
                  * tanhf(cmd / YAW_FRICTION_CMD_EPS);
    }

    /*
     * Lead-rail fill. The 5-take capture showed the +side sitting on
     * the 0.15 rad window at ~2.3 V / 0.23 A, shaft velocity 0, while
     * the stick integrator was thrown away. Blend toward +/-limit as
     * |error| approaches the window so a full lead means full torque.
     * Slewed locally: PID_operator's ramp does not cover this term.
     */
    {
        /* s_rail_v / s_rail_us are at file scope so Motor_stop() can clear
         * them -- see the note at the top of this file. */
        float eabs = (error_deg < 0.0f) ? -error_deg : error_deg;
        float span = YAW_RAIL_ERR_FULL_DEG - YAW_RAIL_ERR_ON_DEG;
        float rail = 0.0f;
        float want = 0.0f;
        float dt;
        float max_step;

        if ((span > 0.0f) && (eabs > YAW_RAIL_ERR_ON_DEG)) {
            rail = (eabs - YAW_RAIL_ERR_ON_DEG) / span;
            if (rail > 1.0f) {
                rail = 1.0f;
            }
        }
        if (rail > 0.0f) {
            float sign = (error_deg > 0.0f) ? 1.0f : -1.0f;
            want = rail * (sign * PID_Pos.limit - output);
        }

        dt = (float)micros_since(s_rail_us) * 1.0e-6f;
        s_rail_us = micros();
        if ((dt <= 0.0f) || (dt > 0.05f)) {
            dt = 0.001f;
        }
        max_step = 80.0f * dt;
        if (want > (s_rail_v + max_step)) {
            s_rail_v += max_step;
        } else if (want < (s_rail_v - max_step)) {
            s_rail_v -= max_step;
        } else {
            s_rail_v = want;
        }
        output += s_rail_v;
    }
    output = _constrain(output, -PID_Pos.limit, PID_Pos.limit);
    setPhaseVoltage(output, 0, _electricalAngle());
}

//速度闭环
void PID_Vel_Set(float P, float I, float D, float limit)
{
    PID_Vel.P = P;
    PID_Vel.I = I;
    PID_Vel.D = D;
    PID_Vel.limit = limit;
}

void VelocityCloseloop(float Velocity)
{
    float output;
    output = PID_operator(&PID_Vel, (Velocity - sensor_direction*Get_Velocity())*180.0f/PI);
    setPhaseVoltage(output, 0, _electricalAngle());
}

//电流环
void PID_Cur_Set(float P, float I, float D, float limit)
{
    PID_Cur.P = P;
    PID_Cur.I = I;
    PID_Cur.D = D;
    PID_Cur.limit = limit;
}

void CurrentCloseloop(float current)
{
    float output;
    output = PID_operator(&PID_Cur, (current - Get_Current()));
    setPhaseVoltage(output, 0, _electricalAngle());
}

//位置速度闭环
void Position_VelocityCloseloop(float position)
{
    float output;
    output = PID_operator(&PID_Pos, position_error_deg(position));
    VelocityCloseloop(output);
}

//速度电流闭环
void Velocity_CurrentCloseloop(float velocity)
{
    float output;
    output = PID_operator(&PID_Vel, (velocity - sensor_direction*Get_Velocity())*180.0f/PI);
    CurrentCloseloop(output);
}

//位置电流闭环
void Position_CurrentCloseloop(float position)
{
    float output;
    output = PID_operator(&PID_Pos, (position - sensor_direction*Get_Angel())*180.0f/PI);
    CurrentCloseloop(output);
}
