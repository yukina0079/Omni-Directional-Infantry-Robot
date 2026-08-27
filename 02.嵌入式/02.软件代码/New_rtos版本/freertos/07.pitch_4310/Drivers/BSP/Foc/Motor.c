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
 * Note on the removed E_EN and M_EN helpers:
 * they toggled PA6 and PB5, which are unconnected on this board (new_foc,
 * DRV8313). The DRV8313's EN1/EN2/EN3 pins are tied active in hardware, so
 * there is no software enable line -- Motor_stop() disabling the PWM outputs
 * is the only available way to de-energise the motor.
 */

void Motor_init(void)
{
	/* A stationary field is enough to establish the electrical offset. Do not
	 * sweep the assembled pitch axis or use its mechanical end stops at boot. */
	motor1_pwm_start();
	setPhaseVoltage(MOTOR_ALIGN_VOLTAGE, 0, _3PI_2);
	delay_ms(400);

	as5600_update();
	zero_electric_angle = 0.0f;
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
	 */
	PID_Reset(&PID_Pos);
	PID_Reset(&PID_Vel);
	PID_Reset(&PID_Cur);
}

//开环速度控制
float velocityOpenloop(float target_velocity)
{
	float Ts, Uq;

	Ts = systime_delta_s(&open_loop_timestamp);

	shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts);
	Uq = 6.0f;

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
    float elev;

    /*
     * Gravity feedforward. The barrel COM sits in front of the pivot,
     * so gravity always tries to lower elevation. That is the +Uq
     * direction with sensor_direction = -1. Compensate with a
     * negative bias that peaks at the horizon.
     *
     * Fade it when error_deg > 0 (commanding lower / +Uq). A constant
     * -2 V hold plus a 4.6 deg lead left only P*4.6 ≈ 0.9 V to fight
     * the bias, so a down-stick could not overpower the feedforward.
     */
    elev = Get_Angel_Notrack() - PITCH_ZERO_RAD;
    while (elev > _PI)  elev -= _2PI;
    while (elev < -_PI) elev += _2PI;
    {
        float grav = -PITCH_GRAVITY_HOLD_V * cosf(elev);
        if (error_deg > 0.5f) {
            grav *= 1.0f / (1.0f + (error_deg - 0.5f) / 2.0f);
        }
        output += grav;
    }

    /*
     * Extra volts only when the following error is past one or two
     * encoder LSBs. At hold the term is off (no buzz). On a climb
     * it covers dry friction; gravity is already in the feedforward.
     */
    {
        float cmd = PID_Pos.P * error_deg;
        float eabs = (error_deg < 0.0f) ? -error_deg : error_deg;
        float live = tanhf((eabs - 0.5f) / PITCH_FRICTION_ERR_ON_DEG);
        if (live < 0.0f) {
            live = 0.0f;
        }
        output += live * PITCH_FRICTION_COMP_VOLTAGE
                  * tanhf(cmd / PITCH_FRICTION_CMD_EPS);
    }
    output = _constrain(output, -PID_Pos.limit, PID_Pos.limit);
    setPhaseVoltage(output, 0, _electricalAngle());
}

/*
 * Stick-to-torque path. A 40 V/s slew stops a full-stick slam from
 * appearing as a step on the winding; the hard cap is still
 * PITCH_TORQUE_LIMIT_V. Encoder is only used for commutation.
 */
#define PITCH_UQ_SLEW_V_S   25.0f

void TorqueCommand(float uq_volts)
{
    static float uq_prev = 0.0f;
    static uint32_t t_prev = 0;
    float dt;
    float max_step;

    uq_volts = _constrain(uq_volts, -PITCH_TORQUE_LIMIT_V, PITCH_TORQUE_LIMIT_V);

    dt = (float)micros_since(t_prev) * 1.0e-6f;
    t_prev = micros();
    if ((dt <= 0.0f) || (dt > 0.05f)) {
        dt = 0.001f;
    }
    max_step = PITCH_UQ_SLEW_V_S * dt;
    if (uq_volts > (uq_prev + max_step)) {
        uq_volts = uq_prev + max_step;
    } else if (uq_volts < (uq_prev - max_step)) {
        uq_volts = uq_prev - max_step;
    }
    uq_prev = uq_volts;

    setPhaseVoltage(uq_volts, 0, _electricalAngle());
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
