#include "Motor.h"
#include <math.h>
#include "delay.h"
#include "systime.h"
#include "uart1.h"
#include "adc.h"
#include "pwm.h"
#include "as5600.h"

static float shaft_angle = 0.0f;
static uint32_t open_loop_timestamp = 0;

void Motor_init(void)
{

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

void motor_direction_test(void)
{
	const int   steps    = 400;
	const float el_span  = 4.0f * _2PI;
	float angle_start, angle_end, mech_delta, expected, implied_pp;
	float el;
	int   i;

	printf("\r\n=== encoder direction / pole pair test ===\r\n");

	motor1_pwm_start();

	setPhaseVoltage(MOTOR_ALIGN_VOLTAGE, 0, 0.0f);
	delay_ms(500);
	as5600_update();
	angle_start = Get_Angel();

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

	PID_Reset(&PID_Pos);
	PID_Reset(&PID_Vel);
	PID_Reset(&PID_Cur);
}

float velocityOpenloop(float target_velocity)
{
	float Ts, Uq;

	Ts = systime_delta_s(&open_loop_timestamp);

	shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts);
	Uq = 6.0f;

	setPhaseVoltage(Uq, 0, shaft_angle * pole_pairs);
	return Uq;
}

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

static float position_error_deg(float target)
{
    return position_error_rad(target) * 180.0f / PI;
}

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

void Position_VelocityCloseloop(float position)
{
    float output;
    output = PID_operator(&PID_Pos, position_error_deg(position));
    VelocityCloseloop(output);
}

void Velocity_CurrentCloseloop(float velocity)
{
    float output;
    output = PID_operator(&PID_Vel, (velocity - sensor_direction*Get_Velocity())*180.0f/PI);
    CurrentCloseloop(output);
}

void Position_CurrentCloseloop(float position)
{
    float output;
    output = PID_operator(&PID_Pos, (position - sensor_direction*Get_Angel())*180.0f/PI);
    CurrentCloseloop(output);
}
