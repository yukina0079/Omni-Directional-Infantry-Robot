#include "FOC.h"
#include "pwm.h"
#include <math.h>
#include "uart1.h"
#include "Data.h"

/*
 * All maths here is deliberately single-precision.
 *
 * The STM32F103 is a Cortex-M3 with NO FPU, so every double-precision
 * operation is a software library call. The previous version used sin(),
 * atan2(), sqrt() and fmod() -- all double -- which meant each float argument
 * was widened with __aeabi_f2d, processed in software double, then narrowed
 * back with __aeabi_d2f. setPhaseVoltage() compiled to 972 bytes and dominated
 * the control-loop period. Using the f-suffixed variants plus a sine lookup
 * table removes essentially all of that cost.
 */

#define SINE_TABLE_SIZE   128

/*
 * Quarter-wave sine table: 129 entries covering 0..PI/2 inclusive, so the
 * interpolation at the top of the quadrant does not need to wrap.
 * Generated as sin(i * (PI/2) / 128).
 */
static const float sine_quarter[SINE_TABLE_SIZE + 1] = {
    0.000000f, 0.012272f, 0.024541f, 0.036807f, 0.049068f, 0.061321f, 0.073565f, 0.085797f,
    0.098017f, 0.110222f, 0.122411f, 0.134581f, 0.146730f, 0.158858f, 0.170962f, 0.183040f,
    0.195090f, 0.207111f, 0.219101f, 0.231058f, 0.242980f, 0.254866f, 0.266713f, 0.278520f,
    0.290285f, 0.302006f, 0.313682f, 0.325310f, 0.336890f, 0.348419f, 0.359895f, 0.371317f,
    0.382683f, 0.393992f, 0.405241f, 0.416430f, 0.427555f, 0.438616f, 0.449611f, 0.460539f,
    0.471397f, 0.482184f, 0.492898f, 0.503538f, 0.514103f, 0.524590f, 0.534998f, 0.545325f,
    0.555570f, 0.565732f, 0.575808f, 0.585798f, 0.595699f, 0.605511f, 0.615232f, 0.624860f,
    0.634393f, 0.643832f, 0.653173f, 0.662416f, 0.671559f, 0.680601f, 0.689541f, 0.698376f,
    0.707107f, 0.715731f, 0.724247f, 0.732654f, 0.740951f, 0.749136f, 0.757209f, 0.765167f,
    0.773010f, 0.780737f, 0.788346f, 0.795837f, 0.803208f, 0.810457f, 0.817585f, 0.824589f,
    0.831470f, 0.838225f, 0.844854f, 0.851355f, 0.857729f, 0.863973f, 0.870087f, 0.876070f,
    0.881921f, 0.887640f, 0.893224f, 0.898674f, 0.903989f, 0.909168f, 0.914210f, 0.919114f,
    0.923880f, 0.928506f, 0.932993f, 0.937339f, 0.941544f, 0.945607f, 0.949528f, 0.953306f,
    0.956940f, 0.960431f, 0.963776f, 0.966976f, 0.970031f, 0.972940f, 0.975702f, 0.978317f,
    0.980785f, 0.983105f, 0.985278f, 0.987301f, 0.989177f, 0.990903f, 0.992480f, 0.993907f,
    0.995185f, 0.996313f, 0.997290f, 0.998118f, 0.998795f, 0.999322f, 0.999699f, 0.999925f,
    1.000000f
};

/*
 * Sine by quadrant folding plus linear interpolation.
 * Worst-case error is about 1.5e-5, far below the resolution of a 12-bit
 * encoder driving a 1440-step PWM, and it costs a handful of cycles instead of
 * a software-double library call.
 */
float _sin(float a)
{
    float quadrant_pos;
    uint32_t idx;
    float frac;
    float s;
    int   negate = 0;

    /* Fold to [0, 2PI). */
    a = _normalizeAngle(a);

    /* Fold to [0, PI) and remember the sign. */
    if (a >= _PI) {
        a -= _PI;
        negate = 1;
    }

    /* Fold to [0, PI/2] by mirroring the second quadrant. */
    if (a > _PI_2) {
        a = _PI - a;
    }

    /* Map [0, PI/2] onto table index space [0, 128]. */
    quadrant_pos = a * ((float)SINE_TABLE_SIZE / _PI_2);
    idx  = (uint32_t)quadrant_pos;
    if (idx >= SINE_TABLE_SIZE) {
        /* Clamp so the idx+1 access below stays inside the table. */
        idx  = SINE_TABLE_SIZE - 1;
        frac = 1.0f;
    } else {
        frac = quadrant_pos - (float)idx;
    }

    s = sine_quarter[idx] + frac * (sine_quarter[idx + 1] - sine_quarter[idx]);

    return negate ? -s : s;
}

float _cos(float a)
{
    return _sin(a + _PI_2);
}

// 归一化角度到 [0,2PI]
float _normalizeAngle(float angle)
{
    float a = fmodf(angle, _2PI);   /* fmodf, not fmod: no double promotion */
    return a >= 0.0f ? a : (a + _2PI);
}

//求解电角度
float _electricalAngle(void)
{
    /*
     * Reads the CACHED single-turn mechanical angle. as5600_update() must have
     * been called earlier in this control cycle. Previously this function
     * triggered its own bit-banged I2C transaction, so a cascaded loop sampled
     * the encoder three or four times per cycle.
     */
    return _normalizeAngle((sensor_direction * pole_pairs) * Get_Angel_Notrack()
                           - zero_electric_angle);
}

float FOC_voltage_limit(void)
{
    return FOC_MAX_MODULATION * voltage_power_supply;
}

void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
	float Uout;
	uint32_t sector;
	float T0,T1,T2;
	float Ta,Tb,Tc;

	if(Ud != 0.0f)
	{
		Uout = sqrtf(Ud*Ud + Uq*Uq) / voltage_power_supply;
		angle_el = _normalizeAngle(angle_el + atan2f(Uq, Ud));
	}
	else
	{
		Uout = Uq / voltage_power_supply;
		angle_el = _normalizeAngle(angle_el + _PI_2);
	}

	/* Clamp to the SVPWM linear range in both directions. A negative Uout is
	 * legitimate -- it simply means a 180 degree phase shift. */
	Uout = _constrain(Uout, -FOC_MAX_MODULATION, FOC_MAX_MODULATION);

	sector = (uint32_t)(angle_el / _PI_3) + 1;
	/*
	 * Guard the sector index. _normalizeAngle can return a value that rounds to
	 * exactly 2PI, which would yield sector 7 and fall through to the default
	 * case, momentarily driving all three phases to zero -- a torque notch.
	 */
	if (sector > 6u) {
		sector = 6u;
	}

	T1 = _SQRT3 * _sin((float)sector*_PI_3 - angle_el) * Uout;
	T2 = _SQRT3 * _sin(angle_el - ((float)sector-1.0f)*_PI_3) * Uout;
	T0 = 1.0f - T1 - T2;

	//扇区计算
	switch(sector)
	{
		case 1:
			Ta = T1 + T2 + T0/2;
			Tb = T2 + T0/2;
			Tc = T0/2;
			break;
		case 2:
			Ta = T1 +  T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T0/2;
			break;
		case 3:
			Ta = T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T2 + T0/2;
			break;
		case 4:
			Ta = T0/2;
			Tb = T1+ T0/2;
			Tc = T1 + T2 + T0/2;
			break;
		case 5:
			Ta = T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T2 + T0/2;
			break;
		case 6:
			Ta = T1 + T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T0/2;
			break;
		default:
			Ta = 0;
			Tb = 0;
			Tc = 0;
	}

	/*
	 * Clamp the duty ratios before scaling. Mathematically Ta/Tb/Tc stay within
	 * [0,1] given the modulation clamp above, but floating point rounding can
	 * push them a hair negative -- and converting a negative float to uint16_t
	 * is undefined behaviour that would wrap to a huge compare value.
	 */
	Ta = _constrain(Ta, 0.0f, 1.0f);
	Tb = _constrain(Tb, 0.0f, 1.0f);
	Tc = _constrain(Tc, 0.0f, 1.0f);

	motor1_pwm_set((uint16_t)(Ta*PWM_PERIOD), (uint16_t)(Tb*PWM_PERIOD), (uint16_t)(Tc*PWM_PERIOD));
}
