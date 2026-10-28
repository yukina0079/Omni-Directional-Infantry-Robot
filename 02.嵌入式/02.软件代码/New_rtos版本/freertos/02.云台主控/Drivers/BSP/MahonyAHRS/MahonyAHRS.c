#include "MahonyAHRS.h"
#include <math.h>

#define sampleFreq         1000.0f
#define twoKpDef           0.12f
#define twoKiDef           0.02f
#define INTEGRAL_LIMIT     0.1f
#define ACC_NORM_MIN_SQ    0.49f
#define ACC_NORM_MAX_SQ    1.69f
#define QUAT_NORM_MIN_SQ   1.0e-12f

volatile float twoKp = twoKpDef;
volatile float twoKi = twoKiDef;

volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;

float invSqrt(float x);
static void mahony_limit_integral(void);
static void mahony_reset_quat(float q[4]);
static unsigned char mahony_float_is_finite(float x);

void MahonyAHRSupdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyAHRSupdateIMU(q, 1.0f / sampleFreq, gx, gy, gz, ax, ay, az);
		return;
	}

	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

        q0q0 = q[0] * q[0];
        q0q1 = q[0] * q[1];
        q0q2 = q[0] * q[2];
        q0q3 = q[0] * q[3];
        q1q1 = q[1] * q[1];
        q1q2 = q[1] * q[2];
        q1q3 = q[1] * q[3];
        q2q2 = q[2] * q[2];
        q2q3 = q[2] * q[3];
        q3q3 = q[3] * q[3];

        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	gx *= (0.5f * (1.0f / sampleFreq));
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * gx - qc * gy - q[3] * gz);
	q[1] += (qa * gx + qc * gz - q[3] * gy);
	q[2] += (qa * gy - qb * gz + q[3] * gx);
	q[3] += (qa * gz + qb * gy - qc * gx);

	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}

void MahonyAHRSupdateIMU(float q[4], float dt, float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	float acc_norm_sq;
	float quat_norm_sq;

	if (dt < 0.0005f) {
		dt = 0.0005f;
	} else if (dt > 0.005f) {
		dt = 0.005f;
	}

	acc_norm_sq = ax * ax + ay * ay + az * az;

	if ((acc_norm_sq > ACC_NORM_MIN_SQ) && (acc_norm_sq < ACC_NORM_MAX_SQ)) {

		recipNorm = invSqrt(acc_norm_sq);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		halfvx = q[1] * q[3] - q[0] * q[2];
		halfvy = q[0] * q[1] + q[2] * q[3];
		halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];

		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		if (twoKi > 0.0f) {
			integralFBx += twoKi * halfex * dt;
			integralFBy += twoKi * halfey * dt;
			integralFBz += twoKi * halfez * dt;
			mahony_limit_integral();
			gx += integralFBx;
			gy += integralFBy;
			gz += integralFBz;
		} else {
			integralFBx = 0.0f;
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	gx *= (0.5f * dt);
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * gx - qc * gy - q[3] * gz);
	q[1] += (qa * gx + qc * gz - q[3] * gy);
	q[2] += (qa * gy - qb * gz + q[3] * gx);
	q[3] += (qa * gz + qb * gy - qc * gx);

	quat_norm_sq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
	if ((quat_norm_sq < QUAT_NORM_MIN_SQ) ||
	    !mahony_float_is_finite(q[0]) || !mahony_float_is_finite(q[1]) ||
	    !mahony_float_is_finite(q[2]) || !mahony_float_is_finite(q[3])) {
		mahony_reset_quat(q);
		return;
	}

	recipNorm = invSqrt(quat_norm_sq);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}

static void mahony_limit_integral(void)
{
	if (integralFBx > INTEGRAL_LIMIT) {
		integralFBx = INTEGRAL_LIMIT;
	} else if (integralFBx < -INTEGRAL_LIMIT) {
		integralFBx = -INTEGRAL_LIMIT;
	}
	if (integralFBy > INTEGRAL_LIMIT) {
		integralFBy = INTEGRAL_LIMIT;
	} else if (integralFBy < -INTEGRAL_LIMIT) {
		integralFBy = -INTEGRAL_LIMIT;
	}
	if (integralFBz > INTEGRAL_LIMIT) {
		integralFBz = INTEGRAL_LIMIT;
	} else if (integralFBz < -INTEGRAL_LIMIT) {
		integralFBz = -INTEGRAL_LIMIT;
	}
}

static void mahony_reset_quat(float q[4])
{
	q[0] = 1.0f;
	q[1] = 0.0f;
	q[2] = 0.0f;
	q[3] = 0.0f;
	integralFBx = 0.0f;
	integralFBy = 0.0f;
	integralFBz = 0.0f;
}

static unsigned char mahony_float_is_finite(float x)
{
	union {
		float f;
		unsigned long u;
	} v;
	v.f = x;
	return (unsigned char)(((v.u >> 23) & 0xFFul) != 0xFFul);
}

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
