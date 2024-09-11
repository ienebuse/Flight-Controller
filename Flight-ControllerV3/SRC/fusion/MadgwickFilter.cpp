/*
 * MadgwickFilter.cpp
 *
 *  Created on: Jan 19, 2023
 *      Author: enebusei
 */


#include "MadgwickFilter.h"
#include "math.h"
#include <TimeTick.h>
#include <string.h>

//#define COMP_RESET	0

static float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


Madgwick::Madgwick(Attitude* att) : Filter(att)//, beta(0.001f * cFilterSetup.GyroMeasError)
{
}

//void Madgwick::init(Filter_setup setup) {
//	this->beta = sqrt(3.0f / 4.0f) * setup.GyroMeasError;
//}

void Madgwick::update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dT, bool magAvailable)
{
	static bool first = true;

	if(magAvailable && first) {

		getInitialOrientation(ax, ay, az, mx, my, mz);
		first = false;
	}

	if(!magAvailable) {
//		float roll = att->euler.r*DEG2RAD;
//		float pitch = att->euler.p*DEG2RAD;
//		// Compensate magnetometer data
//		float mx_comp = mx * cos(pitch) + my * sin(roll) * sin(pitch) + mz * cos(roll) * sin(pitch);
//		float my_comp = my * cos(roll) - mz * sin(roll);
//
//		// Calculate yaw (psi) from compensated magnetometer data
//		float yaw = atan2(-my_comp, mx_comp);
//
//		double t0 = cos(yaw * 0.5);
//		double t1 = sin(yaw * 0.5);
//		double t2 = cos(roll * 0.5);
//		double t3 = sin(roll * 0.5);
//		double t4 = cos(pitch * 0.5);
//		double t5 = sin(pitch * 0.5);
//
//		att->quat.q0 = t2 * t4 * t0 + t3 * t5 * t1;
//		att->quat.q1 = t3 * t4 * t0 - t2 * t5 * t1;
//		att->quat.q2 = t2 * t5 * t0 + t3 * t4 * t1;
//		att->quat.q3 = t2 * t4 * t1 - t3 * t5 * t0;
		update(ax, ay, az,gx, gy, gz, dT);

		return;
	}



	Quat* q = &(att->quat);

	float q1 = q->q0;
	float q2 = q->q1;
	float q3 = q->q2;
	float q4 = q->q3;   // short name local variable for readability

	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0f * q1 * mx;
	_2q1my = 2.0f * q1 * my;
	_2q1mz = 2.0f * q1 * mz;
	_2q2mx = 2.0f * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = sqrt(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

	// Gradient decent algorithm corrective step
	s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	norm = 1.0f/norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

	// Integrate to yield quaternion
	q1 += qDot1 * dT;
	q2 += qDot2 * dT;
	q3 += qDot3 * dT;
	q4 += qDot4 * dT;
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f/norm;

	q->q0 = q1 * norm;
	q->q1 = q2 * norm;
	q->q2 = q3 * norm;
	q->q3 = q4 * norm;

}

void Madgwick::update(float ax, float ay, float az,float gx, float gy, float gz, float dT)
{
#ifdef COMP_RESET
	static uint8_t count = 0;
	static float _dT = 0;

	_dT += dT;
	++count;
	if(count % 10 == 0) {

		compUpdate(ax, ay, az, gx, gy, gz, _dT);
		_dT = 0;
	}

	if(count == 20) {
	    double q2sqr = att->quat.q2 * att->quat.q2;
	    double t0 = -2.0 * (q2sqr + att->quat.q3 * att->quat.q3) + 1.0;
	    double t1 = +2.0 * (att->quat.q1 * att->quat.q2 + att->quat.q0 * att->quat.q3);
	    double t2 = -2.0 * (att->quat.q1 * att->quat.q3 - att->quat.q0 * att->quat.q2);
	    double t3 = +2.0 * (att->quat.q2 * att->quat.q3 + att->quat.q0 * att->quat.q1);
	    double t4 = -2.0 * (att->quat.q1 * att->quat.q1 + q2sqr) + 1.0;

	    t2 = t2 > 1.0 ? 1.0 : t2;
	    t2 = t2 < -1.0 ? -1.0 : t2;

	    compAtt.euler.p = asin(t2)*RAD2DEG;
	    compAtt.euler.r = atan2(t3, t4)*RAD2DEG;
	    compAtt.euler.y = atan2(t1, t0)*RAD2DEG;
	}

	if(count == 100) {
		double t0 = cos(compAtt.euler.y * DEG2RAD * 0.5);
		double t1 = sin(compAtt.euler.y * DEG2RAD * 0.5);
		double t2 = cos(compAtt.euler.r * DEG2RAD * 0.5);
		double t3 = sin(compAtt.euler.r * DEG2RAD * 0.5);
		double t4 = cos(compAtt.euler.p * DEG2RAD * 0.5);
		double t5 = sin(compAtt.euler.p * DEG2RAD * 0.5);

		att->quat.q0 = t2 * t4 * t0 + t3 * t5 * t1;
		att->quat.q1 = t3 * t4 * t0 - t2 * t5 * t1;
		att->quat.q2 = t2 * t5 * t0 + t3 * t4 * t1;
		att->quat.q3 = t2 * t4 * t1 - t3 * t5 * t0;
		count = 0;
		return;
	}
#endif

	Quat* q = &(att->quat);

	float q0 = q->q0;
	float q1 = q->q1;
	float q2 = q->q2;
	float q3 = q->q3;   // short name local variable for readability

	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = 1/sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * dT;
	q1 += qDot2 * dT;
	q2 += qDot3 * dT;
	q3 += qDot4 * dT;

	// Normalise quaternion
	recipNorm = 1/sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q->q0 = q0 * recipNorm;
	q->q1 = q1 * recipNorm;
	q->q2 = q2 * recipNorm;
	q->q3 = q3 * recipNorm;
}

void Madgwick::compUpdate(float ax, float ay, float az,float gx, float gy, float gz, float dT) {
	float acc_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;
	float acc_roll = atan2f(ay, az) * RAD2DEG;

	Euler* e = &(compAtt.euler);
    // Integrate gyroscope data
    e->p += gx * dT;
    e->r += gy * dT;

    e->p = m_alpha * (e->p) + (1.0 - m_alpha) * acc_pitch;
    e->r = m_alpha * (e->r) + (1.0 - m_alpha) * acc_roll;
}

