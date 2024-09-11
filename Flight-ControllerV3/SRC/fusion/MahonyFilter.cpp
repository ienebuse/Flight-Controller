/*
 * MahonyFilter.cpp
 *
 *  Created on: Jan 19, 2023
 *      Author: enebusei
 */


#include "MahonyFilter.h"
#include "math.h"


Mahony::Mahony(Attitude* att) : Filter(att), Kp(cFilterSetup.Kp), Ki(cFilterSetup.Ki)
{
}

//void Mahony::init(Filter_setup setup) {
//	this->Kp = setup.Kp;
//	this->Ki = setup.Ki;
//}

void Mahony::update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dT, bool magAvailable)
{
	static bool first = true;

	if(first) {
//		getInitialOrientation(ax, ay, az, mx, my, mz);
		first = false;
		return;
	}

	float spinRate = sqrt(gx*gx + gy*gy + gz*gz);
//	spinRate = spinRate * DEG2RAD;

//	magAvailable = false;

	Quat* q = &(att->quat);

	float q1 = q->q0;
	float q2 = q->q1;
	float q3 = q->q2;
	float q4 = q->q3;   // short name local variable for readability

	float norm;
	float hx, hy, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex = 0, ey = 0, ez = 0;
	float pa, pb, pc;

	// Auxiliary variables to avoid repeated arithmetic
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

//	static float eInt[3] = {0.0f, 0.0f, 0.0f};

	// Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f / norm;        // use reciprocal for division
	ax *= norm;
	ay *= norm;
	az *= norm;

	vx = 2.0f * (q2q4 - q1q3);
	vy = 2.0f * (q1q2 + q3q4);
	vz = q1q1 - q2q2 - q3q3 + q4q4;

	ex = (ay * vz - az * vy);
	ey = (az * vx - ax * vz);
	ez = (ax * vy - ay * vx);

	if(magAvailable) {

	// Normalise magnetometer measurement
		norm = sqrt(mx * mx + my * my + mz * mz);
		if (norm != 0.0f) { //return; // handle NaN
			norm = 1.0f / norm;        // use reciprocal for division
			mx *= norm;
			my *= norm;
			mz *= norm;

			// Reference direction of Earth's magnetic field
			hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
			hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
			bx = sqrt((hx * hx) + (hy * hy));
			bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

			// Estimated direction of gravity and magnetic field
	//		vx = 2.0f * (q2q4 - q1q3);
	//		vy = 2.0f * (q1q2 + q3q4);
	//		vz = q1q1 - q2q2 - q3q3 + q4q4;
			wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
			wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
			wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

			// Error is cross product between estimated direction and measured direction of gravity
			ex += (my * wz - mz * wy);
			ey += (mz * wx - mx * wz);
			ez += (mx * wy - my * wx);
		}
	}

	if (Ki > 0.0f)
	{
		if(spinRate < 20) {
			eInt[0] += ex;      // accumulate integral error
			eInt[1] += ey;
			eInt[2] += ez;
		}
	}
	else
	{
		eInt[0] = 0.0f;     // prevent integral wind up
		eInt[1] = 0.0f;
		eInt[2] = 0.0f;
	}

	// Apply feedback terms
	gx = gx + Kp * ex + Ki * eInt[0];
	gy = gy + Kp * ey + Ki * eInt[1];
	gz = gz + Kp * ez + Ki * eInt[2];

	// Integrate rate of change of quaternion
	pa = q2;
	pb = q3;
	pc = q4;
	q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * dT);
	q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * dT);
	q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * dT);
	q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * dT);

	// Normalise quaternion
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	norm = 1.0f / norm;
	q->q0 = q1 * norm;
	q->q1 = q2 * norm;
	q->q2 = q3 * norm;
	q->q3 = q4 * norm;
 }


