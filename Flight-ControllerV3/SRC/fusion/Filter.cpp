/*
 * Filter.cpp
 *
 *  Created on: Jan 19, 2023
 *      Author: enebusei
 */
#include "Filter.h"
#include <math.h>

Filter::Filter(Attitude* att, void* setupValue)
{
	this->att = att;
	this->mp_setupValue = setupValue;
}

//Filter::Filter(EulerAttitude* e)
//{
//	this->e = e;
//}

Filter::~Filter()
{

}

void Filter::getInitialOrientation(float ax, float ay, float az, float mx, float my, float mz) {
    // Calculate roll (phi) and pitch (theta) from accelerometer data
    float roll = atan2(ay, az);
    float pitch = atan2(-ax, sqrt(ay * ay + az * az));

    // Compensate magnetometer data
    float mx_comp = mx * cos(pitch) + my * sin(roll) * sin(pitch) + mz * cos(roll) * sin(pitch);
    float my_comp = my * cos(roll) - mz * sin(roll);

    // Calculate yaw (psi) from compensated magnetometer data
    float yaw = atan2(-my_comp, mx_comp);

    double t0 = cos(yaw * 0.5);
	double t1 = sin(yaw * 0.5);
	double t2 = cos(roll * 0.5);
	double t3 = sin(roll * 0.5);
	double t4 = cos(pitch * 0.5);
	double t5 = sin(pitch * 0.5);

	Quaternion_t q;

	q.q0 = t2 * t4 * t0 + t3 * t5 * t1;
	q.q1 = t3 * t4 * t0 - t2 * t5 * t1;
	q.q2 = t2 * t5 * t0 + t3 * t4 * t1;
	q.q3 = t2 * t4 * t1 - t3 * t5 * t0;



	float sumSq = q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3;
//	float invSumSq = fastInvereSqrt(sumSq);
	float invSumSq = 1/sqrt(sumSq);

	q.q0 *= invSumSq;
	q.q1 *= invSumSq;
	q.q2 *= invSumSq;
	q.q3 *= invSumSq;


	att->quat.q0 = q.q0;
	att->quat.q1 = q.q1;
	att->quat.q2 = q.q2;
	att->quat.q3 = q.q3;

	att->euler.r = roll * RAD2DEG;
	att->euler.p = pitch * RAD2DEG;
	att->euler.y = yaw * RAD2DEG;
}
