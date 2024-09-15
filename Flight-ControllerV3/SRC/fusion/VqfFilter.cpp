/*
 * VqfFilter.cpp
 *
 *  Created on: Jun 28, 2024
 *      Author: Ikenna
 */

#include <fusion/VqfFilter.h>

VqfFilter::VqfFilter(Attitude* att) : Filter(att), m_vqf((float)IMU_SAMPLING_PERIOD_US/1000000, (float)IMU_SAMPLING_PERIOD_US/1000000, 0.1f) {
	// TODO Auto-generated constructor stub

}

VqfFilter::~VqfFilter() {
	// TODO Auto-generated destructor stub
}

//void VqfFilter::init(Filter_setup setup) {
//
//}


void VqfFilter::update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dT, bool magAvailable) {
	vqf_real_t gyr[3] = {gx, gy, gz};
	vqf_real_t acc[3] = {ax, ay, az};
	vqf_real_t mag[3] = {mx, my, mz};
	vqf_real_t quat[4];

	magAvailable = false;
	if(magAvailable) {
		m_vqf.update(gyr, acc, mag);
		m_vqf.getQuat9D(quat);
	}
	else {
		m_vqf.update(gyr, acc);
		m_vqf.getQuat6D(quat);
	}

	Quat* q = &(att->quat);

	q->q0 = quat[0];
	q->q1 = quat[1];
	q->q2 = quat[2];
	q->q3 = quat[3];
}

void VqfFilter::update(float ax, float ay, float az, float gx, float gy, float gz, float dT) {
	vqf_real_t gyr[3] = {gx, gy, gz};
	vqf_real_t acc[3] = {ax, ay, az};
	vqf_real_t quat[4];

	m_vqf.update(gyr, acc);
	m_vqf.getQuat6D(quat);

	Quat* q = &(att->quat);

	q->q0 = quat[0];
	q->q1 = quat[1];
	q->q2 = quat[2];
	q->q3 = quat[3];
}

