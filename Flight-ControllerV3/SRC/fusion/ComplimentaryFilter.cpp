/*
 * ComplimentaryFilter.cpp
 *
 *  Created on: Jun 28, 2024
 *      Author: Ikenna
 */

#include <fusion/ComplimentaryFilter.h>
#include <math.h>

#define RAD2DEG 180/M_PI
#define DEG2RAD M_PI/180

ComplimentaryFilter::ComplimentaryFilter(Attitude* att) : Filter(att) {
	// TODO Auto-generated constructor stub

}

ComplimentaryFilter::~ComplimentaryFilter() {
	// TODO Auto-generated destructor stub
}

//void ComplimentaryFilter::init(Filter_setup setup) {
//
//}

void ComplimentaryFilter::update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dT, bool magAvailable)
{
	// Calculate pitch and roll from accelerometer data
//	float acc_pitch = atan2f(ay, sqrtf(ax * ax + az * az)) * RAD2DEG;
//	float acc_roll = atan2f(-ax, az) * RAD2DEG;

	float acc_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;
	float acc_roll = atan2f(ay, az) * RAD2DEG;

	Euler* e = &(att->euler);
    // Integrate gyroscope data
    e->p += gx * dT;
    e->r += gy * dT;

    e->p = m_alpha * (e->p) + (1.0 - m_alpha) * acc_pitch;
    e->r = m_alpha * (e->r) + (1.0 - m_alpha) * acc_roll;

	// Normalise magnetometer measurement
	float norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	mx *= norm;
	my *= norm;
	mz *= norm;

    // Tilt compensation
    float mag_x = mx * cos(e->p * DEG2RAD) + mz * sin(e->p * DEG2RAD);
    float mag_y = mx * sin(e->r *DEG2RAD) * sin(e->p * DEG2RAD) + my * cos(e->r * DEG2RAD) - mz * sin(e->r * DEG2RAD) * cos(e->p * DEG2RAD);

    // Calculate heading
    e->y = atan2f(mag_y, mag_x) * RAD2DEG;
    if (e->y < 0) {
    	e->y += 360;
    }
}
