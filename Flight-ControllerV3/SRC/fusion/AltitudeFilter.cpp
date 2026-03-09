/*
 * AltitudeFilter.cpp
 *
 *  Created on: Dec 29, 2024
 *      Author: Ikenna
 */

#include <AltitudeFilter.h>
#include <math.h>
#include <Debugger.h>

AltitudeFilter* AltitudeFilter::mp_instance;


constexpr float DeadBand{50};
constexpr float Alpha{0.8};

AltitudeFilter::AltitudeFilter(Barometer* baro, OpticalFlow* optFlw) : m_baro(baro), m_optFlw(optFlw) {
	// TODO Auto-generated constructor stub
	mp_instance = this;
}

AltitudeFilter::~AltitudeFilter() {
	// TODO Auto-generated destructor stub
}

float AltitudeFilter::getAltitude() {
	static char buff[64];
	float alt;
	float ofAlt = mp_instance->m_optFlw->getHeight();
	float baroAlt = (mp_instance->m_baro->getAltitude() - mp_instance->m_baroGroundOffset) * 1000;

	if(abs(ofAlt - baroAlt) > DeadBand) {
		alt =  baroAlt;
	}
	else {
		alt = Alpha * ofAlt + (1 - Alpha) * baroAlt;
	}

//	snprintf(buff, 64, "%.3f,%.3f,%.3f\r\n", ofAlt, baroAlt, alt);
//	Debugger::sendDbgLog(buff);

	return alt;
}

