/*
 * AltitudeFilter.h
 *
 *  Created on: Dec 29, 2024
 *      Author: Ikenna
 */

#ifndef FUSION_ALTITUDEFILTER_H_
#define FUSION_ALTITUDEFILTER_H_

#include <Barometer.h>
#include <OpticalFlow.h>

class AltitudeFilter {
public:
	AltitudeFilter(Barometer* baro, OpticalFlow* optFlw);
	virtual ~AltitudeFilter();

	static AltitudeFilter* getInstance() {
		return mp_instance;
	}

	static void start() {
		mp_instance->m_baroGroundOffset = mp_instance->m_baro->getAltitude() - 0.05;
	}

	static float getAltitude();

private:
	static AltitudeFilter* mp_instance;
	Barometer* m_baro;
	OpticalFlow* m_optFlw;
	float m_baroGroundOffset;
};

#endif /* FUSION_ALTITUDEFILTER_H_ */
