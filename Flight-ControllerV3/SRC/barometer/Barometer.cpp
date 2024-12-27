/*
 * Barometer.cpp
 *
 *  Created on: Aug 8, 2024
 *      Author: Ikenna
 */

#include <barometer/Barometer.h>

Barometer::Barometer() {
	// TODO Auto-generated constructor stub

}

Barometer::~Barometer() {
	// TODO Auto-generated destructor stub
}

bool Barometer::init(I2C_Bus* i2cBus) {
	return m_sensor.init(i2cBus);
}

float Barometer::getAltitude() {
//	static uint8_t initCount = 0;
//	static float offset = 0;
//	if(initCount == 10 && m_altData.altAvailable) {
//		offset = m_altData.altitude;
//		++initCount;
//	}
//	if(initCount < 10 && m_altData.altAvailable) {
//		++initCount;
//	}
//	m_altData.altAvailable = false;
//	return (m_altData.altitude-offset)*1000;

	return m_altData.altitude;
}

bool Barometer::dataAvailable() {
	return m_altData.altAvailable;
}

void Barometer::setTaskPeriod(timetick_us newPeriod) {
	period = newPeriod;
}

void Barometer::taskFunc(timetick_us currenTimeUs) {
#if USE_BARO_CONT_UPDATE
	m_altData = m_sensor.getContAltitude();
#else
	m_altData = m_sensor.getAltitude();
	m_osd->setOptBaroAltitude(m_altData.altitude);
//	setTaskPeriod(m_altData.acqTimeUs);
#endif


}

