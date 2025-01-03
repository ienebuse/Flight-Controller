/*
 * Barometer.h
 *
 *  Created on: Aug 8, 2024
 *      Author: Ikenna
 */

#ifndef BAROMETER_BAROMETER_H_
#define BAROMETER_BAROMETER_H_

#include <sensors/dps/DPS310.h>
#include <Task.h>
#include "osd/IOSD.h"
#include <LowPassFilter.h>

class Barometer : public Task{
public:
	Barometer();
	virtual ~Barometer();

	bool init(I2C_Bus* i2cBus);

	float getAltitude();

	bool dataAvailable();

	inline void setGroundOffset() {
		m_groundOffset = m_altData.altitude;
	}

	inline void registerOSD(IOSD* osd) {
		m_osd = osd;
	}

	virtual void taskFunc(timetick_us currenTimeUs);

private:
	dps310::DPS310 m_sensor;
	AltData m_altData;
	float m_groundOffset;
	IOSD* m_osd;
	LowPassFilter m_filt;

	void setTaskPeriod(timetick_us period);
};

#endif /* BAROMETER_BAROMETER_H_ */
