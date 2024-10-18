/*
 * Barometer.h
 *
 *  Created on: Aug 8, 2024
 *      Author: Ikenna
 */

#ifndef BAROMETER_BAROMETER_H_
#define BAROMETER_BAROMETER_H_

#include <DPS310.h>
#include <Task.h>

class Barometer : public Task{
public:
	Barometer();
	virtual ~Barometer();

	bool init(I2C_Bus* i2cBus);

	float getAltitude();

	bool dataAvailable();

	virtual void taskFunc(timetick_us currenTimeUs);

private:
	dps310::DPS310 m_sensor;
	AltData m_altData;

	void setTaskPeriod(timetick_us period);
};

#endif /* BAROMETER_BAROMETER_H_ */
