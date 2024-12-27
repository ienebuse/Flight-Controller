/*
 * OSD.h
 *
 *  Created on: Dec 16, 2024
 *      Author: Ikenna
 */

#ifndef OSD_OSD_H_
#define OSD_OSD_H_

#include <Task.h>
#include "AT7456.h"
#include "typedefs.h"
#include "Meter.h"
#include "OpticalFlow.h"
#include "Barometer.h"
#include "IOSD.h"

class OSD : public Task, public IOSD {
public:
	OSD(Meter* meter);
	virtual ~OSD();

	void init(SPI_Config config);

	virtual void taskFunc(timetick_us currenTimeUs);

private:
	AT7456 osdDriver;
	Meter* m_meter;
};

#endif /* OSD_OSD_H_ */
