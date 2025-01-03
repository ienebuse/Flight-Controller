/*
 * OSD.cpp
 *
 *  Created on: Dec 16, 2024
 *      Author: Ikenna
 */

#include <osd/OSD.h>
#include <AltitudeFilter.h>

OSD::OSD(Meter* meter) :
	m_meter(meter)
{
	// TODO Auto-generated constructor stub

}

OSD::~OSD() {
	// TODO Auto-generated destructor stub
}

void OSD::init(SPI_Config config) {
	osdDriver.init(config.hspi, config.csPort, config.csPin);
}

void OSD::taskFunc(timetick_us currenTimeUs) {
	osdDriver.add_battery_info(m_battVolt, m_battCap, 1, 1);
	osdDriver.add_altitude(m_optFlwAlt, 0, -2);
//	osdDriver.add_altitude(m_baroAlt * 1000, 0, -1);
	osdDriver.add_altitude(AltitudeFilter::getAltitude(), 0, -1);
	osdDriver.add_gps_info(m_lon, m_lat, m_height, 0);
	osdDriver.add_heading(m_heading, 1, 3);
}

