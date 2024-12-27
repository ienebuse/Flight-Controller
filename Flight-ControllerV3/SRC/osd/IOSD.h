/*
 * IOSD.h
 *
 *  Created on: Dec 16, 2024
 *      Author: Ikenna
 */

#ifndef OSD_IOSD_H_
#define OSD_IOSD_H_

class IOSD {
public:
	IOSD();
	virtual ~IOSD();

	virtual void setOptFlwAltitude(float optFlwAltitude) {
		m_optFlwAlt = optFlwAltitude;
	}

	virtual void setOptBaroAltitude(float baroAltitude) {
		m_baroAlt = baroAltitude;
	}

	virtual void setBatteryInfo(float voltage, float capacity) {
		m_battVolt = voltage;
		m_battCap = capacity;
	}

	virtual void setHeading(float heading) {
		m_heading = heading;
	}

	virtual void setGPSInfo(float lon, float lat, float height) {
		m_lon = lon * 1e-7;
		m_lat = lat * 1e-7;
		m_height = height;
	}

protected:
	float m_optFlwAlt, m_baroAlt;
	float m_battVolt, m_battCap;
	float m_heading;
	float m_lon, m_lat, m_height;
};

#endif /* OSD_IOSD_H_ */
