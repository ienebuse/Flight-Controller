/*
 * Logger.h
 *
 *  Created on: Aug 8, 2024
 *      Author: Ikenna
 */

#ifndef LOGGER_LOGGER_H_
#define LOGGER_LOGGER_H_

#include <iLogger.h>
#include <usart.h>

typedef struct __attribute__ ((packed)) {
	timetick_us logTime;
	AhrsLog ahrsLog;
	ControlLog ctrlLog;
	OptFlw_Data optFlw;
	float battVoltage;
	float battCapacity;
	float altitude;
	float lat;
	float lon;
	float hMSL;
}Log_Data;

class Logger : public iLogger{
public:
	Logger(AHRS* ahrs, FlightControl* mtor, Meter* meter, GPS* gps, Barometer* baro, OpticalFlow* optFlw);
	virtual ~Logger();

	virtual void init();

	virtual void log(timetick_us currenTimeUs);

	void handleRxInterrupt(bool reset);

	void handleUSBRxData(uint8_t* rxData);

private:
	char m_data[200];
	Log_Data m_logData;
	static const uint8_t MAX_BUFFER_SIZE{50};
	uint8_t rx_buffer[MAX_BUFFER_SIZE];
	uint8_t m_state = 0;

	void getLogData();
};

#endif /* LOGGER_LOGGER_H_ */
