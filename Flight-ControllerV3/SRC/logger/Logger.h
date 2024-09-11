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
	Vector_t<float> optFlw;
	float battVoltage;
	float altitude;
	float lat;
	float lon;
	float hMSL;
}Log_Data;

class Logger : public iLogger{
public:
	Logger(AHRS* ahrs, MotorControl* mtor, Meter* meter, GPS* gps, Barometer* baro, OpticalFlow* optFlw);
	virtual ~Logger();

	virtual void init(UART_HandleTypeDef* huart);

	virtual void log(timetick_us currenTimeUs);

private:
	char m_data[200];
	Log_Data m_logData;
};

#endif /* LOGGER_LOGGER_H_ */
