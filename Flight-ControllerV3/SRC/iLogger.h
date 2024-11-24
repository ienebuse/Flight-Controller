/*
 * Logger.h
 *
 *  Created on: Aug 8, 2024
 *      Author: Ikenna
 */

#ifndef ILOGGER_H_
#define ILOGGER_H_

#include <TimeTick.h>
#include <AHRS.h>
#include <meter/Meter.h>
#include <barometer/Barometer.h>
#include <FlightControl.h>
#include <GPS.h>
#include <OpticalFlow.h>

#include <Task.h>



class iLogger : public Task {
public:
	iLogger(AHRS* ahrs, FlightControl* mtor, Meter* meter, GPS* gps, Barometer* baro, OpticalFlow* optFlw);
	virtual ~iLogger();
	
	virtual void init(UART_HandleTypeDef* huart);

	virtual void log(timetick_us currenTimeUs) = 0;

	virtual void taskFunc(timetick_us currenTimeUs);

	static inline UART_HandleTypeDef* getUart() {
		return m_uart;
	}

	static inline void handleInterrupt() {
		if(rxData == 0xE9){
			blkBoxReq = true;
		}
		rxStarted = false;
		startRx();
	}

	static inline void startRx() {
		if(rxStarted) {
			return;
		}
		HAL_UART_Receive_IT(m_uart, &rxData, 1);
		rxStarted = true;
	}

	uint8_t checkSum(const uint8_t* data, uint8_t len);



protected:
	static UART_HandleTypeDef* m_uart;
	AHRS* m_ahrs;
	FlightControl* m_motor;
	Meter* m_meter;
	GPS* m_gps;
	Barometer* m_barometer;
	OpticalFlow* m_optFlw;
	static uint8_t rxData;
	
	static volatile bool blkBoxReq;
	static volatile bool rxStarted;

};

#endif /* ILOGGER_H_ */
