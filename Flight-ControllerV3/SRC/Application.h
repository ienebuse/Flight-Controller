/*
 * Application.h
 *
 *  Created on: Jun 29, 2024
 *      Author: Ikenna
 */

#ifndef APPLICATION_H_
#define APPLICATION_H_

#include <devicedef.h>
#include <ahrs/AHRS.h>
#include <GPS.h>
#include <receiver/SBus.h>
#include <TimeTick.h>
#include <MotorControl.h>
#include <receiver/iSBusRx.h>
#include <sensors/DPS310.h>
#include <Scheduler.h>
#include <Logger.h>
#include <HeartBeat.h>
#include <meter/Meter.h>
#include <barometer/Barometer.h>
#include <OpticalFlow.h>
#include <BlackBok.h>

//#include <Task.h>


class Application: iSBusRx {
public:
	Application();
	virtual ~Application();

	inline uint32_t getTickUs() {
		return (uint32_t)__HAL_TIM_GET_COUNTER(m_devices->appTmr);
	}

	inline void increamentTickUs() {
		static uint16_t temp = 0;
		++m_sysTickuS;
		if(++temp == 1000){
			uwTick += 1;
			temp = 0;
		}
	}

	inline SBus* getSbusRx() {
		return &m_sbusRx;
	}

	inline OpticalFlow* getOptFlwRx() {
			return &m_optflw;
		}

	inline GPS* getM8NRx() {
		return &m_gps;
	}

	inline BlackBok* getBlackBox() {
		return &m_blackBox;
	}

	void init(HAL_Devices_t *devices);

	void run();

	static volatile bool sUartTxReady;

	void handleChannelData(SbusData);

	void checkPIDRequest(uint16_t cmd);

	void checkBlackBloxErase(uint16_t cmd);

	void checkBlackBloxRead(uint16_t cmd);


private:
	AHRS m_ahrs;
	HAL_Devices_t *m_devices;
	volatile uint32_t m_sysTickuS;
	Attitude m_attitude;
	I2C_Bus m_i2cBus;
	SBus m_sbusRx;
	MotorControl m_motorControl;
	Meter m_meter;
//	Dps310 barometer;
	DPS310 barometer;
	GPS m_gps;
	Barometer m_barometer;
	OpticalFlow m_optflw;
	BlackBok m_blackBox;


	Channel m_rxCh;
	Logger m_log;
	HeartBeat m_hrtBt;


	bool m_PidMode{false};

	Scheduler m_scheduler;


};

#endif /* APPLICATION_H_ */
