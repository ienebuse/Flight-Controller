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
#include <receiver/SBus.h>
#include <TimeTick.h>
#include <receiver/iSBusRx.h>
#include <Scheduler.h>
#include <Logger.h>
#include <HeartBeat.h>
#include <meter/Meter.h>
#include <barometer/Barometer.h>
#include <blackbox/BlackBok.h>
#include <OpticalFlow.h>
#include <FlightControl.h>
#include <config.h>
#include <Configurator.h>
#include <gps/GPS.h>
#include <sensors/dps/DPS310.h>
#include <Buzzer.h>
#include <osd/OSD.h>

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

	inline Logger* getLogger() {
		return &m_log;
	}

	inline Configurator* getConfigurator() {
		return &m_configurator;
	}

	void init(HAL_Devices_t *devices);

	void run();

	static volatile bool sUartTxReady;

	void handleChannelData(SbusData);

	void checkPIDRequest(uint16_t cmd);

	void checkBlackBloxErase(uint16_t cmd);

	void checkBlackBloxRead(uint16_t cmd);

	static inline void buzzerOn() {
		if(buzz_state) {
			return;
		}
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
		buzz_state = true;
	}

	static inline void buzzerOff() {
		if(!buzz_state) {
			return;
		}
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		buzz_state = false;
	}

	static inline void buzzerToggle() {
		buzz_state ? buzzerOff() : buzzerOn();
	}


private:
	AHRS m_ahrs;
	HAL_Devices_t *m_devices;
	volatile uint32_t m_sysTickuS;
	Attitude m_attitude;
	I2C_Bus m_i2cBus;
	SBus m_sbusRx;
	FlightControl m_flightControl;
	Meter m_meter;
//	Dps310 barometer;
//	dps310::DPS310 barometer;
	GPS m_gps;
	Barometer m_barometer;
	OpticalFlow m_optflw;
	BlackBok m_blackBox;
	Buzzer m_buzzer;

	Channel m_rxCh;
	Logger m_log;
	HeartBeat m_hrtBt;
	Configurator m_configurator;


	OSD m_osd;


	bool m_PidMode{false};

	static bool buzz_state;

	Scheduler m_scheduler;


};

#endif /* APPLICATION_H_ */
