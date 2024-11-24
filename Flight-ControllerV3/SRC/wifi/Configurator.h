/*
 * Configurator.h
 *
 *  Created on: Nov 13, 2024
 *      Author: Ikenna
 */

#ifndef WIFI_CONFIGURATOR_H_
#define WIFI_CONFIGURATOR_H_

#include <TimeTick.h>
#include <AHRS.h>
#include <meter/Meter.h>
#include <barometer/Barometer.h>
#include <FlightControl.h>
#include <GPS.h>
#include <OpticalFlow.h>
#include <BlackBok.h>
#include <Logger.h>

#include <config.h>

#include <Task.h>

typedef struct __attribute__((packed)) {
	uint32_t Header 		= 0xDEADFACE;
	uint8_t Cmd 			= 6;
	uint8_t Len 			= sizeof(Log_Data) + 1;
	Log_Data logData;
	uint8_t crc;
}Log_Packet;

class Configurator : public Task {
public:
	typedef enum {
		CMD_CONF_REQ = 0x01,
		CMD_CONF_RESP,
		CMD_PID_RESP,
		CMD_CONFIG_DEFAULT = 5,
		CMD_LOG,
		CMD_RESET,
	} ConfigCMD;


	Configurator(AHRS* ahrs, FlightControl* mtor, Meter* meter, GPS* gps, Barometer* baro, OpticalFlow* optFlw, BlackBok* bbox);
	virtual ~Configurator();

	void init(UART_HandleTypeDef* huart);

	static bool canSenLog() {
		return m_canSendLog;
	}

	static bool sendLog(Log_Data log);

	void loadConfig();

	void saveConfig();

	void handleRxInterrupt(bool reset);

	void taskFunc(timetick_us currenTimeUs);

	UART_HandleTypeDef* getUart() {
		return m_uart;
	}

	static Config getConfig() {
		return m_config;
	}

	static Config initConfig() {
		m_config = Config();
		m_defaultConfig = Config();
		return m_config;
	}

	void setReady(bool ready) {
		m_txReady = ready;
	}



private:


	typedef enum {
		SendingNone,
		SendingConfig,
		SendingLog,
	}SendingState;

	UART_HandleTypeDef* m_uart;
	AHRS* m_ahrs;
	FlightControl* m_motor;
	Meter* m_meter;
	GPS* m_gps;
	Barometer* m_barometer;
	OpticalFlow* m_optFlw;
	BlackBok* m_bBox;

	uint8_t m_buffer[256];
	uint16_t m_state = 0;

	static Config m_config;
	static Config m_defaultConfig;

	const uint32_t CONFIG_HEADER{0xDEADFACE};

	bool m_txReady{true};

	bool m_saveNewConfig{false};
	bool m_sendConfig{false};
	SendingState sendingState{SendingNone};

	static bool m_canSendLog;
	static Log_Packet logPacket;
	static bool m_sendNewLog;

	void loadDefaultConfig();


};

#endif /* WIFI_CONFIGURATOR_H_ */
