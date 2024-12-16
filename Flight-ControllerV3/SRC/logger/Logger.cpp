/*
 * Logger.cpp
 *
 *  Created on: Aug 8, 2024
 *      Author: Ikenna
 */

#include <Logger.h>
#include <stdio.h>
#include <string.h>
#include <Application.h>
#include "usbd_cdc_if.h"

#define BYTE_START_FLAG1	0x8E
#define BYTE_START_FLAG2	0x81
#define BYTE_END_FLAG1		0xA5
#define BYTE_END_FLAG2		0xAE
#define BYTE_ESCAPE_FLAG	0x9E
//#define FSLP_BYTE_START_FLAG_ESCAPED = 0x81
//#define FSLP_BYTE_END_FLAG_ESCAPED = 0xA1
//#define FSLP_BYTE_ESCAPE_FLAG_ESCAPED = 0x91
//#define FSLP_BYTE_ESCAPED_DELTA = 0x0D

#define RX_BYTE_START_FLAG1	0x85
#define RX_BYTE_START_FLAG2	0x8B

typedef struct __attribute__ ((packed)) {
	const uint8_t startByte 	= BYTE_START_FLAG1;
	const uint8_t id			= BYTE_ESCAPE_FLAG;
	Log_Data log;
	uint8_t crc;
	const uint8_t endByte1		= BYTE_END_FLAG1;
	const uint8_t endByte2		= BYTE_END_FLAG2;
}Log_Packet_t;


typedef struct __attribute__ ((packed)) {
	uint8_t axis;
	float p;
	float i;
	float d;
	uint8_t crc;
}PID_Axis_Packet_t;

typedef struct __attribute__ ((packed)) {
	uint8_t axis;
	float rollP;
	float rollI;
	float rollD;
	float pitchP;
	float pitchI;
	float pitchD;
	float yawP;
	float yawI;
	float yawD;
	uint8_t crc;
}PID_Axes_Packet_t;


const uint8_t Logger::MAX_BUFFER_SIZE;

Logger::Logger(AHRS* ahrs, FlightControl* mtor, Meter* meter, GPS* gps, Barometer* baro, OpticalFlow* optFlw) :iLogger(ahrs, mtor, meter, gps, baro, optFlw) {
	// TODO Auto-generated constructor stub

}

Logger::~Logger() {
	// TODO Auto-generated destructor stub
}

void Logger::init() {
	iLogger::init(nullptr);
}

void Logger::getLogData() {
	GPS_UBX_NAV gpsData = m_gps->getData();
	m_logData.lat = (float)gpsData.lat * 1e-7;
	m_logData.lon = (float)gpsData.lon * 1e-7;
	m_logData.hMSL = gpsData.hMSL;
	m_logData.ahrsLog = m_ahrs->getLog();
	m_logData.optFlw = m_optFlw->getOptFlowData();
	m_logData.ctrlLog = m_motor->getLog();
	m_logData.battVoltage = m_meter->getBatteryVoltage();
	m_logData.battCapacity = m_meter->getBatteryCapacity();
	if(m_barometer->dataAvailable()){
		m_logData.altitude = m_barometer->getAltitude();
	}
}

void Logger::log(timetick_us currenTimeUs) {

	if(!Configurator::getConfig().settings.EnableLogging) {
		return;
	}
	static uint8_t buffer[40];
	static uint8_t pcktIndx = 0;
	const uint8_t MaxPcktSegment = 4;
	const uint8_t packtSize = sizeof(Log_Data) / MaxPcktSegment;
	static timetick_us lastLogTime = 0;

	if(1) {
		if(currenTimeUs - lastLogTime > 100000 && Configurator::canSenLog()) {
			getLogData();
			if(Configurator::sendLog(m_logData)) {
				lastLogTime = currenTimeUs;
			}
		}
	}
	else if (Application::sUartTxReady) {
		if(pcktIndx == 0) {
			getLogData();
		}

		buffer[0] = BYTE_START_FLAG1;
		buffer[1] = BYTE_START_FLAG2;
		buffer[2] = pcktIndx;

		uint8_t indx = pcktIndx * packtSize;
		memcpy(&buffer[3],(uint8_t*)(&m_logData) + indx, packtSize);

		uint8_t crc = checkSum(&buffer[2], packtSize+1);
		buffer[packtSize + 3] = crc;
		buffer[packtSize + 4] = BYTE_END_FLAG1;
		buffer[packtSize + 5] = BYTE_END_FLAG2;



//		HAL_UART_Transmit_DMA(m_uart, (uint8_t*) &logPckt, sizeof(Log_Packet_t));
//		HAL_UART_Transmit_DMA(m_uart, (uint8_t*) buffer, packtSize + 6);

		Application::sUartTxReady = false;
		CDC_Transmit_FS((uint8_t*) buffer, packtSize + 6);

//		Application::sUartTxReady = false;
		pcktIndx  = (pcktIndx + 1) % 4;
	}
}

void Logger::handleRxInterrupt(bool reset) {
	if(reset) {
		m_state = 0;
	}
	uint8_t remLen = 1;
	uint8_t data;
	switch(m_state) {
		case 0:
			data = rx_buffer[0];
			if(data == RX_BYTE_START_FLAG1) {
				m_state = 1;
				remLen = 1;
			}
			break;
		case 1:
			data = rx_buffer[0];
			if(data == RX_BYTE_START_FLAG2) {
				m_state = 2;
				remLen = 1;
			}
			else {
				m_state = 0;
				remLen = 1;
			}
			break;
		case 2:
			remLen = rx_buffer[0];
			m_state = 3;
			break;
		case 3:
			uint8_t axis = rx_buffer[0];
			uint8_t crc = 0;

			if(axis >= 0 && axis < 3) {
				PID_Axis_Packet_t* data = (PID_Axis_Packet_t*)rx_buffer;
				crc = data->crc;
				uint8_t crc_eval = checkSum((uint8_t*)data, sizeof(PID_Axis_Packet_t)-1);
				if(crc == crc_eval) {
					m_motor->setPIDGains((eAxis)data->axis, data->p, data->i, data->d);
				}
			}
			else if(axis == 3) {
				PID_Axes_Packet_t* data = (PID_Axes_Packet_t*)rx_buffer;
				crc = data->crc;
				uint8_t crc_eval = checkSum((uint8_t*)data, sizeof(PID_Axis_Packet_t)-1);
				if(crc == crc_eval) {
					m_motor->setPIDGains(ROLL, data->rollP, data->rollI, data->rollD);
					m_motor->setPIDGains(PITCH, data->pitchP, data->pitchI, data->pitchD);
					m_motor->setPIDGains(YAW, data->yawP, data->yawI, data->yawD);
				}
			}

			m_state = 0;
			remLen = 1;
			break;
	}

	HAL_UART_Receive_DMA(m_uart, rx_buffer, remLen);
}

void Logger::handleUSBRxData(uint8_t* rxData) {
	uint8_t crc = 0;

	if((rxData[0] == RX_BYTE_START_FLAG1) && (rxData[1] == RX_BYTE_START_FLAG2) && ((rxData[2] == 14) || (rxData[2] == 38))) {

		uint8_t axis = rxData[3];

		if(axis >= 0 && axis < 3) {
			PID_Axis_Packet_t* data = (PID_Axis_Packet_t*)&rxData[3];
			crc = data->crc;
			uint8_t crc_eval = checkSum((uint8_t*)data, sizeof(PID_Axis_Packet_t)-1);
			if(crc == crc_eval) {
				m_motor->setPIDGains((eAxis)data->axis, data->p, data->i, data->d);
			}
		}
		else if(axis == 3) {
			PID_Axes_Packet_t* data = (PID_Axes_Packet_t*)&rxData[3];
			crc = data->crc;
			uint8_t crc_eval = checkSum((uint8_t*)data, sizeof(PID_Axis_Packet_t)-1);
			if(crc == crc_eval) {
				m_motor->setPIDGains(ROLL, data->rollP, data->rollI, data->rollD);
				m_motor->setPIDGains(PITCH, data->pitchP, data->pitchI, data->pitchD);
				m_motor->setPIDGains(YAW, data->yawP, data->yawI, data->yawD);
			}
		}
	}
}
