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

#define BYTE_START_FLAG1	0x8E
#define BYTE_START_FLAG2	0x81
#define BYTE_END_FLAG1		0xA5
#define BYTE_END_FLAG2		0xAE
#define BYTE_ESCAPE_FLAG	0x9E
//#define FSLP_BYTE_START_FLAG_ESCAPED = 0x81
//#define FSLP_BYTE_END_FLAG_ESCAPED = 0xA1
//#define FSLP_BYTE_ESCAPE_FLAG_ESCAPED = 0x91
//#define FSLP_BYTE_ESCAPED_DELTA = 0x0D

typedef struct __attribute__ ((packed)) {
	const uint8_t startByte 	= BYTE_START_FLAG1;
	const uint8_t id			= BYTE_ESCAPE_FLAG;
	Log_Data log;
	uint8_t crc;
	const uint8_t endByte1		= BYTE_END_FLAG1;
	const uint8_t endByte2		= BYTE_END_FLAG2;
}Log_Packet_t;




Logger::Logger(AHRS* ahrs, MotorControl* mtor, Meter* meter, GPS* gps, Barometer* baro, OpticalFlow* optFlw) :iLogger(ahrs, mtor, meter, gps, baro, optFlw) {
	// TODO Auto-generated constructor stub

}

Logger::~Logger() {
	// TODO Auto-generated destructor stub
}

void Logger::init(UART_HandleTypeDef* huart) {
	iLogger::init(huart);
}


//void Logger::log(timetick_us currenTimeUs) {
//	if (Application::sUartTxReady) {
//		GPS_UBX_NAV gpsData = m_gps->getData();
//		if (gpsData.new_data) {
//			m_logData.lat = gpsData.lat_f64;
//			m_logData.lon = gpsData.lon_f64;
//			m_logData.hMSL = gpsData.hMSL;
//		}
//
//		m_logData.optFlw = m_optFlw->getVel();
//		m_logData.ahrsLog = m_ahrs->getLog();
//		m_logData.ctrlLog = m_motor->getLog();
//		m_logData.battVoltage = m_meter->getBatteryVoltage();
//		if(m_barometer->dataAvailable()){
//			m_logData.altitude = m_barometer->getAltitude();
//		}
//		snprintf(m_data, 200,
//				"r: %.3f\t\tp: %.3f\t\ty: %.3f\t\th: %.1f\t\tdT:%.6f\t\t%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\t\t%.1f\t\t%.1f\t\t%.3f,%.3f,%.3f\t\t%.3f,%.3f,%.3f\n",
//				m_logData.ahrsLog.euler.r, m_logData.ahrsLog.euler.p, m_logData.ahrsLog.euler.y, m_logData.ahrsLog.heading, m_logData.ahrsLog.dT,
//				m_logData.ctrlLog.m1, m_logData.ctrlLog.m2, m_logData.ctrlLog.m3, m_logData.ctrlLog.m4,
//				m_logData.ctrlLog.roll.Kp, m_logData.ctrlLog.roll.Ki, m_logData.ctrlLog.roll.Kd,
//				m_logData.ctrlLog.pitch.Kp, m_logData.ctrlLog.pitch.Ki, m_logData.ctrlLog.pitch.Kd,
//				m_logData.ctrlLog.yaw.Kp, m_logData.ctrlLog.yaw.Ki, m_logData.ctrlLog.yaw.Kd,
//				m_logData.battVoltage,
//				m_logData.altitude, // altitude
//				m_logData.lat, m_logData.lon,(float)m_logData.hMSL,
//				m_logData.optFlw.x, m_logData.optFlw.y, m_logData.optFlw.z
//		); //vx,vy,vz);
//		HAL_UART_Transmit_DMA(m_uart, (const uint8_t*) (m_data),
//				strlen(m_data));
//		Application::sUartTxReady = false;
//	}
//}

void Logger::log(timetick_us currenTimeUs) {
	static uint8_t buffer[40];
	static uint8_t pcktIndx = 0;
	const uint8_t MaxPcktSegment = 4;
	const uint8_t packtSize = sizeof(Log_Data) / MaxPcktSegment;



	if (Application::sUartTxReady) {
		if(pcktIndx == 0) {
			GPS_UBX_NAV gpsData = m_gps->getData();
//			if (gpsData.new_data) {
				m_logData.lat = (float)gpsData.lat * 1e-7;
				m_logData.lon = (float)gpsData.lon * 1e-7;
				m_logData.hMSL = gpsData.hMSL;
//			}
			m_logData.ahrsLog = m_ahrs->getLog();
			OptFlw_Data optFlw = m_optFlw->getOptFlowData();
//			m_logData.optFlw = m_optFlw->getFlowData();
			m_logData.optFlw.x = optFlw.px;
			m_logData.optFlw.y = optFlw.py;
			m_logData.optFlw.z = optFlw.h;
			m_logData.ctrlLog = m_motor->getLog();
			m_logData.battVoltage = m_meter->getBatteryVoltage();
			if(m_barometer->dataAvailable()){
				m_logData.altitude = m_barometer->getAltitude();
			}
		}

//		Log_Packet_t logPckt;
//		logPckt.log = m_logData;
//		uint8_t crc = checkSum((uint8_t*)(&logPckt) + 1, sizeof(Log_Packet_t) - 3);
//		logPckt.crc = crc;


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
		HAL_UART_Transmit_DMA(m_uart, (uint8_t*) buffer, packtSize + 6);
		Application::sUartTxReady = false;
		pcktIndx  = (pcktIndx + 1) % 4;
	}
}
