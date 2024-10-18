/*
 * BlackBok.cpp
 *
 *  Created on: Jul 27, 2024
 *      Author: Ikenna
 */

#include <BlackBok.h>
#include <Application.h>

#define BYTE_START_FLAG1	0x85
#define BYTE_START_FLAG2	0x8B
#define BYTE_END_FLAG1		0xA5
#define BYTE_END_FLAG2		0xAE
#define BYTE_ESCAPE_FLAG	0x9E

typedef struct __attribute__ ((packed)) {
	const uint8_t startByte1 	= BYTE_START_FLAG1;
	const uint8_t startByte2	= BYTE_START_FLAG2;
	BlackBox_Data bbLog;
	const uint8_t endByte1		= BYTE_END_FLAG1;
	const uint8_t endByte2		= BYTE_END_FLAG2;
}Log_Packet_t;

const uint16_t BlackBok::cm_MAX_LOG;;
const uint16_t BlackBok::cm_NUM_LOG_BYTES;

BlackBok::BlackBok(AHRS* ahrs, MotorControl* mtor, Meter* meter, GPS* gps, Barometer* baro, OpticalFlow* optFlw) :iLogger(ahrs, mtor, meter, gps, baro, optFlw) {
	// TODO Auto-generated constructor stub

}

BlackBok::~BlackBok() {
	// TODO Auto-generated destructor stub
}

void BlackBok::init(UART_HandleTypeDef* huart, SPI_Config config) {
//	uint16_t badBlocks[1024];
	BlackBox_Data buffer[cm_MAX_LOG];

	iLogger::init(huart);
	flashDev.init(config);

////	uint16_t numBadBlocks = flashDev.getBadBlocks(badBlocks);
////	bool resp = fullErase();
//	bool resp = eraseData();
//	resp = flashDev.eraseFailed();
//
//	resp = flashDev.readPage(65, 0, (uint8_t*)buffer, cm_NUM_LOG_BYTES);
//
//	BlackBox_Data pckt = getPacket(1234567);
//	m_buffer[5] = pckt;
//	flashDev.loadProgAndExecute(65, 0, (uint8_t*)m_buffer, cm_NUM_LOG_BYTES);
//
//	resp = flashDev.programFailed();
//
//	resp = flashDev.readPage(65, 0, (uint8_t*)buffer, cm_NUM_LOG_BYTES);
//
//	int x = 0;

}

bool BlackBok::eraseData() {
	m_eraseResp = flashDev.bulkErase(64);
	m_justErased = true;
	return m_eraseResp;
}

bool BlackBok::eraseConfig() {
	m_eraseResp = flashDev.blockErase(0);
	m_justErased = true;
	return m_eraseResp;
}

bool BlackBok::fullErase() {
	m_eraseResp = flashDev.bulkErase(0);
	m_justErased = true;
	return m_eraseResp;
}

BlackBox_Data BlackBok::getPacket(timetick_us currenTimeUs) {
	BlackBox_Data m_logData;
	m_logData.id = 0x9E;
	m_logData.logTime = currenTimeUs;

	GPS_UBX_NAV gpsData = m_gps->getData();
	m_logData.lat = (float)gpsData.lat * 1e-7;
	m_logData.lon = (float)gpsData.lon * 1e-7;
	m_logData.hMSL = gpsData.hMSL;


	m_logData.att = m_ahrs->getLog().euler;
//	m_logData.optFlw = m_optFlw->getFlowData();
	m_logData.optFlw = m_optFlw->getOptFlowData();

	ControlLog_t ctrlLog = m_motor->getBBxLog();
	m_logData.mSpeed = ctrlLog.mSpeed;
	m_logData.pid = ctrlLog.pid;
	m_logData.chState = ctrlLog.chState;

	m_logData.battVoltage = m_meter->getBatteryVoltage();

	m_logData.baroAlt = m_barometer->getAltitude();

	uint8_t crc = checkSum((uint8_t*)&m_logData, sizeof(BlackBox_Data) - 1);

	m_logData.crc = crc;

	return m_logData;
}

void BlackBok::log(timetick_us currenTimeUs) {
	if(blkBoxReq) {
		BlackBox_Data buffer[cm_MAX_LOG];
		for(uint16_t page = 64; page <= cm_MAX_PAGE; page++) {
			HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
			bool resp = flashDev.readPage(page, 0, (uint8_t*)buffer, cm_NUM_LOG_BYTES);

			for(uint16_t i = 0; i < cm_MAX_LOG; i++) {
				uint8_t crc = buffer[i].crc;
				uint8_t crcEval = checkSum((uint8_t*)&buffer[i], sizeof(BlackBox_Data) - 1);
				if(crc == crcEval) {
					while(!Application::sUartTxReady) {
						TimeTick::delay_us(100);
					}
					Log_Packet_t pckt = {.bbLog = buffer[i]};
					Application::sUartTxReady = false;
					HAL_UART_Transmit_IT(m_uart, (uint8_t*)&pckt, sizeof(Log_Packet_t));
				}
			}
		}
		BlackBox_Data endData = {.id = 0xD8};
		uint8_t crc = checkSum((uint8_t*)&endData, sizeof(BlackBox_Data) - 1);
		endData.crc = crc;
		Log_Packet_t pckt = {.bbLog = endData};
		while(!Application::sUartTxReady) {
			TimeTick::delay_us(100);
		}
		Application::sUartTxReady = false;
		HAL_UART_Transmit_IT(m_uart, (uint8_t*)&pckt, sizeof(Log_Packet_t));
		blkBoxReq = false;
	}
	if(m_justErased) {
		if(m_eraseResp) {
			Application::buzzerOn();
			TimeTick::delay_ms(300);
			Application::buzzerOff();
			m_eraseSuccess = true;
			m_justErased = false;
		}
		else {
			m_eraseSuccess = false;
			return;
		}
	}

	if(!m_eraseSuccess) {
		return;
	}
	BlackBox_Data data = getPacket(currenTimeUs);
	m_buffer[m_numlogs++] = data;

	if(m_numlogs == cm_MAX_LOG && m_currentPage <= cm_MAX_PAGE) {
		m_numlogs = 0;
//		flashDev.loadProgData(0, (uint8_t*)m_buffer, cm_NUM_LOG_BYTES);
//		flashDev.ProgramExecute(m_currentPage++);

		flashDev.loadProgAndExecute(m_currentPage++, 0, (uint8_t*)m_buffer, cm_NUM_LOG_BYTES);

//	    bool resp = flashDev.readPage(m_currentPage++, 0, (uint8_t*)m_buffer, cm_NUM_LOG_BYTES);
	}
}
