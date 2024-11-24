/*
 * Logger.cpp
 *
 *  Created on: Aug 8, 2024
 *      Author: Ikenna
 */

#include <iLogger.h>

UART_HandleTypeDef* iLogger::m_uart;
volatile bool iLogger::blkBoxReq{false}, iLogger::rxStarted{false};
uint8_t iLogger::rxData{0};

iLogger::iLogger(AHRS* ahrs, FlightControl* mtor, Meter* meter, GPS* gps, Barometer* baro, OpticalFlow* optFlw) :
	m_ahrs(ahrs),
	m_motor(mtor),
	m_meter(meter),
	m_gps(gps),
	m_barometer(baro),
	m_optFlw(optFlw)
{
	// TODO Auto-generated constructor stub

}

iLogger::~iLogger() {
	// TODO Auto-generated destructor stub
}

//void iLogger::log() {
//
//}

void iLogger::init(UART_HandleTypeDef* huart) {
	m_uart = huart;
	startRx();
}

void iLogger::taskFunc(timetick_us currenTimeUs) {
	log(currenTimeUs);
}

static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a)
{
    crc ^= a;
    for (int i = 0; i < 8; ++i) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

uint8_t iLogger::checkSum(const uint8_t* data, uint8_t len) {
	uint8_t ck2 = 0; // initialise CRC

	for (int i = 0; i < len; i++) {
	    ck2 = crc8_dvb_s2(ck2, data[i]);
	}
	return ck2;
}

