/*
 * BlackBok.h
 *
 *  Created on: Jul 27, 2024
 *      Author: Ikenna
 */

#ifndef DRIVERS_BLACKBOX_BLACKBOK_H_
#define DRIVERS_BLACKBOX_BLACKBOK_H_

#include <spi.h>
#include <Task.h>
#include <WinbondW25N.h>
#include <typedefs.h>
#include <iLogger.h>

typedef struct __attribute__ ((packed)) {
	timetick_us logTime = 0;
	uint8_t id = 0;
	Euler att;
	M_Speed mSpeed;
	Pid_Vals pid;
	Channel chState;
//	Vector_t<float> optFlw;
	OptFlw_Data optFlw;
	float battVoltage = 0;
	float baroAlt = 0;
	float lat = 0;
	float lon = 0;
	float hMSL = 0;
	uint8_t crc = 0;
}BlackBox_Data;


class BlackBok : public iLogger{
public:
	BlackBok(AHRS* ahrs, MotorControl* mtor, Meter* meter, GPS* gps, Barometer* baro, OpticalFlow* optFlw);
	virtual ~BlackBok();

	void init(UART_HandleTypeDef* huart, SPI_Config config);

	BlackBox_Data getPacket(timetick_us currenTimeUs);

	bool eraseData();

	bool eraseConfig();

	bool fullErase();

	inline void setBlkBoxReq() {
		blkBoxReq = true;
	}

	void log(timetick_us currenTimeUs);

private:
	SPI_HandleTypeDef* m_hspi;
	W25N flashDev;
	uint8_t m_numlogs = 0;
	static const uint16_t cm_MAX_LOG = 2048/sizeof(BlackBox_Data);
	static const uint16_t cm_NUM_LOG_BYTES = cm_MAX_LOG * sizeof(BlackBox_Data);
	static const uint16_t cm_MAX_PAGE = 65471;
	uint16_t m_currentPage = 64;
	bool m_eraseSuccess{false}, m_eraseResp{false};
	bool m_justErased{false};

	BlackBox_Data m_buffer[cm_MAX_LOG];

//	bool blkBoxReq{false};
};

#endif /* DRIVERS_BLACKBOX_BLACKBOK_H_ */
