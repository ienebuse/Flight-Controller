/*
 * M8N.h
 *
 *  Created on: Jul 29, 2024
 *      Author: Ikenna
 */

#ifndef GPS_GPS_H_
#define GPS_GPS_H_

#include <main.h>
#include <TimeTick.h>
#include <Task.h>
#include <CircularBuffer.h>
#include <osd/IOSD.h>

typedef struct
{
	unsigned char CLASS;
	unsigned char ID;
	unsigned short length;

	unsigned int iTOW;
	signed int lon;
	signed int lat;
	signed int height;
	signed int hMSL;
	unsigned int hAcc;
	unsigned int vAcc;
	int32_t vx;
	int32_t vy;
	int32_t vz;

	float lon_f64;
	float lat_f64;
	bool new_data = false;
	timetick_us lastDataTime = 0;
}GPS_UBX_NAV;



class GPS : public Task{
public:
	GPS();
	virtual ~GPS();


	void init(UART_HandleTypeDef* huart);

	GPS_UBX_NAV getData();

	void posllhRequest();

	void pvtRequest();

	void rxHandler();

	UART_HandleTypeDef* getUart();

	inline void registerOSD(IOSD* osd) {
		m_osd = osd;
	}

	virtual void taskFunc(timetick_us currenTimeUs);

private:

	typedef enum {
		POSLLH = 36,
		PVT	= 100,
	}NAV_Data_Length;


	UART_HandleTypeDef* m_uart;
	GPS_UBX_NAV navData;
	uint8_t* gps_rx_buf = nullptr;
	uint8_t activeBuff = 0;
	uint8_t doubleBuffer[2][150];
	bool gps_rx_cplt_flag{false};
	uint8_t m_rxData;
	uint8_t m_rxState{0};
	bool m_rxError{false};
	timetick_us m_startTime{0};
	const timetick_us cm_TIMEOUT_US{3000000};
	uint8_t m_dataLength{0};
	IOSD* m_osd;

	void transmitData(uint8_t* data, uint16_t len);
	bool chkSumCheck(unsigned char* data, unsigned char len);
	void parseNavPOSLLH();
	void parseNavPVT();
};

#endif /* GPS_GPS_H_ */
