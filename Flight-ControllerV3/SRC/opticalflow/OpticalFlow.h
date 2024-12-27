/*
 * OpticalFlow.h
 *
 *  Created on: Aug 9, 2024
 *      Author: Ikenna
 */

#ifndef OPTICALFLOW_H_
#define OPTICALFLOW_H_

#include <typedefs.h>
#include <TimeTick.h>
#include <Task.h>
#include <LowPassFilter.h>
#include <AHRS.h>
#include <uart/UartReceiver.h>
#include <osd/IOSD.h>

typedef union {
	uint8_t buff[2];
	uint16_t val;
}le16_t;

typedef union {
	uint8_t buff[4];
	uint32_t val;
}le32_t;

typedef struct __attribute__((packed)) {
	float vx = 0;
	float vy = 0;
	float h = 0;
	float px = 0;
	float py = 0;
	float wx = 0;
	float wy = 0;
}OptFlw_Data;

class OpticalFlow : public Task {
public:
	OpticalFlow();
	virtual ~OpticalFlow();

	void init(UART_HandleTypeDef* huart, AHRS* ahrs);

	inline void startOptFlw() {
//		m_uartRx.startReceiver();
		HAL_UART_Receive_IT(m_uart, buffer, 1);
	}

	inline void handleRxInterrupt(bool reset) {
		parseOptFlwData();
//		m_uartRx.putData();
//		newEvent = true;
//		if(reset) {
//			m_state = 0;
//		}
	}

	inline UartReceiver* getUartRx() {
		return &m_uartRx;
	}

	inline UART_HandleTypeDef* getUart() {
		return m_uart;
	}

	inline Vector_t<float> getFlowData() {
		Vector_t<float> flwData = currentFlowData;
		Quat q = m_ahrs->getCurrentAttitude().quat;
		float q02 = q.q0*q.q0;
		float q12 = q.q1*q.q1;
		float q22 = q.q2*q.q2;
		float q32 = q.q3*q.q3;

		flwData.z *= (q02 - q12 - q22 + q32);
		return currentFlowData;
	}

	inline Vector_t<float> getLocalPosition() {
		Vector_t<float> lPos;
		Quat q = m_ahrs->getCurrentAttitude().quat;
		float q02 = q.q0*q.q0;
		float q12 = q.q1*q.q1;
		float q22 = q.q2*q.q2;
		float q32 = q.q3*q.q3;

		float _2q1q2 = 2*q.q1*q.q2;
		float _2q0q3 = 2*q.q0*q.q3;

		lPos.x = wPos.x * (q02 + q12 - q22 - q32) + wPos.y*(_2q1q2 + _2q0q3);
		lPos.y = wPos.x * (_2q1q2 - _2q0q3) + wPos.y*(q02 - q12 + q22 - q32);
		lPos.z = currentFlowData.z * (q02 - q12 - q22 + q32);
		return lPos;
	}

	inline OptFlw_Data getOptFlowData() {
		OptFlw_Data flwData;

		Quat q = m_ahrs->getCurrentAttitude().quat;
		float q02 = q.q0*q.q0;
		float q12 = q.q1*q.q1;
		float q22 = q.q2*q.q2;
		float q32 = q.q3*q.q3;

		float _2q1q2 = 2*q.q1*q.q2;
		float _2q0q3 = 2*q.q0*q.q3;

		flwData.vx = currentFlowData.x;
		flwData.vy = currentFlowData.y;
		flwData.px = wPos.x * (q02 + q12 - q22 - q32) + wPos.y*(_2q1q2 + _2q0q3);
		flwData.py = wPos.x * (_2q1q2 - _2q0q3) + wPos.y*(q02 - q12 + q22 - q32);
		flwData.h = currentFlowData.z * (q02 - q12 - q22 + q32);
		flwData.wx = wPos.x;
		flwData.wy = wPos.y;



//		flwData.px = lPos.x;
//		flwData.py = lPos.y;
//		flwData.wx = lPos.x*(q02 + q12 - q22 - q32) + lPos.y*(_2q1q2 - _2q0q3);
//		flwData.wy = lPos.x*(_2q1q2 + _2q0q3) + lPos.y*(q02 - q12 + q22 - q32);

		return flwData;
	}

	inline void resetPos() {
		wPos.x = 0;
		wPos.y = 0;
//		lPos.x = 0;
//		lPos.y = 0;
	}

	inline Vector_t<float>getWorldPos() {
		return wPos;
	}

	bool parseOptFlwData();

	inline void registerOSD(IOSD* osd) {
		m_osd = osd;
	}

	virtual void taskFunc(timetick_us currenTimeUs);


private:
	UartReceiver m_uartRx;
	UART_HandleTypeDef* m_uart;
	AHRS* m_ahrs;
	IOSD* m_osd;
	uint8_t m_state = 0;
    const uint16_t FUNC_LIDAR = 7937;
    const uint16_t FUNC_FLOW = 7938;
    static const uint8_t MAX_BUFFER_SIZE{50};
    uint8_t buffer[MAX_BUFFER_SIZE];
    Vector_t<float> wPos;
    Vector_t<float> lPos;

    volatile float m_xFlwSum = 0, m_yFlwSum = 0, m_hLidar = 0, m_qlty=0;
    float dT = 0, lastTime = 0, totalTime = 0;
    volatile uint32_t flowCount = 0;
    volatile bool flowRdy = false, lidarRdy = false;

    Vector_t<float>currentFlowData, lastFlowData;

    uint8_t checkSum(uint8_t* data, uint8_t len);

    uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a);

    uint16_t le16(const uint8_t* const data);

    uint32_t le32(const uint8_t* const data);

    LowPassFilter xFilt, yFilt, zFilt;

};

#endif /* OPTICALFLOW_H_ */
