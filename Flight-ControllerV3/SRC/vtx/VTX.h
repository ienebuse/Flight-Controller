/*
 * VTX.h
 *
 *  Created on: Dec 17, 2024
 *      Author: Ikenna
 */

#ifndef VTX_VTX_H_
#define VTX_VTX_H_

#include "usart.h"

class VTX {
public:

	VTX();

	virtual ~VTX();

	void init(UART_HandleTypeDef* huart);

	inline UART_HandleTypeDef* getUart() {
		return m_uart;
	}

	inline void txComplete() {

	}

	inline void handleRxInterrupt() {

	}

private:
	UART_HandleTypeDef* m_uart;
	uint8_t m_trampReqBuffer[16];
	uint8_t m_trampRespBuffer[16];

	// Device limits, read from device during init
	uint32_t trampRFFreqMin = 0;
	uint32_t trampRFFreqMax = 0;
	uint32_t trampRFPowerMax;

	// Device status, read from device periodically
	uint32_t trampCurFreq = 0;
	uint16_t trampCurConfPower = 0; // Configured power
	uint16_t trampCurActPower = 0; // Actual power
	uint8_t trampCurPitMode = 0; // Expect to startup out of pitmode
	int16_t trampCurTemp = 0;
	uint8_t trampCurControlMode = 0;

	// Device configuration, desired state of device
	uint32_t trampConfFreq = 0;
	uint16_t trampConfPower = 0;
	uint8_t trampConfPitMode = 0; // Initially configured out of pitmode

	// Last device configuration, last desired state of device - used to reset
	// retry count
	uint32_t trampLastConfFreq = 0;
	uint16_t trampLastConfPower = 0;
	uint8_t trampLastConfPitMode = 0; // Mirror trampConfPitMode

	// Retry count
//	uint8_t trampRetryCount = TRAMP_MAX_RETRIES;

	void sendCommand(uint8_t cmd, uint16_t param);
};

#endif /* VTX_VTX_H_ */
