/*
 * UartReceiver.h
 *
 *  Created on: Jul 12, 2024
 *      Author: Ikenna
 */

#ifndef UART_UARTRECEIVER_H_
#define UART_UARTRECEIVER_H_

#include <usart.h>
#include <CircularBuffer.h>
#include <assert.h>

class UartReceiver {
public:
	UartReceiver();
	virtual ~UartReceiver();

	inline void init(UART_HandleTypeDef* huart, uint8_t buffSize=1) {
		m_uart = huart;
		assert(buffSize <= MAX_BUFFSIZE);
		m_buffSize = buffSize;
	}

	inline void startReceiver() {
		HAL_UART_Receive_IT(m_uart, (uint8_t*)rxData, m_buffSize);
	}

	inline UART_HandleTypeDef* getUart() {
		return m_uart;
	}

	inline bool dataAvailable() {
		return m_buffer.isEmpty() == false;
	}

	inline uint8_t getData() {
		return m_buffer.get();
	}

	inline uint8_t getData(uint8_t* buffer, uint8_t len) {
		uint8_t count = 0;
		while(!m_buffer.isEmpty() && count < len) {
			buffer[count] = m_buffer.get();
			count++;
		}
		return count;
	}

	inline void resetBuffer() {
		m_buffer.reset();
	}

	inline void putData() {
		for(uint8_t i = 0; i < m_buffSize; i++) {
			m_buffer.put(rxData[i]);
		}
		startReceiver();
	}

protected:
	static const uint8_t MAX_BUFFSIZE{32};
	uint8_t m_buffSize{1};
	UART_HandleTypeDef* m_uart;
	CircularBuffer m_buffer;
//	volatile uint8_t rxData;
	volatile uint8_t rxData[MAX_BUFFSIZE];


};

#endif /* UART_UARTRECEIVER_H_ */
