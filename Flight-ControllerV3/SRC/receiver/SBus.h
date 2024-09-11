/*
 * SBus.h
 *
 *  Created on: Jul 12, 2024
 *      Author: Ikenna
 */

#ifndef RECEIVER_SBUS_H_
#define RECEIVER_SBUS_H_

#include <UartReceiver.h>
#include <receiver/iSBusRx.h>
#include <Task.h>


//typedef void (*callback)(SbusData);

class SBus : public Task {
public:
	SBus();
	virtual ~SBus();

	inline void init(UART_HandleTypeDef* huart, iSBusRx* callback) {
		m_uartRx.init(huart, PAYLOAD_LEN_);
		m_callback = callback;

		HAL_UART_AbortReceive(huart);

		__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);  // Enable RXNE interrupt
		HAL_NVIC_EnableIRQ(USART2_IRQn);              // Enable the USART2 interrupt in the NVIC

		__HAL_UART_ENABLE(huart);

		startSBus();
	}

	inline void startSBus() {
		m_uartRx.startReceiver();
	}

	inline void handleRxInterrupt(bool reset) {
		m_uartRx.putData();
		if(reset) {
			m_state = 0;
		}
	}

	inline UartReceiver* getUartRx() {
		return &m_uartRx;
	}

	virtual void taskFunc(timetick_us currenTimeUs);

	bool parse();

private:
	UartReceiver m_uartRx;
	iSBusRx* m_callback;

	static constexpr int8_t PAYLOAD_LEN_ = 23;
	static constexpr int8_t HEADER_LEN_ = 1;
	static constexpr int8_t FOOTER_LEN_ = 1;
	/* SBUS message defs */
	static constexpr int8_t NUM_SBUS_CH_ = 16;
	static constexpr uint8_t HEADER_ = 0x0F;
	static constexpr uint8_t FOOTER_ = 0x00;
	static constexpr uint8_t FOOTER2_ = 0x04;
	static constexpr uint8_t CH17_MASK_ = 0x01;
	static constexpr uint8_t CH18_MASK_ = 0x02;
	static constexpr uint8_t LOST_FRAME_MASK_ = 0x04;
	static constexpr uint8_t FAILSAFE_MASK_ = 0x08;
	/* Parsing state tracking */
	volatile int8_t m_state = 0;
	uint8_t m_prevByte_ = FOOTER_;
	uint8_t m_currByte_;
	/* Buffer for storing messages */
	uint8_t m_Buff[25];
	/* Data */
	bool m_newData_;

	SbusData m_sbusData;
};

#endif /* RECEIVER_SBUS_H_ */
