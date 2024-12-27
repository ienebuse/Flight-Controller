/*
 * VTX.cpp
 *
 *  Created on: Dec 17, 2024
 *      Author: Ikenna
 */

#include <VTX.h>
#include <string.h>

VTX::VTX() {
	// TODO Auto-generated constructor stub

}

VTX::~VTX() {
	// TODO Auto-generated destructor stub
}

void VTX::init(UART_HandleTypeDef* huart) {
	m_uart = huart;
}

static uint8_t checksum(uint8_t *trampBuf)
{
    uint8_t cksum = 0;

    for (int i = 1 ; i < 14 ; i++) {
        cksum += trampBuf[i];
    }

    return cksum;
}

void VTX::sendCommand(uint8_t cmd, uint16_t param) {
	memset(m_trampReqBuffer, 0, 16);
	m_trampReqBuffer[0] = 0x0F;
	m_trampReqBuffer[1] = cmd;
	m_trampReqBuffer[2] = param & 0xff;
	m_trampReqBuffer[3] = (param >> 8) & 0xff;
	m_trampReqBuffer[14] = checksum(m_trampReqBuffer);

	HAL_UART_Transmit_IT(m_uart, m_trampReqBuffer, 16);
}

