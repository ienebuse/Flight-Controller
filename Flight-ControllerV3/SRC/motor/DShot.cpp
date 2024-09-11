/*
 * DShot.cpp
 *
 *  Created on: Jul 10, 2024
 *      Author: Ikenna
 */

/* Definition */
#define MHZ_TO_HZ(x) 			((x) * 1000000)

#define DSHOT600_HZ     		MHZ_TO_HZ(12)
#define DSHOT300_HZ     		MHZ_TO_HZ(6)
#define DSHOT150_HZ     		MHZ_TO_HZ(3)

#define MOTOR_BIT_0            	7
#define MOTOR_BIT_1            	14
#define MOTOR_BITLENGTH        	20-1

#define DSHOT_FRAME_SIZE       	16

#define DSHOT_MIN_THROTTLE      48
#define DSHOT_MAX_THROTTLE     	2047
#define DSHOT_RANGE 			(DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE)



#include <motor/DShot.h>

uint8_t DShot::s_ChIdx = 0;
uint16_t DShot::m_DmaID[4];

DShot::DShot() {
	// TODO Auto-generated constructor stub

}

DShot::~DShot() {
	// TODO Auto-generated destructor stub
}

void DShot::init(TIM_HandleTypeDef* htim, Motor_Channel_t channel, eDshot_Type DShotType, uint32_t clockFreq) {
	m_Tmr = htim;
	em_DshotType = DShotType;
	m_TmrFreq = clockFreq;

	getChannel(channel);
	setTimer(em_DshotType);
	registerTcCallbackFunction();
	startPwm();
}

void DShot::write(uint16_t motor_value) {
	uint16_t DShotMotorValue = (uint16_t)(DSHOT_MIN_THROTTLE + (motor_value * DSHOT_RANGE / 100));
	loadDmaBuffer(DShotMotorValue);
	dmaStart();
	enableDmaRequest();
}

uint32_t DShot::getType(eDshot_Type dshot_type) {
	switch (dshot_type)
	{
		case(DSHOT600):
				return DSHOT600_HZ;

		case(DSHOT300):
				return DSHOT300_HZ;

		default:
		case(DSHOT150):
				return DSHOT150_HZ;
	}
}

void DShot::setTimer(eDshot_Type dshot_type) {
	// Calculate prescaler by dshot type
	uint16_t dshot_prescaler = lrintf((float) m_TmrFreq / getType(dshot_type) + 0.01f) - 1;

	// motor1
	__HAL_TIM_SET_PRESCALER(m_Tmr, dshot_prescaler);
	__HAL_TIM_SET_AUTORELOAD(m_Tmr, MOTOR_BITLENGTH);
}

void DShot::dmaTcCallback(DMA_HandleTypeDef *hdma) {
	TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

	for(uint16_t id = 0; id < 4; id++) {
		if (hdma == htim->hdma[m_DmaID[id]])
		{
			__HAL_TIM_DISABLE_DMA(htim, m_DmaID[id]);
//			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
			break;
		}
	}
}

void DShot::registerTcCallbackFunction() {
	m_Tmr->hdma[m_DmaID[m_ChIdx]]->XferCpltCallback = dmaTcCallback;
}

void DShot::startPwm() {
	HAL_TIM_PWM_Start(m_Tmr, m_Channel);
}

uint16_t DShot::getPacket(uint16_t value) {
	uint16_t packet;
	bool dshot_telemetry = false;

	packet = (value << 1) | (dshot_telemetry ? 1 : 0);

	// compute checksum
	unsigned csum = 0;
	unsigned csum_data = packet;

	for(int i = 0; i < 3; i++)
	{
        csum ^=  csum_data; // xor data by nibbles
        csum_data >>= 4;
	}

	csum &= 0xf;
	packet = (packet << 4) | csum;

	return packet;
}

void DShot::loadDmaBuffer(uint16_t value) {

	uint16_t packet = getPacket(value);

	for(int i = 0; i < 16; i++)
	{
		motor_dmabuffer[i] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;
		packet <<= 1;
	}

	motor_dmabuffer[16] = 0;
	motor_dmabuffer[17] = 0;
}

void DShot::prepare_dmabuffer_all() {

}

void DShot::dmaStart() {
	HAL_DMA_Start_IT(m_Tmr->hdma[m_DmaID[m_ChIdx]], (uint32_t)motor_dmabuffer, (uint32_t)m_tmrCCR, DSHOT_DMA_BUFFER_SIZE);
}

void DShot::enableDmaRequest() {
	__HAL_TIM_ENABLE_DMA(m_Tmr, m_dmaCC);
}

void DShot::getChannel(Motor_Channel_t channel) {
	switch (channel) {
		case 1:
			m_Channel =  TIM_CHANNEL_1;
			m_DmaID[s_ChIdx] = TIM_DMA_ID_CC1;
			m_tmrCCR = &(m_Tmr->Instance->CCR1);
			m_dmaCC = TIM_DMA_CC1;
			break;
		case 2:
			m_Channel = TIM_CHANNEL_2;
			m_DmaID[s_ChIdx] = TIM_DMA_ID_CC2;
			m_tmrCCR = &(m_Tmr->Instance->CCR2);
			m_dmaCC = TIM_DMA_CC2;
			break;
		case 3:
			m_Channel = TIM_CHANNEL_3;
			m_DmaID[s_ChIdx] = TIM_DMA_ID_CC3;
			m_tmrCCR = &(m_Tmr->Instance->CCR3);

			m_dmaCC = TIM_DMA_CC3;
			break;
		case 4:
			m_Channel = TIM_CHANNEL_4;
			m_DmaID[s_ChIdx] = TIM_DMA_ID_CC4;
			m_tmrCCR = &(m_Tmr->Instance->CCR4);
			m_dmaCC = TIM_DMA_CC4;
			break;
	}
	m_ChIdx = s_ChIdx;
	s_ChIdx++;
}


