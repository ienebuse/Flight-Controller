/*
 * DShot.h
 *
 *  Created on: Jul 10, 2024
 *      Author: Ikenna
 */

#ifndef MOTOR_DSHOT_H_
#define MOTOR_DSHOT_H_

#include <tim.h>


#define DSHOT_DMA_BUFFER_SIZE   18 /* resolution + frame reset (2us) */

typedef uint32_t Motor_Channel_t;

/* Enumeration */
typedef enum
{
    DSHOT150,
    DSHOT300,
    DSHOT600
} eDshot_Type;

class DShot {
public:
	DShot();
	virtual ~DShot();

	void init(TIM_HandleTypeDef* htim, Motor_Channel_t channel, eDshot_Type DShotType, uint32_t clockFreq = 240000000);
	void write(uint16_t motor_value);

private:
	/* Variables */
	uint32_t motor_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
	static uint8_t s_ChIdx;
	uint8_t m_ChIdx;

	TIM_HandleTypeDef* m_Tmr;
	uint32_t m_Channel;
	static uint16_t m_DmaID[4];
	volatile uint32_t* m_tmrCCR;
	uint32_t m_dmaCC;
	uint32_t m_TmrFreq;
	eDshot_Type em_DshotType;



	uint32_t getType(eDshot_Type dshot_type);
	void setTimer(eDshot_Type dshot_type);
	static void dmaTcCallback(DMA_HandleTypeDef *hdma);
	void registerTcCallbackFunction();
	void startPwm();

	uint16_t getDmaID();


	uint16_t getPacket(uint16_t value);
	void loadDmaBuffer(uint16_t value);
	void prepare_dmabuffer_all();
	void dmaStart();
	void enableDmaRequest();
	void getChannel(Motor_Channel_t channel);



};

#endif /* MOTOR_DSHOT_H_ */
