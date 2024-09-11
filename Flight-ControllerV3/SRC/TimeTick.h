/*
 * TimeTick.h
 *
 *  Created on: Jul 1, 2024
 *      Author: IEnebuse
 */

#ifndef TIMETICK_H_
#define TIMETICK_H_
#include <main.h>

typedef uint32_t timetick_us;

class TimeTick {
public:
	TimeTick();
	virtual ~TimeTick();

	static inline void init(TIM_HandleTypeDef* tmr) {
		mp_tmr = tmr;
		HAL_TIM_Base_Start_IT(mp_tmr);
	}

	static inline timetick_us getTimeUs() {
		return __HAL_TIM_GET_COUNTER(mp_tmr);
	}

	static inline timetick_us getDeltaNow() {
		return 0;
	}

	static inline void delay_us(timetick_us delayUs) {
		volatile timetick_us tick_start = getTimeUs();
		while(getTimeUs() - tick_start < delayUs);
	}

	static inline void delay_ms(timetick_us delayMs) {
			delay_us(delayMs*1000);
		}

	static inline void rollover() {
//		++m_tickHighWordUs;
	}

private:
	static TIM_HandleTypeDef* mp_tmr;
	static volatile timetick_us  m_tickLow;

//	inline TIM_HandleTypeDef* getInstance() {
//		return this;
//	}
};

#endif /* TIMETICK_H_ */
