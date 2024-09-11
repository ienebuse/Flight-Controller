/*
 * TimeTick.cpp
 *
 *  Created on: Jul 1, 2024
 *      Author: IEnebuse
 */

#include <TimeTick.h>

TIM_HandleTypeDef* TimeTick::mp_tmr;
volatile timetick_us TimeTick::m_tickLow{0};

TimeTick::TimeTick() {
	// TODO Auto-generated constructor stub

}

TimeTick::~TimeTick() {
	// TODO Auto-generated destructor stub
}

