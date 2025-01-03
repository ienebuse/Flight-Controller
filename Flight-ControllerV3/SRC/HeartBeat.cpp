/*
 * HeartBeat.cpp
 *
 *  Created on: Aug 8, 2024
 *      Author: Ikenna
 */

#include <HeartBeat.h>
#include <gpio.h>
#include <AltitudeFilter.h>

HeartBeat::HeartBeat() {
	// TODO Auto-generated constructor stub

}

HeartBeat::~HeartBeat() {
	// TODO Auto-generated destructor stub
}

void HeartBeat::taskFunc(timetick_us currenTimeUs) {
	static uint8_t count = 0;

	AltitudeFilter::getAltitude();

	if(count++ == 10) {
		HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
		count = 0;
	}
}

