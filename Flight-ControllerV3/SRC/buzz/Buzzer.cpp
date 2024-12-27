/*
 * Buzzer.cpp
 *
 *  Created on: Dec 12, 2024
 *      Author: Ikenna
 */

#include <buzz/Buzzer.h>
#include <Application.h>

Buzzer* Buzzer::mp_instance;

Buzzer::Buzzer() {
	// TODO Auto-generated constructor stub
}

Buzzer::~Buzzer() {
	// TODO Auto-generated destructor stub
}

void Buzzer::init() {
	mp_instance = this;
}

void Buzzer::buzz(timetick_us period) {
	m_endTime  = TimeTick::getTimeUs() + period;
	m_buzz = true;
}

void Buzzer::taskFunc(timetick_us currenTimeUs) {
	if(m_buzz) {
		Application::buzzerOn();
		m_buzz = false;
		m_buzzing = true;
	}

	if(m_buzzing) {
		if(currenTimeUs >= m_endTime) {
			Application::buzzerOff();
			m_buzzing = false;
		}
	}
}

