/*
 * Buzzer.h
 *
 *  Created on: Dec 12, 2024
 *      Author: Ikenna
 */

#ifndef BUZZ_BUZZER_H_
#define BUZZ_BUZZER_H_

#include "Task.h"

class Buzzer : public Task {
public:
	Buzzer();
	virtual ~Buzzer();

	void init();

	inline static Buzzer* getInstance() {
		return mp_instance;
	}

	virtual void taskFunc(timetick_us currenTimeUs);

	void buzz(timetick_us period = 300000);

private:
	static Buzzer* mp_instance;
	timetick_us m_startTime{0}, m_endTime;
	bool m_buzzing{false}, m_buzz{false};



};

#endif /* BUZZ_BUZZER_H_ */
