/*
 * HeartBeat.h
 *
 *  Created on: Aug 8, 2024
 *      Author: Ikenna
 */

#ifndef HEARTBEAT_H_
#define HEARTBEAT_H_

#include <Task.h>

class HeartBeat : public Task {
public:
	HeartBeat();
	virtual ~HeartBeat();

	virtual void taskFunc(timetick_us currenTimeUs);
};

#endif /* HEARTBEAT_H_ */
