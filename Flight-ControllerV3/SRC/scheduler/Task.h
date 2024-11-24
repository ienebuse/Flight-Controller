/*
 * Task.h
 *
 *  Created on: Jun 30, 2024
 *      Author: Ikenna
 */

#ifndef SCHEDULER_TASK_H_
#define SCHEDULER_TASK_H_

#include <TimeTick.h>

#define PRIORITY_REALTIME		-1
#define PRIORITY_HIGH			5
#define PRIORITY_MEDIUM			10
#define PRIORITY_LOW			15

typedef enum {
	FUSION_TASK,
	FLIGHT_CONTROL_TASK,
	MAG_TASK,
	GPS_TASK,
	BAROMETER_TASK,
	OPTICAL_FLOW_TASK,
	CMD_RX_TASK,
	TELEMETRY_TASK,
	BLKBOX_TASK,
	CONFIG_TAX,
	OSD_TASK,
	LOG_TASK,
	HEARTBEAT_TASK,
	METER_TASK,
	NUM_TASK,
}TaskId_t;


class Task
{

public:

	uint8_t Id;
	int8_t staticPriority;
	int8_t priority;
	int8_t eventPriority;
	timetick_us period;
	timetick_us lastRunTime = 0;
	timetick_us nextRuntTime = 0;
	uint32_t runCount = 0;
	timetick_us waitTime = 0;
	float avgWaitTime = 0;
	volatile bool newEvent = false;

	void setTaskInfo(uint8_t id, int8_t sPrio, int8_t prio, int8_t evntPrio, timetick_us period) {
		this->Id = id;
		this->staticPriority = sPrio;
		this->priority = prio;
		this->eventPriority = evntPrio;
		this->period = period;
	}
	virtual void taskFunc(timetick_us currenTimeUs) = 0;

};
#endif /* SCHEDULER_TASK_H_ */
