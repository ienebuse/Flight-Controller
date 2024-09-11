/*
 * Scheduler.h
 *
 *  Created on: Jun 30, 2024
 *      Author: Ikenna
 */

#ifndef SCHEDULER_SCHEDULER_H_
#define SCHEDULER_SCHEDULER_H_

#include <Task.h>

class Scheduler {
public:
	Scheduler();
	virtual ~Scheduler();

	void init();

	bool addTask(Task* task);

	bool queueContains(Task *task);

	Task* getTask(TaskId_t);

	Task* firstTask();

	Task* nextTask();

	void run();

	void executeTask(Task* selectedTask, timetick_us currentTime);

private:
	Task* taskQueue[NUM_TASK+1];	// +1 so that the end of the queue is null without going out-of-bounds
	uint8_t numTasks{0};
	uint8_t taskIndex{0};
};

#endif /* SCHEDULER_SCHEDULER_H_ */
