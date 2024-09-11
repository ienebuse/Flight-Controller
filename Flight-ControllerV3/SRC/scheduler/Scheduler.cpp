/*
 * Scheduler.cpp
 *
 *  Created on: Jun 30, 2024
 *      Author: Ikenna
 */

#include <scheduler/Scheduler.h>
#include <cstring>

Scheduler::Scheduler() {
	// TODO Auto-generated constructor stub

}

Scheduler::~Scheduler() {
	// TODO Auto-generated destructor stub
}

void init() {

}

bool Scheduler::addTask(Task* task) {
	if(numTasks >= NUM_TASK || queueContains(task)) {
		return  false;
	}
	for(uint8_t tsk = 0; tsk <= numTasks; tsk++) {
		if((taskQueue[tsk] == nullptr) || (taskQueue[tsk]->priority > task->priority)) {
			memmove(&taskQueue[tsk+1], &taskQueue[tsk], sizeof(task) * (NUM_TASK-tsk));
			taskQueue[tsk] = task;
			numTasks++;
			return true;
		}
	}
	return false;
}

bool Scheduler::queueContains(Task *task)
{
    for (int tsk = 0; tsk < NUM_TASK; ++tsk) {
        if (taskQueue[tsk] == task) {
            return true;
        }
    }
    return false;
}

Task* Scheduler::getTask(TaskId_t taskId) {
	return taskQueue[taskId];
}

Task* Scheduler::firstTask() {
	taskIndex = 0;
	return taskQueue[0];
}

Task* Scheduler::nextTask() {
	return taskQueue[++taskIndex];
}

void Scheduler::executeTask(Task* selectedTask, timetick_us currentTime) {
	selectedTask->taskFunc(currentTime);
	selectedTask->nextRuntTime = currentTime + selectedTask->period;
	selectedTask->lastRunTime = currentTime;
	++selectedTask->runCount;
	selectedTask->avgWaitTime = (float)selectedTask->waitTime/selectedTask->runCount;
}

void Scheduler::run() {
	timetick_us currentTime = TimeTick::getTimeUs();
	Task* selectedTask = nullptr;
	timetick_us waitTime = 0;

	for(Task* task = firstTask(); task != nullptr; task = nextTask()) {
		if(selectedTask == nullptr) {
			if(currentTime >= task->nextRuntTime) {
				selectedTask = task;
				waitTime = currentTime - selectedTask->nextRuntTime;
			}
			continue;
		}
		// From this point selectedTask is guaranteed not to be null
		if(task->staticPriority == PRIORITY_REALTIME && currentTime >= task->nextRuntTime && selectedTask->staticPriority != PRIORITY_REALTIME) {
			selectedTask = task;
			waitTime = currentTime - selectedTask->nextRuntTime;
		}
		else if(currentTime >= task->nextRuntTime) {
			timetick_us taskWaitTime = currentTime - task->nextRuntTime;
			if(taskWaitTime > waitTime) {
				waitTime = taskWaitTime;
				selectedTask = task;
			}
			else if((taskWaitTime == waitTime) && (task->staticPriority > selectedTask->staticPriority)) {
				selectedTask = task;
			}
		}
	}

	if(selectedTask != nullptr) {
		selectedTask->waitTime += waitTime;
		executeTask(selectedTask, currentTime);
	}
}
