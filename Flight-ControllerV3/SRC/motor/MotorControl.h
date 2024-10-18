/*
 * MotorControl.h
 *
 *  Created on: Jul 10, 2024
 *      Author: Ikenna
 */

#ifndef MOTOR_MOTORCONTROL_H_
#define MOTOR_MOTORCONTROL_H_

#include <DShot.h>
#include <PID.h>
#include <typedefs.h>
#include <AHRS.h>
#include <OpticalFlow.h>
#include <scheduler/Task.h>
#include <string.h>

typedef enum {
	M1,
	M2,
	M3,
	M4
}eMotor;

typedef struct {
	DShot instance;
	float speed = 0;
}sMotor;


typedef struct  __attribute__ ((packed)) {
	float M1 = 0;
	float M2 = 0;
	float M3 = 0;
	float M4 = 0;
}M_Speed;

typedef struct  __attribute__ ((packed)) {
	volatile float roll = 0;
	volatile float pitch = 0;
	volatile float throttle = 0;
	volatile float yaw = 0;
	volatile int16_t SL1 = 0;
	volatile int16_t SL2 = 2000;
	volatile int16_t SR2 = 0;
	volatile int16_t SR1 = 0;
}Channel;


typedef struct __attribute__ ((packed)){
	M_Speed mSpeed;
	Pid_Gains roll;
	Pid_Gains pitch;
	Pid_Gains yaw;
	Channel chState;
}ControlLog;

typedef struct __attribute__ ((packed)){
	M_Speed mSpeed;
	Pid_Gains roll;
	Pid_Gains pitch;
	Pid_Gains yaw;
	Channel chState;
	Pid_Vals pid;
}ControlLog_t;

typedef enum {
	ROLL,
	PITCH,
	YAW,
	eNUM_AXIS,
}eAxis;



class MotorControl: public Task {
public:
	MotorControl(AHRS *ahrs, OpticalFlow* optflw, Channel* rxCh);
	virtual ~MotorControl();

	void init(TIM_HandleTypeDef* htim);

	void armMotors(bool shouldArm);

	void setMotorSpeed(eMotor m, uint8_t speed);

	void setMotorsSpeed(float sM1, float sM2, float sM3, float sM4);

	inline ControlLog getLog() {
		Channel channel;
		memcpy(&channel, m_rxCh, sizeof(Channel));
		ControlLog log = {
//				.mSpeed = {.M1 = m_motor[0].speed, .M2 = m_motor[1].speed, .M3 = m_motor[2].speed, .M4 = tPID},//m_motor[3].speed},
				.mSpeed = {.M1 = m_motor[0].speed, .M2 = m_motor[1].speed, .M3 = m_motor[2].speed, .M4 = m_motor[3].speed},
				.roll = getPID(ROLL),
				.pitch = getPID(PITCH),
				.yaw = getPID(YAW),
				.chState = channel
		};
		return log;
	}

	inline ControlLog_t getBBxLog() {
		Channel channel;
		memcpy(&channel, m_rxCh, sizeof(Channel));
		ControlLog_t log = {
				.mSpeed = {.M1 = m_motor[0].speed, .M2 = m_motor[1].speed, .M3 = m_motor[2].speed, .M4 = m_motor[3].speed},
				.roll = getPID(ROLL),
				.pitch = getPID(PITCH),
				.yaw = getPID(YAW),
				.chState = channel,
				.pid = m_pidVals,
		};
		return log;
	}

	inline float getCurrentHeight(float height) {
		float currentHeight = 100 * height / OPTICAL_FLOW_MAX_HEIGHT;
		return currentHeight;
	}

	void increamentPID(eAxis axis, float p, float i, float d);

	void setPIDGains(eAxis axis, float p, float i, float d) {
		m_pid[axis].setGains(p, i, d);
	}

	inline Pid_Gains getPID(eAxis axis) {
		return m_pid[axis].getGains();
	}

	inline bool isArmed(){
		return m_isArmed;
	}

	float scaleAngle(float angle, eAxis axis);

	float scaleAngleRate(float rate, eAxis axis);

	Vector_t<float> getLocalPos(Vector_t<float> wPos, float yaw);

	float scalePid(float pid);

//	sMotor* run(Attitude currentAttitude,  Channel* rxCh, timetick_us currentTime);
	void run(Attitude currentAttitude,  Channel* rxCh, timetick_us currentTime);

	virtual void taskFunc(timetick_us currenTimeUs);

private :
	AHRS* m_ahrs;
	OpticalFlow* m_optflw;
	Channel* m_rxCh;

	sMotor m_motor[4];
	PID m_pid[eNUM_AXIS];
	PID m_throttlePid;
	PID m_xPosPID, m_yPosPID;

	bool m_isArmed = false;
	float tPID = 0;
	Pid_Vals m_pidVals;
};

#endif /* MOTOR_MOTORCONTROL_H_ */
