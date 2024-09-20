/*
 * PID.cpp
 *
 *  Created on: Jul 10, 2024
 *      Author: Ikenna
 */

#include <motor/PID.h>

PID::PID() {
	// TODO Auto-generated constructor stub

}

PID::~PID() {
	// TODO Auto-generated destructor stub
}

static float fastSqrt(float number) {
    if (number <= 0) return 0;

    float x = number;
    float xhalf = 0.5f * x;

    int i = *(int*)&x; // Get bits for floating value
    i = 0x5f3759df - (i >> 1); // Initial guess for Newton's method
    x = *(float*)&i; // Convert bits back to float

    // Perform iterations of Newton's method to improve the estimate
    x = x * (1.5f - xhalf * x * x); // First iteration
    x = x * (1.5f - xhalf * x * x); // Second iteration (usually sufficient)

    return 1.0f / x;
}


float PID::run(float pos, float rate, CNTRL_Type controlType, PID_Type pidType, CNTRL_Mode mode, float dT) {
	float error;
	float dError = 0;
	float kpScale = 1;

	// Yaw should always be position control except when there is a command to move the yaw then it should go to rate control
	if(pidType == PID_YAW) {
		if(controlType != CONTROL_POS) {
			controlType = CONTROL_RATE;
		}
	}


	if(controlType == CONTROL_RATE && m_lastControlType != CONTROL_RATE) { // After switching to rate control, the initial position should be the last position of the last control method
		m_lastRatePosition = m_lastPosition;
	}

//	if(pidType == PID_THROTTLE) {	// Throttle PID uses only single loop position control
	if(mode == MODE_S_LOOP) {
		error = m_setPoint - pos;
		pos = 0.7*pos + 0.3*m_lastOutput;
		dError = (pos - m_lastOutput)/dT;
		m_lastOutput = pos;
	}
	else if(controlType == CONTROL_RATE && abs(m_setPoint) > 2) {
//	if(controlType == CONTROL_RATE && abs(m_setPoint) > 2) {
		// using a dead-band of 2 in the controller position for the rate control
		// rate control is only active when there is a pitch or roll command
		error = m_setPoint - rate;
		dError = (rate - m_lastOutput)/dT;
		m_lastOutput = rate;
		m_lastRatePosition = pos;
	}
	else {
		if(controlType == CONTROL_RATE) {
			// use position control rate control is active and no pitch or roll command is sent
			m_setPoint = m_lastRatePosition;
		}

		float eP = m_setPoint - pos;

		float rateSp = copysign(fastSqrt(abs(eP)),eP);

		error = rateSp - rate;
		dError = (pos - m_lastOutput)/dT;
		m_lastOutput = pos;
	}

	if(pidType == PID_THROTTLE) {
		kpScale = 1 - (pos / m_setPoint);
		kpScale = MAX(kpScale,THROTTLE_PID_Kp_Scale);
	}

	m_integral = (error + m_lastError) * dT/2;

	m_lastError = error;

	float pid = (kpScale*m_Kp * error) + (m_Ki * m_integral) + (m_Kd * dError);

	float _pid = pid;

	if(pid < m_lowLimit) {
		pid = m_lowLimit;
	}
	if(pid > m_highLimit) {
		pid = m_highLimit;
	}

	m_integral -= m_backCalculationGain * (pid - _pid);

//	m_lastTime = currentTime;
	m_lastOutput = pid;
	m_lastControlType = controlType;
	m_lastPosition = pos;

	return pid;
}

