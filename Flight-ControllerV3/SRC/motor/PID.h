/*
 * PID.h
 *
 *  Created on: Jul 10, 2024
 *      Author: Ikenna
 */

#ifndef FLIGHT_PID_H_
#define FLIGHT_PID_H_

#include <TimeTick.h>
#include <typedefs.h>


typedef enum {
	PID_ROLL,
	PID_PITCH,
	PID_YAW,
	PID_THROTTLE
}PID_Type;

typedef enum {
	CONTROL_POS,
	CONTROL_RATE,
	CONTROL_VEL,
}CNTRL_Type;

typedef enum {
	MODE_S_LOOP,
	MODE_D_LOOP
}CNTRL_Mode;



class PID {
public:
	PID();
	PID(float kp, float ki, float kd) : m_Kp(kp), m_Ki(ki), m_Kd(kd){}
	virtual ~PID();

	inline void setGains(float kp, float ki, float kd) {
		m_Kp = kp;
		m_Ki = ki;
		m_Kd = kd;
	}

	inline void updateSetpoint(float newSetpoint, PID_Type pidType = PID_ROLL) {
		if(pidType == PID_THROTTLE) {
			m_setPoint = newSetpoint/OPTICAL_FLOW_MAX_HEIGHT * 100;;
		}
		else {
			m_setPoint = 2*(newSetpoint - 50);
		}
	}

	inline void setLimits(float lowLimit, float highLimit) {
		m_lowLimit = lowLimit;
		m_highLimit = highLimit;
	}

	inline void setLowLimits(float lowLimit) {
		m_lowLimit = lowLimit;
	}

	inline void setHighLimits(float highLimit) {
		m_highLimit = highLimit;
	}

	inline void reset() {
		m_lastTime = TimeTick::getTimeUs();
		m_integral = 0;
		m_lastError = 0;
	}

	inline Pid_Gains getGains() {
		Pid_Gains gains = {m_Kp, m_Ki, m_Kd};
		return gains;
	}

	inline void setGains(Pid_Gains gains) {
		m_Kp = MAX(0,gains.Kp);
		m_Ki = MAX(0,gains.Ki);
		m_Kd = MAX(0,gains.Kd);
	}

//	inline float pidOutput(float x) {
//		return (x - m_lowLimit) * 100 / (m_highLimit - m_lowLimit);
//	}

	float run(float pos, float rate, CNTRL_Type rateControl = CONTROL_POS, PID_Type pidType = PID_ROLL, CNTRL_Mode mode = MODE_D_LOOP, float dT = 0.0021);

private:
	float m_Kp = 0.8, m_Ki = 0.2, m_Kd = 0;
	float m_setPoint = 0;
	float m_output;
	float m_lowLimit = -100, m_highLimit = 100;
	float m_backCalculationGain = 0.1;
	timetick_us m_lastTime;
	float m_lastError;
	float m_integral = 0;
	float m_lastOutput = 0;
	float m_lastRatePosition = 0;
	bool m_lastControlType = true;
	bool m_lastPosition = 0;
};

#endif /* FLIGHT_PID_H_ */
