/*
 * MotorControl.cpp
 *
 *  Created on: Jul 10, 2024
 *      Author: Ikenna
 */

#include <FlightControl.h>
#include <TimeTick.h>
#include <stdlib.h>
#include <Configurator.h>
#include <config.h>

bool FlightControl::LAND = false;

static float motorConstraint(float m) {
	return MIN(Configurator::getConfig().settings.MotorMax,MAX(Configurator::getConfig().settings.MotorMin,m));
}

FlightControl::FlightControl(AHRS *ahrs, OpticalFlow* optflw,  Channel* rxCh) : m_ahrs(ahrs), m_optflw(optflw), m_rxCh(rxCh) {
	// TODO Auto-generated constructor stub

}

FlightControl::~FlightControl() {
	// TODO Auto-generated destructor stub
}

void FlightControl::init(TIM_HandleTypeDef* htim) {
	Pid pids = Configurator::getConfig().pid;

	m_pid[ROLL].setGains(pids.roll.p, pids.roll.i, pids.roll.d, pids.roll.rp);
	m_pid[PITCH].setGains(pids.pitch.p, pids.pitch.i, pids.pitch.d, pids.pitch.rp);
	m_pid[YAW].setGains(pids.yaw.p, pids.yaw.i, pids.yaw.d, pids.yaw.rp);

	m_pid[THROTTLE].setGains(pids.alt.p, pids.alt.i, pids.alt.d, pids.alt.rp);
	m_pid[THROTTLE].setLimits(0,100);

	m_pid[PX].setLimits(-20,20);
	m_pid[PX].setGains(pids.px.p, pids.px.i, pids.px.d, pids.px.rp);
	m_pid[PY].setLimits(-20,20);
	m_pid[PY].setGains(pids.py.p, pids.py.i, pids.py.d, pids.py.rp);

	for(uint8_t m = 0; m < 4; m++) {
		m_motor[m].instance.init(htim, m+1, DSHOT600);
	}
}

void FlightControl::armMotors(bool shouldArm) {

	if(m_isArmed == shouldArm) return;

	if(shouldArm) {
		TimeTick::delay_ms(1000);

		for(int i = 0; i < 1000; i++) {
			for(uint8_t m = 0; m < 4; m++) {
				m_motor[m].instance.write(0);
			}
			TimeTick::delay_ms(1);
		}

		for(int i = 0; i < 500; i++) {
			for(uint8_t m = 0; m < 4; m++) {
				m_motor[m].instance.write(10);
			}
			TimeTick::delay_ms(1);
		}

		for(int i = 0; i < 1000; i++) {
			for(uint8_t m = 0; m < 4; m++) {
				m_motor[m].instance.write(0);
			}
			TimeTick::delay_ms(1);
		}
		m_isArmed = true;
		LAND = false;
	}
	else {
		m_isArmed = false;
	}
}


void FlightControl::setMotorSpeed(eMotor m, uint8_t speed) {
	m_motor[m].speed = speed;
}

void FlightControl::increamentPID(eAxis axis, float p, float i, float d) {
	Pid_Gains gains = m_pid[axis].getGains();
	gains.Kp += p;
	gains.Ki += i;
	gains.Kd += d;
	m_pid[axis].setGains(gains);
}

void FlightControl::setMotorsSpeed(float speed) {
	setMotorsSpeed(speed, speed, speed, speed);
}

void FlightControl::setMotorsSpeed(float frontRight, float rearRight, float frontLeft, float rearLeft) {
	m_motor[0].speed = frontRight;
	m_motor[1].speed = rearRight;
	m_motor[2].speed = frontLeft;
	m_motor[3].speed = rearLeft;

#if ENABLE_MOTORS
	for(uint8_t m = 0; m < 4; m++) {
		m_motor[m].instance.write(lrintf(m_motor[m].speed));
	}
#endif
}

float FlightControl::scaleAngle(float angle, eAxis axis) {
	float angleScale = 0;
//	if(axis == YAW) {
//		angleScale = ((angle + Configurator::getConfig().settings.MaxYawAngle)*200/(2*Configurator::getConfig().settings.MaxYawAngle)) - 100;
//	}
//	else {
//		angleScale = ((angle + Configurator::getConfig().settings.MaxAngle)*200/(2*Configurator::getConfig().settings.MaxAngle)) - 100;
//	}

	if(axis == YAW) {
		angleScale = ((angle + MAX_YAW_ANGLE)*200/(2*MAX_YAW_ANGLE)) - 100;
	}
	else {
		angleScale = ((angle + MAX_ANGLE)*200/(2*MAX_ANGLE)) - 100;
	}
	return angleScale;
}

float FlightControl::scaleAngleRate(float rate, eAxis axis) {
	float rateScale = 0;
	rateScale = ((-rate + Configurator::getConfig().settings.MaxRate)*200/(2*Configurator::getConfig().settings.MaxRate)) - 100;
	return rateScale;
}

float FlightControl::getCurrentHeight(float height) {
	float currentHeight = 100 * height / Configurator::getConfig().settings.OptFlwMaxHeight;
	return currentHeight;
}

//float MotorControl::scaleVel(float rate, eAxis axis) {
//	float rateScale = 0;
//	rateScale = ((-rate + MAX_RATE)*200/(2*MAX_RATE)) - 100;
//	return rateScale;
//}

Vector_t<float> FlightControl::getLocalPos(Vector_t<float> wPos, float yaw) {
	Vector_t<float> lPos;

	lPos.x = wPos.x*cos(yaw) + wPos.y*sin(yaw);
	lPos.y = -wPos.x*sin(yaw) + wPos.y*cos(yaw);

	return lPos;
}

float FlightControl::scalePid(float pid) {
	float val = (pid + 100)*PID_MAX_SCALE/200;
	return val;
}

static float scaleMotor(float mVal, float min, float max) {
	float m = (mVal - min) * PID_MAX_SCALE / (max - min);
	return m;
}

void FlightControl::run(Attitude currentAttitude,  Channel* rxCh, timetick_us currentTime) {
	static float heightSetpoint = (float)Configurator::getConfig().settings.OptFlwHoverHeight;
	static timetick_us lastTime = 0;
	static float lastHeight = 0;
	static float heightRate = 0;
	float throttle = 0;
	float dT = (float)(FLIGHT_CONTROL_PERIOD_US)/1000000;
	static float lastHeightControl = 0;
	static float heightControl_dT = (float)OPT_FLW_PERIOD_US/1000000;
	bool HORIZONTAL_STAB = false;
	static float rollAngle, rollSetPoint;
	float pitchAngle, pitchSetPoint;
	static float throttleVal = 0, throttlePid = 0;
	bool idle = false;
	float rollPIDSetPoint = 0, pitchPIDSetPoint = 0;
	float rollPid = 0, pitchPid = 0, yawPid = 0;
	static float decentRate = (float)DECENT_RATE_MMpS * (float)FLIGHT_CONTROL_PERIOD_US / 1000000;

	static const float throttleScale = 0.001;
	static bool firstChange = true;


	if(m_isArmed) {
		float m1=0,m2=0,m3=0,m4=0;
		CNTRL_Type controlType = CONTROL_POS;
		OptFlw_Data optFlwData = m_optflw->getOptFlowData();


		if(rxCh->SR2 > 40 || rxCh->SR1 > 40 || rxCh->pitch != 50 || rxCh->roll != 50) {
			m_optflw->resetPos();
			m_pid[PX].reset();
			m_pid[PY].reset();
			firstChange = true;
		}
//		else if(firstChange) {
//			float xSP = optFlwData.vx * 0.5;
//			float ySP = optFlwData.vy * 0.5;
//
//			m_pid[PX].updateSetpoint(xSP, PID_XY);
//			m_pid[PY].updateSetpoint(ySP, PID_XY);
//
//			firstChange = false;
//		}

		if(rxCh->SR2 > 40) {
			m_pid[ROLL].reset();
			m_pid[PITCH].reset();
			m_pid[YAW].reset();
			m_pid[THROTTLE].reset();
		}


		float height = getCurrentHeight(optFlwData.h);

		heightRate = getCurrentHeight((optFlwData.h - lastHeight) / heightControl_dT);
		lastHeight = optFlwData.h;


		if((rxCh->SL1 < 40 && optFlwData.h > 150)  || Meter::batteryCritical()) {
			LAND = true;
		}

		if(rxCh->SR1 < 40) {
			if(rxCh->pitch == 50 && rxCh->roll == 50 && optFlwData.h > 150) {
				HORIZONTAL_STAB = true;
			}
			controlType = CONTROL_POS;
		}
		else if(rxCh->SR1 > 40 && rxCh->SR1 < 60) {
			controlType = CONTROL_POS;
		}
		else {
			controlType = CONTROL_RATE;
		}

		if(LAND) {
			if(optFlwData.h > 190) {
				heightSetpoint -= decentRate;
			}
			else if(optFlwData.h < 190) {
				idle = true;
			}
		}
		else {
			float throttleRate = 2*(rxCh->throttle - 50);
			if(rxCh->SR2 < 40) {
				heightSetpoint += throttleRate * (float)Configurator::getConfig().settings.ThrottleSensitivity * throttleScale;
			}
			else {
				heightSetpoint = (float)Configurator::getConfig().settings.OptFlwHoverHeight;
			}
		}

//		HOVER = true;

		bool HEIGHT_CONTROL = false;

		if(currentTime - lastHeightControl > 20000) {
			HEIGHT_CONTROL = true;
			lastHeightControl = currentTime;
		}

		if(HORIZONTAL_STAB) {
			rollPIDSetPoint = m_pid[PY].run(optFlwData.py, optFlwData.vy, CONTROL_POS, PID_POS, MODE_D_LOOP, dT);
			pitchPIDSetPoint = m_pid[PX].run(optFlwData.px, optFlwData.vx, CONTROL_POS, PID_POS, MODE_D_LOOP, dT);

			rollSetPoint = (rollPIDSetPoint - RPY_MIN_ANGLE) * 100 / (RPY_MAX_ANGLE - RPY_MIN_ANGLE);
			pitchSetPoint = (pitchPIDSetPoint - RPY_MIN_ANGLE) * 100 / (RPY_MAX_ANGLE - RPY_MIN_ANGLE);

			m_pid[ROLL].updateSetpoint(rollSetPoint, PID_POS);
			m_pid[PITCH].updateSetpoint(pitchSetPoint, PID_POS);
		}
		else {
			rollSetPoint = 100-rxCh->roll;
			pitchSetPoint = rxCh->pitch;

			m_pid[ROLL].updateSetpoint(rollSetPoint, PID_ROLL);
			m_pid[PITCH].updateSetpoint(pitchSetPoint, PID_PITCH);
		}

		m_pid[YAW].updateSetpoint(rxCh->yaw, PID_YAW);


		rollAngle = scaleAngle(currentAttitude.euler.r, ROLL);
		pitchAngle = scaleAngle(currentAttitude.euler.p, PITCH);

		float rollRate = scaleAngleRate(currentAttitude.gyro.x, ROLL);
		float pitchRate = scaleAngleRate(currentAttitude.gyro.y, PITCH);

		float yawAngle = scaleAngle(currentAttitude.euler.y, YAW);
		float yawRate = scaleAngleRate(-currentAttitude.gyro.z, YAW);

		if(rxCh->SR2 < 40) {
			rollPid = m_pid[ROLL].run(rollAngle, rollRate, controlType, PID_ROLL, MODE_D_LOOP, dT);
			pitchPid = m_pid[PITCH].run(pitchAngle, pitchRate, controlType, PID_PITCH, MODE_D_LOOP, dT);
			yawPid = m_pid[YAW].run(yawAngle, yawRate, CONTROL_RATE, PID_YAW, MODE_D_LOOP, dT);
		};

//		throttleVal = throttle;

		if(rxCh->SL1 < 60) {
			if(HEIGHT_CONTROL && rxCh->SR2 < 40) {
				m_pid[THROTTLE].updateSetpoint(heightSetpoint, PID_THROTTLE);
				throttle = m_pid[THROTTLE].run(height, heightRate, CONTROL_POS, PID_THROTTLE, MODE_S_LOOP, heightControl_dT);
				throttleVal = throttle;
				throttlePid = throttleVal;
			}
		}
//		else if(rxCh->SL1 > 60) {
//			throttleVal = rxCh->throttle;
//			throttle = 0;
//			m_pid[THROTTLE].reset();
//			throttlePid = 0;
//		}

		/*
		 * 	M4				 M2
		 *  cw *          *  ccw
		 *        *     *                     +Z <----
		 *           *                  +Y <-------   |
		 *        *     *                          |  |
		 *     *           *                       |
		 *	M3				 M1                    |
		 *	ccw				cw                     v +X
		*/

		m1 = throttleVal - pitchPid + rollPid + yawPid;
		m2 = throttleVal + pitchPid + rollPid - yawPid;
		m3 = throttleVal - pitchPid - rollPid - yawPid;
		m4 = throttleVal + pitchPid - rollPid + yawPid;

		m1 = motorConstraint(m1);
		m2 = motorConstraint(m2);
		m3 = motorConstraint(m3);
		m4 = motorConstraint(m4);

		if(rxCh->SR2 > 40 || idle || optFlwData.h > 500) {
			setMotorsSpeed(Configurator::getConfig().settings.MotorIdleLimit);
		}
		else {
#if PROTOTYPE
			setMotorsSpeed(m1*MOTOR_MULTIPLIER, m2*MOTOR_MULTIPLIER, m3*MOTOR_MULTIPLIER, m4*MOTOR_MULTIPLIER);
#else
			setMotorsSpeed(m2*MOTOR_MULTIPLIER, m1*MOTOR_MULTIPLIER, m4*MOTOR_MULTIPLIER, m3*MOTOR_MULTIPLIER);
#endif
		}

		m_pidVals.throttle = throttlePid;
		m_pidVals.roll = rollPid;
		m_pidVals.pitch = pitchPid;
		m_pidVals.yaw = yawPid;
		m_pidVals.pos_y = rollPIDSetPoint;
		m_pidVals.pos_x = pitchPIDSetPoint;
	}
}

void FlightControl::taskFunc(timetick_us currenTimeUs) {
	run(m_ahrs->getCurrentAttitude(), m_rxCh, currenTimeUs);

}
