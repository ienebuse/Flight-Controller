/*
 * MotorControl.cpp
 *
 *  Created on: Jul 10, 2024
 *      Author: Ikenna
 */

#include <motor/MotorControl.h>
#include <TimeTick.h>
#include <stdlib.h>

static float motorConstraint(float m) {
	return MIN(MOTOR_MAX,MAX(MOTOR_MIN,m));
}

MotorControl::MotorControl(AHRS *ahrs, OpticalFlow* optflw,  Channel* rxCh) : m_ahrs(ahrs), m_optflw(optflw), m_rxCh(rxCh) {
	// TODO Auto-generated constructor stub

}

MotorControl::~MotorControl() {
	// TODO Auto-generated destructor stub
}

void MotorControl::init(TIM_HandleTypeDef* htim) {
	m_pid[ROLL].setGains(ROLL_PID_Kp, ROLL_PID_ki, ROLL_PID_kd);
	m_pid[PITCH].setGains(PITCH_PID_Kp, PITCH_PID_ki, PITCH_PID_kd);
	m_pid[YAW].setGains(YAW_PID_Kp, YAW_PID_ki, YAW_PID_kd);

	m_throttlePid.setGains(THROTTLE_PID_Kp, THROTTLE_PID_Ki, THROTTLE_PID_Kd);
	m_throttlePid.setLimits(0,100);

	m_xPosPID.setLimits(-100,100);
	m_xPosPID.setGains(POS_PID_Kp, POS_PID_Ki, POS_PID_Kd);
	m_yPosPID.setLimits(-100,100);
	m_yPosPID.setGains(POS_PID_Kp, POS_PID_Ki, POS_PID_Kd);

	for(uint8_t m = 0; m < 4; m++) {
		m_motor[m].instance.init(htim, m+1, DSHOT600);
	}
}

void MotorControl::armMotors(bool shouldArm) {

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
	}
	else {
		m_isArmed = false;
	}
}


void MotorControl::setMotorSpeed(eMotor m, uint8_t speed) {
	m_motor[m].speed = speed;
}

void MotorControl::increamentPID(eAxis axis, float p, float i, float d) {
	Pid_Gains gains = m_pid[axis].getGains();
	gains.Kp += p;
	gains.Ki += i;
	gains.Kd += d;
	m_pid[axis].setGains(gains);
}

void MotorControl::setMotorsSpeed(float sM1, float sM2, float sM3, float sM4) {
	m_motor[0].speed = sM1;
	m_motor[1].speed = sM2;
	m_motor[2].speed = sM3;
	m_motor[3].speed = sM4;

#if ENABLE_MOTORS
	for(uint8_t m = 0; m < 4; m++) {
		m_motor[m].instance.write(lrintf(m_motor[m].speed));
//		m_motor[m].instance.write(2);
	}
#endif
}

float MotorControl::scaleAngle(float angle, eAxis axis) {
	float angleScale = 0;
	if(axis == YAW) {
		angleScale = ((angle + MAX_YAW_POS)*200/(2*MAX_YAW_POS)) - 100;
	}
	else {
		angleScale = ((angle + MAX_POS)*200/(2*MAX_POS)) - 100;
	}
	return angleScale;
}

float MotorControl::scaleAngleRate(float rate, eAxis axis) {
	float rateScale = 0;
	rateScale = ((-rate + MAX_RATE)*200/(2*MAX_RATE)) - 100;
	return rateScale;
}

//float MotorControl::scaleVel(float rate, eAxis axis) {
//	float rateScale = 0;
//	rateScale = ((-rate + MAX_RATE)*200/(2*MAX_RATE)) - 100;
//	return rateScale;
//}

Vector_t<float> MotorControl::getLocalPos(Vector_t<float> wPos, float yaw) {
	Vector_t<float> lPos;

	lPos.x = wPos.x*cos(yaw) + wPos.y*sin(yaw);
	lPos.y = -wPos.x*sin(yaw) + wPos.y*cos(yaw);

	return lPos;
}

//sMotor* MotorControl::run(Attitude currentAttitude,  Channel* rxCh, timetick_us currentTime) {
void MotorControl::run(Attitude currentAttitude,  Channel* rxCh, timetick_us currentTime) {
	static float heightSetpoint = HOVER_HEIGHT;
	static timetick_us lastTime = 0;
	static float lastHeight = 0;
	static float heightRate = 0;
	static float throttle = 0;
	float dT = (float)(currentTime - lastTime)/1000000;
	static float lastHeightControl = 0;
	float heightControl_dT = 0;
	bool HOVER = false;
	float rollAngle;
	float pitchAngle;


	if(m_isArmed) {
		float m1=0,m2=0,m3=0,m4=0;
		CNTRL_Type controlType = CONTROL_POS;

		OptFlw_Data optFlwData = m_optflw->getOptFlowData();
		float height = getCurrentHeight(optFlwData.h);
		if(currentTime - lastHeightControl > 200000) {
//			heightControl_dT = (float)(currentTime - lastHeightControl)/1000000;
			heightRate = ((optFlwData.h - lastHeight)/0.2) * 100 /MAX_ALT_RATE;
			lastHeight = optFlwData.h;
			lastHeightControl = currentTime;
		}

		m_pid[ROLL].updateSetpoint(100-rxCh->roll);
		m_pid[PITCH].updateSetpoint(rxCh->pitch);
		m_pid[YAW].updateSetpoint(rxCh->yaw);

		if(rxCh->SR1 > 60 && optFlwData.h > 250 && currentTime - lastTime > 1000000) {
			//Decend gradually
			heightSetpoint = optFlwData.h - 100;
			m_throttlePid.updateSetpoint(heightSetpoint, PID_THROTTLE);
			lastTime = currentTime;
		}
		else if(rxCh->SR1 > 60 && optFlwData.h < 250) {
			return;
		}
		else if(rxCh->SR1 < 40 && rxCh->pitch == 50 && rxCh->roll == 50) {
			// HOVER CONDITION
			if(optFlwData.h > 200) {
				HOVER = true;
			}
			controlType = CONTROL_POS;
			m_throttlePid.updateSetpoint(HOVER_HEIGHT, PID_THROTTLE);
		}
		else if(rxCh->SR1 > 40 && rxCh->SR1 < 60) {
			controlType = CONTROL_RATE;
//			m_throttlePid.updateSetpoint(rxCh->throttle, PID_THROTTLE);
			m_throttlePid.updateSetpoint(HOVER_HEIGHT, PID_THROTTLE);
		}
		else {
			controlType = CONTROL_POS;
//			m_throttlePid.updateSetpoint(rxCh->throttle, PID_THROTTLE);
			m_throttlePid.updateSetpoint(HOVER_HEIGHT, PID_THROTTLE);
		}


		if(HOVER) {
			rollAngle = m_yPosPID.run(optFlwData.py, optFlwData.vy, controlType, PID_ROLL, MODE_D_LOOP, dT);
			pitchAngle = m_xPosPID.run(optFlwData.px, optFlwData.vx, controlType, PID_PITCH, MODE_D_LOOP, dT);
		}
		else {
			rollAngle = scaleAngle(currentAttitude.euler.r, ROLL);
			pitchAngle = scaleAngle(currentAttitude.euler.p, PITCH);
		}

		float rollRate = scaleAngleRate(currentAttitude.gyro.x, ROLL);

		float pitchRate = scaleAngleRate(currentAttitude.gyro.y, PITCH);

		float yawAngle = scaleAngle(currentAttitude.euler.y, YAW);
		float yawRate = scaleAngleRate(-currentAttitude.gyro.z, YAW);




		float rollPid = m_pid[ROLL].run(rollAngle, rollRate, controlType, PID_ROLL, MODE_D_LOOP, dT);
		float pitchPid = m_pid[PITCH].run(pitchAngle, pitchRate, controlType, PID_PITCH, MODE_D_LOOP, dT);
		float yawPid = m_pid[YAW].run(yawAngle, yawRate, controlType, PID_YAW, MODE_D_LOOP, dT);

		if(currentTime - lastHeightControl > 20000) {
			throttle = m_throttlePid.run(height, -heightRate, CONTROL_POS, PID_THROTTLE, MODE_S_LOOP, dT);
			tPID = throttle;

		}
		/*
		 * 	M4				 M2
		 *  cw *          *  ccw
		 *        *     *
		 *           *
		 *        *     *
		 *     *           *
		 *	M3				 M1
		 *	ccw				cw
		*/


		m1 = rxCh->throttle - pitchPid + rollPid + yawPid;
		m2 = rxCh->throttle + pitchPid + rollPid - yawPid;
		m3 = rxCh->throttle - pitchPid - rollPid - yawPid;
		m4 = rxCh->throttle + pitchPid - rollPid + yawPid;

//		m1 = throttle - pitchPid + rollPid + yawPid;
//		m2 = throttle + pitchPid + rollPid - yawPid;
//		m3 = throttle - pitchPid - rollPid - yawPid;
//		m4 = throttle + pitchPid - rollPid + yawPid;

		m1 = motorConstraint(m1);
		m2 = motorConstraint(m2);
		m3 = motorConstraint(m3);
		m4 = motorConstraint(m4);

		if(rxCh->SR2 > 40) {
			setMotorsSpeed(10, 10, 10, 10);
		}
		else {
			setMotorsSpeed(m1*MOTOR_LIMIT_SCALE, m2*MOTOR_LIMIT_SCALE, m3*MOTOR_LIMIT_SCALE, m4*MOTOR_LIMIT_SCALE);
		}

		m_pidVals.throttle = throttle;
		m_pidVals.roll = rollPid;
		m_pidVals.pitch = pitchPid;
		m_pidVals.yaw = yawPid;
		m_pidVals.pos_y = rollAngle;
		m_pidVals.pos_x = pitchAngle;


//		return m_motor;
	}
}

void MotorControl::taskFunc(timetick_us currenTimeUs) {
	run(m_ahrs->getCurrentAttitude(), m_rxCh, currenTimeUs);

}
