/*
 * config.h
 *
 *  Created on: Nov 8, 2024
 *      Author: Ikenna
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include "typedefs.h"

typedef struct __attribute__((packed)) {
  float rp;
  float p;
  float i;
  float d;
}PidParam_t;

typedef struct __attribute__((packed)) {
  PidParam_t roll = {ROLL_PID_rKp, ROLL_PID_Kp, ROLL_PID_Ki, ROLL_PID_Kd};
  PidParam_t pitch = {PITCH_PID_rKp, PITCH_PID_Kp, PITCH_PID_Ki, PITCH_PID_Kd};
  PidParam_t yaw = {YAW_PID_rKp, YAW_PID_Kp, YAW_PID_Ki, YAW_PID_Kd};
  PidParam_t px = {POS_PID_rKp, POS_PID_Kp, POS_PID_Ki, POS_PID_Kd};
  PidParam_t py = {POS_PID_rKp, POS_PID_Kp, POS_PID_Ki, POS_PID_Kd};
  PidParam_t alt = {THROTTLE_PID_rKp, THROTTLE_PID_Kp, THROTTLE_PID_Ki, THROTTLE_PID_Kd};
}Pid;

typedef struct __attribute__((packed)) {
	float MotorLimit 			= MOTOR_LIMIT_SCALE;
	uint8_t MotorIdleLimit 		= MOTOR_IDLE_STATE;

//	uint32_t ImuPeriod 			= IMU_SAMPLING_PERIOD_US;
//	uint32_t FlightCtrlPeriod	= FLIGHT_CONTROL_PERIOD_US;
//	uint32_t OptFlwPeriod		= OPT_FLW_PERIOD_US;
//	uint32_t HeatbeatPeriod		= HEARTBEAT_PERIOD_US;
//	uint32_t MagPeriod			= MAG_ACQ_TIME_US;
//	uint32_t ConfigPeriod		= CONFIG_CHECK_PERIOD_US;
//	uint32_t BlackBoxPeriod		= BLACKBOX_UPDATEPERIOD_US;
//	uint32_t BarometerPeriod	= BAROMETER_PERIOD_US;
//	uint32_t ReceiverPriod		= RECEIVER_PERIOD_US;
//	uint32_t MeterPeriod		= METER_PERIOD_US;
//	uint32_t GpsPeriod			= GPS_PERIOD_US;
//	uint32_t LoggerPeriod		= LOGGER_PERIOD_US;

	uint16_t DecentRate			= DECENT_RATE_MMpS;
	uint8_t ThrottleSensitivity = THROTTLE_SENSITIVITY;

	bool UseMadgwick			= USE_MADGWICK;
	bool UseMahony				= USE_MAHONY;
	bool UseVQF					= USE_VQF;
	bool UseEKF					= USE_EKF;

	bool UseBarometer			= USE_BAROMETER;
	bool UseGPS					= USE_GPS;
	bool UseMagnetometer		= USE_MAGNETOMETER;

	float MagDeclination		= MAG_DECLINATION;
	uint8_t MagDevAddr			= MAG_DEV_ADDR;

	bool EnableLogging			= ENABLE_DEBUG;

	bool UseGyroFilter			= USE_GYRO_FILTER;
	bool UseAccelFilter			= USE_ACCEL_FILTER;

	bool OptFlwUseMSP			= OPTICAL_FLOW_USE_MSP;
	bool OptFlwUseMavLink		= OPTICAL_FLOW_USE_MAVLINK;
	bool OptFlwUseMicrolink		= OPTICAL_FLOW_USE_MICROLINK;
	bool OptFlwUseUPixel		= OPTICAL_FLOW_USE_UPIXEL;

	uint16_t OptFlwMaxVel		= OPTICAL_FLOW_MAX_VEL;
	uint16_t OptFlwMaxHeight	= OPTICAL_FLOW_MAX_HEIGHT;
	uint16_t OptFlwHoverHeight	= HOVER_HEIGHT;

	uint16_t MaxRate			= MAX_RATE;
	uint16_t MaxAngle			= MAX_ANGLE;
	uint16_t MaxAltRate			= MAX_ALT_RATE;
	uint16_t MaxYawAngle		= MAX_YAW_ANGLE;
	uint8_t MotorMin			= MOTOR_MIN;
	uint8_t MotorMax			= MOTOR_MAX;

//	PID_t Pid;

	float VoltScale				= VOLTAGE_MEASUREMENT_SCALE;
	float CurrentScale			= CURRENT_MEASUREMENT_SCALE;

	float SealLevel				= SEA_LEVEL_hPA;

	uint8_t NumCell				= BATTERY_TYPE_S;
}Settings;



typedef struct __attribute__((packed)) {
	Pid pid;
	Settings settings;
}Config;

typedef struct __attribute__((packed)) {
	uint32_t Header 		= 0xDEADFACE;
	uint8_t Cmd 			= 0xE8;
	uint8_t Len = sizeof(Config) + 1;
	Config config;
	uint8_t crc;
}Config_Packet;

typedef struct __attribute__((packed)) {
	uint32_t Header 		= 0xDEADFACE;
	uint8_t Len = sizeof(Pid) + 1;
	Pid pid;
	uint8_t crc;
}Pid_Packet;

typedef struct __attribute__((packed)) {
	uint32_t Header 		= 0xDEADFACE;
	uint8_t Len = sizeof(Settings) + 1;
	Settings settings;
	uint8_t crc;
}Settings_Packet;



#endif /* CONFIG_H_ */
