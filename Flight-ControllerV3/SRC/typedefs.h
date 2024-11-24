/*
 * vectordef.h
 *
 *  Created on: Feb 7, 2024
 *      Author: ienebuse
 */

#ifndef COMMON_VECTORDEF_H_
#define COMMON_VECTORDEF_H_

#include <stm32h7xx.h>
#include <stdint.h>
#include <math.h>
#include <I2CBus.h>

 #define MAX(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

 #define MIN(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

#define PROTOTYPE					0

#define RAD2DEG 					180/M_PI
#define DEG2RAD 					M_PI/180

#define ENABLE_MOTORS				1
#define MOTOR_LIMIT_SCALE			0.65f
#define MOTOR_MULTIPLIER			1.0f
#define MOTOR_IDLE_STATE			(uint8_t)10
#define ENABLE_DEBUG				1
#define PID_MAX_SCALE				20
#define THROTTLE_MAX_SCALE 			((MOTOR_LIMIT_SCALE * 100) - PID_MAX_SCALE) / 100

#define USE_MAGNETOMETER			1
#define MAG_DECLINATION				0.6667f
#if PROTOTYPE
	#define MAG_DEV_ADDR			(uint8_t)0x1E
#else
	#define MAG_DEV_ADDR			(uint8_t)0x0D
#endif

#define USE_BAROMETER				0
#define USE_BARO_CONT_UPDATE		0

#define USE_GPS						0

#define IMU_SAMPLING_PERIOD_US		(uint32_t)2100
#define FLIGHT_CONTROL_PERIOD_US	(uint32_t)5000
#if PROTOTYPE
	#define OPT_FLW_PERIOD_US		(uint32_t)20000
#else
	#define OPT_FLW_PERIOD_US		(uint32_t)20000
#endif
#define HEARTBEAT_PERIOD_US			(uint32_t)500000

#define CONFIG_CHECK_PERIOD_US		(uint32_t)100000
#define BLACKBOX_UPDATEPERIOD_US	(uint32_t)5000
#define BAROMETER_PERIOD_US			(uint32_t)100000
#define RECEIVER_PERIOD_US			(uint32_t)20000
#define METER_PERIOD_US				(uint32_t)1000000
#define GPS_PERIOD_US				(uint32_t)1000000
#define LOGGER_PERIOD_US			(uint32_t)30000


#if PROTOTYPE
	#define MAG_ACQ_TIME_US			(uint32_t)100000
#else
	#define MAG_ACQ_TIME_US			(uint32_t)20000
#endif

#define DECENT_RATE_MMpS			(uint16_t)200

#define USE_MADGWICK				0
#define USE_MAHONY					0
#define USE_EKF						0
#define USE_MADGWICK_FUSION			0
#define USE_VQF						1
#define USE_COMP					0


#define USE_GYRO_FILTER				1
#define USE_ACCEL_FILTER			1

#define USE_ACC_EST					0
#define USE_GYRO_BIAS				0
#define USE_VEL_EST					0

#if PROTOTYPE
	#define OPTICAL_FLOW_USE_MSP		0
	#define OPTICAL_FLOW_USE_MAVLINK	0
	#define OPTICAL_FLOW_USE_MICROLINK	1
#else
	#define OPTICAL_FLOW_USE_MSP		0
	#define OPTICAL_FLOW_USE_MAVLINK	0
	#define OPTICAL_FLOW_USE_MICROLINK	1
#endif

#define OPTICAL_FLOW_MAX_VEL		(uint16_t)700


#if (OPTICAL_FLOW_USE_MICROLINK == 1)
	#define HOVER_HEIGHT				(uint16_t)1000
	#define OPTICAL_FLOW_MAX_HEIGHT		(uint16_t)8000
#else
	#define HOVER_HEIGHT			(uint16_t)100
	#define OPTICAL_FLOW_MAX_HEIGHT	(uint16_t)800
#endif



#define THROTTLE_LIMIT	1
#define MAX_RATE					360.0f	// deg/s
#define MAX_ANGLE					45.0f	// deg
#define MAX_ALT_RATE				(uint16_t)1000 	// mm/s
#define MAX_ALT_RATE_SCALE			MAX_ALT_RATE * 100 / OPTICAL_FLOW_MAX_HEIGHT
#define MAX_YAW_ANGLE				180.0f
#define MOTOR_MIN					5.0f
#define MOTOR_MAX					45.0f

#define ROLL_PID_rKp				2.0f
#define ROLL_PID_Kp					0.7f
#define ROLL_PID_Ki					0.2f
#define ROLL_PID_Kd					0.0002f

#define PITCH_PID_rKp				2.0f
#define PITCH_PID_Kp				0.7f
#define PITCH_PID_Ki				0.2f
#define PITCH_PID_Kd				0.0002f

#define YAW_PID_rKp					2.0f
#define YAW_PID_Kp					0.6f
#define YAW_PID_Ki					0.2f
#define YAW_PID_Kd					0.0002f

#define THROTTLE_PID_rKp			0.0f
#define THROTTLE_PID_Kp				7.0f
#define THROTTLE_PID_Ki				0.08f
#define THROTTLE_PID_Kd				1.5f

#define THROTTLE_PID_Kp_Scale  		0.15f

#define POS_PID_Kp					0.075f
#define POS_PID_Ki					0.001f
#define POS_PID_Kd					0.05f
#define POS_PID_rKp					0.3f


#define VOLTAGE_MEASUREMENT_SCALE	14.46f
#define CURRENT_MEASUREMENT_SCALE	22.35f//170

#define SEA_LEVEL_hPA				101300.0f

#define BATTERY_TYPE_S				6

#if BATTERY_TYPE_S == 6
	#define BATTERY_ALARM_THRESHOLD		21.0f
#elif BATTERY_TYPE_S == 4
	#define BATTERY_ALARM_THRESHOLD		14.0f
#else
	#define BATTERY_ALARM_THRESHOLD		10.5f
#endif
#define BATTERY_ALARM_PERIOD_US		(uint32_t)1000000




#define THROTTLE_MAX		192.0f
#define THROTTLE_MIN		1792.0f
#define THROTTLE_RANGE 		THROTTLE_MAX - THROTTLE_MIN

#define SW_MIN		192.0f
#define SW_MAX		1792.0f
#define SW_RANGE 		SW_MAX - SW_MIN


#define RPY_MAX_ANGLE		45.0f
#define RPY_MIN_ANGLE		-45.0f
#define RPY_MAX				1552.0f
#define RPY_MIN				432.0f
#define RPY_RANGE 			RPY_MAX - RPY_MIN

#define R_MAX_ANGLE			45.0f
#define R_MIN_ANGLE			-45.0f
#define R_MIN				1552.0f
#define R_MAX				432.0f
#define R_RANGE 			RPY_MAX - RPY_MIN



#define THROTTLE(t) ((t-THROTTLE_MIN)*100/(THROTTLE_RANGE))
#define RPY(rpy) ((rpy-RPY_MIN)*100/(RPY_RANGE))	//(((rpy-RPY_MIN)*(RPY_MAX_ANGLE-RPY_MIN_ANGLE)/(RPY_RANGE)) + RPY_MIN_ANGLE)
#define RTF(r) ((r-R_MIN)*100/(R_RANGE))
#define SW(t) ((t-SW_MIN)*100/(SW_RANGE))



const float pi = 3.141592653589793238462643383279502884f;
//float GyroMeasError = pi * (1.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
//float GyroMeasDrift = pi * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
//float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;   // compute beta
//float zeta = sqrtf(3.0f / 4.0f) * GyroMeasDrift;


typedef struct {
	float Kp = 0.0010f; // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
	float Ki = 0.0001f;
	float GyroMeasError = pi * (1.0f  / 180.0f);

	float gyroFc = 0.5;
	float gyroFs = 500;
}Filter_setup;

const Filter_setup cFilterSetup;

template <typename T>
struct __attribute__((packed)) Vector_t
{
    T x = 0;
    T y = 0;
    T z = 0;
    void reset() {x=0;y=0;z=0;}
    T sum() {return x+y+z;}
};

typedef struct {
    float q0=1, q1=0, q2=0, q3=0;
}Quaternion_t;


typedef struct  __attribute__ ((packed))
{
	float r = 0;
	float p = 0;
	float y = 0;
}Euler;
//
typedef struct __attribute__ ((packed))
{
	float q0 = 1;
	float q1 = 0;
	float q2 = 0;
	float q3 = 0;
}Quat;

typedef struct __attribute__ ((packed)) {
	Euler euler;
	Quat quat;
	Vector_t<float> gyro;
	Vector_t<float> accel;
	Vector_t<float> vel;
	Vector_t<float> acc;
}Attitude;

typedef struct {
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *csPort;
	uint16_t csPin;
	uint32_t SPI_HS_CLK;
}SPI_Config;

typedef struct I2C_config{
	I2C_Bus* i2cBus;
	devAddr_t devAddr;
}I2C_config_t;

typedef struct __attribute__ ((packed)){
	float Kp = 0;
	float Ki = 0;
	float Kd = 0;
}Pid_Gains ;

typedef struct __attribute__ ((packed)){
	float pos_x = 0;
	float pos_y = 0;
	float roll = 0;
	float pitch = 0;
	float yaw = 0;
	float throttle = 0;
}Pid_Vals ;

typedef struct __attribute__ ((packed)) CompassData {
	Vector_t<float> mag;
	float heading;
}CompassData_t;




#endif /* COMMON_VECTORDEF_H_ */
