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

#define RAD2DEG 					180/M_PI
#define DEG2RAD 					M_PI/180

#define ENABLE_MOTORS				1
#define MOTOR_LIMIT_SCALE			0.75
#define MOTOR_IDLE_SCALE			0.1
#define ENABLE_DEBUG				0

#define USE_MAGNETOMETER			1

#define USE_BAROMETER				0
#define USE_BARO_CONT_UPDATE		0

#define USE_GPS						0

#define IMU_SAMPLING_PERIOD_US		2100
#define FLIGHT_CONTROL_PERIOD_US	10000
#define MAG_DEV_ADDR	0x1E

#define MAG_DECLINATION				0.6667f

//#define USE_MADGWICK
//#define USE_MAHOHY
//#define USE_EKF
//#define USE_MADGWICK_FUSION
#define USE_VQF
//#define USE_COMP


#define USE_GYRO_FILTER				0
#define USE_ACCEL_FILTER			0

#define USE_ACC_EST					0
#define USE_GYRO_BIAS				0
#define USE_VEL_EST					0


#define OPTICAL_FLOW_USE_MSP		0
#define OPTICAL_FLOW_USE_MAVLINK	0
#define OPTICAL_FLOW_USE_MICROLINK	1

#define OPTICAL_FLOW_MAX_VEL		700


#if (OPTICAL_FLOW_USE_MICROLINK == 1)
	#define HOVER_HEIGHT			1000
#define OPTICAL_FLOW_MAX_HEIGHT		8000
#else
	#define HOVER_HEIGHT			100
	#define OPTICAL_FLOW_MAX_HEIGHT	800
#endif



#define THROTTLE_LIMIT	1
#define MAX_RATE					360.0f	// deg/s
#define MAX_POS						45.0f	// deg
#define MAX_ALT_RATE				3000 // mm/s
#define MAX_YAW_POS					180.0f
#define MOTOR_MIN						5.0f
#define MOTOR_MAX					100.0f

#define ROLL_PID_Kp					0.5f
#define ROLL_PID_ki					0.2f
#define ROLL_PID_kd					0.0f

//#define PITCH_PID_Kp				40.0f
//#define PITCH_PID_ki				30.0f
//#define PITCH_PID_kd				0.0f

#define PITCH_PID_Kp				0.5f
#define PITCH_PID_ki				0.2f
#define PITCH_PID_kd				0.0f

#define YAW_PID_Kp					0.5f
#define YAW_PID_ki					0.2f
#define YAW_PID_kd					0.0f

#define THROTTLE_PID_Kp				0.80f
#define THROTTLE_PID_Ki				1.50f
#define THROTTLE_PID_Kd				0.0f

#define THROTTLE_PID_Kp_Scale  		0.15f

#define POS_PID_Kp					0.05f
#define POS_PID_Ki					0.001f
#define POS_PID_Kd					0.000000f


#define VOLTAGE_MEASUREMENT_SCALE	14.46f
#define CURRENT_MEASUREMENT_SCALE	22.35f//170

#define SEA_LEVEL_hPA				101300





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




#endif /* COMMON_VECTORDEF_H_ */
