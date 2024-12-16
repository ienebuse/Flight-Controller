/*
 * AHRS.h
 *
 *  Created on: Jun 29, 2024
 *      Author: Ikenna
 */

#ifndef AHRS_AHRS_H_
#define AHRS_AHRS_H_

#include <sensors/ICM42688.h>
#if PROTOTYPE
	#include <sensors/HMC5883L.h>
#else
	#include <sensors/QMC5883L.h>
#endif
#include <typedefs.h>
#include <fusion/ComplimentaryFilter.h>
#include <fusion/MadgwickFilter.h>
#include <fusion/MahonyFilter.h>
#include <fusion/VqfFilter.h>
//#include <fusion/EKF.h>
#include <I2CBus.h>
#include <Task.h>
#include <LowPassFilter.h>
#include <MadgwickFusion.h>


typedef struct __attribute__ ((packed)) {
	Euler euler = {0,0,0};
	float heading = 0;
	float dT = 0;
}AhrsLog;

typedef struct {
	float xN_2 = 0;
	float xN_1 = 0;
	float xN = 0;
	float yN_2 = 0;
	float yN_1 = 0;
}fParam;



class AHRS : public Task {
public:
	AHRS();
	virtual ~AHRS();

	typedef struct SensorData {
		Vector_t<float> gyro;
		Vector_t<float> acc;
		Vector_t<float> mag;
	}SensorData_t;

	void init(SPI_Config config1, SPI_Config config2, I2C_config mag_config);

	inline void reset() {
//		filter.reset();
	}

	inline Vector_t<float>  getAccel() {
		return m_sensorData.acc;
	}

	inline Vector_t<float> getGyro() {
		return m_sensorData.gyro;
	}

	void updateSensorData();

	SensorData getSensorData();

	float getHeading();

	Vector_t<float> getGroundAcc(Vector_t<float> acc);

	Attitude fushionUpdate(float dT);

	inline Attitude getCurrentAttitude() {
		return m_attitude;
	}

	inline AhrsLog getLog() {
		AhrsLog log = {.euler = m_attitude.euler, .heading = m_Heading, .dT = m_dT};
		return log;
	}

	virtual void taskFunc(timetick_us currenTimeUs);

private:

//	BMI270 m_imuSensor1;
	ICM42688 m_imuSensor1;
	ICM42688 m_imuSensor2;
#if PROTOTYPE
	HMC5883L m_compassSensor;
#else
	QMC5883L m_compassSensor;
#endif

	SensorData m_sensorData;
	Attitude m_attitude;
	float m_Heading;
    bool m_magAvailable{false};
    Vector_t<float>gyroOffset = {0,0,0};
    Vector_t<float>m_initialMag;

//    double ita =1.0/ tanf(M_PI * cFilterSetup.gyroFc / cFilterSetup.gyroFs);
//    double q=sqrt(2.0);
    double b0;
    double b1;
    double b2;
    double a1;
    double a2;

    float m_dT{0};

    LowPassFilter gxf, gyf, gzf;
    fParam fgx, fgy, fgz;

    double gyroMean = 0;    // Current mean
    double gyroM2 = 0;      // Sum of squares of differences from the current mean

    double accMean = 0;    // Current mean
	double accM2 = 0;

	double magMean = 0;    // Current mean
	double magM2 = 0;

#if USE_MAHONY
	Mahony filter;
#elif USE_MADGWICK
	Madgwick filter;
#elif USE_VQF
	VqfFilter filter;
#elif USE_COMP
	ComplimentaryFilter filter;
#elif USE_EKF
	EKF filter;
#elif USE_MADGWICK_FUSION
	MadgwickFusion filter;
#endif

	void calibrateGyro();

	void waitForSteadyGyro();

	void setCompassRef();

	Vector_t<float> getCompassRef();

	void updateVariance();

	void filterGyro();

	float _getHeading();

	Euler quat2Euler(Quat data);

	Quat euler2Quaternion(Euler data);
};

#endif /* AHRS_AHRS_H_ */
