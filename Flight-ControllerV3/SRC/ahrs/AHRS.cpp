/*
 * AHRS.cpp
 *
 *  Created on: Jun 29, 2024
 *      Author: Ikenna
 */

#include <ahrs/AHRS.h>
#include <math.h>
#include <TimeTick.h>
#include <usart.h>
#include <string.h>
#include <stdio.h>
#include <Configurator.h>
#include <math.h>

#define _SensorData(f) -f.acc.x, -f.acc.y, f.acc.z, -f.gyro.x*DEG2RAD, -f.gyro.y*DEG2RAD, f.gyro.z*DEG2RAD, -f.mag.y, f.mag.x, f.mag.z
#define _SensorDataNoMag(f) -f.acc.x, -f.acc.y, f.acc.z, -f.gyro.x*DEG2RAD, -f.gyro.y*DEG2RAD, f.gyro.z*DEG2RAD

AHRS::AHRS() : filter(&m_attitude){
	// TODO Auto-generated constructor stub
//	double ita = 0.726542528005361;//1.0/ tan(M_PI * cFilterSetup.gyroFc / cFilterSetup.gyroFs);
//	double q=sqrt(2.0);
//	b0 = 1.0 / (1.0 + q*ita + ita*ita);
//	b1= 2*b0;
//	b2= b0;
//	a1 = 2.0 * (ita*ita - 1.0) * b0;
//	a2 = -(1.0 - q*ita + ita*ita) * b0;
}

AHRS::~AHRS() {
	// TODO Auto-generated destructor stub
}

void AHRS::calibrateGyro() {

	Vector_t<float> gyro = {0,0,0};
	uint16_t NUM_CAL_SAMPLE = 3000;

	for(int i = 0; i < 10; i++) {
		HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
		TimeTick::delay_us(100000);
	}

	TimeTick::delay_ms(3000);

	for(int i = 0; i < NUM_CAL_SAMPLE; i++) {
		Vector_t<float> gyro1 = m_imuSensor1.getGyroData();
		Vector_t<float> gyro2 = m_imuSensor2.getGyroData();

		gyro.x += (gyro1.x - gyro2.x)/2;
		gyro.y += (gyro1.y - gyro2.y)/2;
		gyro.z += (gyro1.z + gyro2.z)/2;

//		gyro.x += gyro2.x;
//		gyro.y += gyro2.y;
//		gyro.z += gyro2.z;

		TimeTick::delay_us(1300);
	}

	gyroOffset.x = gyro.x/NUM_CAL_SAMPLE;
	gyroOffset.y = gyro.y/NUM_CAL_SAMPLE;
	gyroOffset.z = gyro.z/NUM_CAL_SAMPLE;
}

void AHRS::setCompassRef() {
	uint16_t NUM_CAL_SAMPLE = 20;

	for(int i = 0; i < 10; i++) {
		HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
		TimeTick::delay_us(100000);
	}

//	TimeTick::delay_ms(3000);

	for(int i = 0; i < NUM_CAL_SAMPLE; i++) {
		Vector_t<float> mag = m_compassSensor.getCompass().mag;

		m_initialMag.x += -mag.y;
		m_initialMag.y += mag.x;
		m_initialMag.z += mag.z;

		TimeTick::delay_us(100000);
	}

	m_initialMag.x /= NUM_CAL_SAMPLE;
	m_initialMag.y /= NUM_CAL_SAMPLE;
	m_initialMag.z /= NUM_CAL_SAMPLE;
}

Vector_t<float> AHRS::getCompassRef() {
	return m_initialMag;
}

void AHRS::waitForSteadyGyro() {
	uint8_t count = 0;
	float xt = 0, yt = 0, zt = 0;
	float t = 0.2;
	while (count < 200) {
		Vector_t<float> gyro1 = m_imuSensor1.getGyroData();
		Vector_t<float> gyro2 = m_imuSensor2.getGyroData();

		float x = (gyro1.x - gyro2.x)/2;
		float y = (gyro1.y - gyro2.y)/2;
		float z = (gyro1.z + gyro2.z)/2;

		if(abs(x-xt)<=t && abs(y-yt)<=t && abs(z-zt)<t) {
			++count;
		}
		else{
			count = 0;
		}
		xt = x;
		yt = y;
		zt = z;
		TimeTick::delay_us(1300);
	}
}

void AHRS::updateVariance() {
    static int n = 0;

    n++;

//    double delta = new_value - gyroMean;
//    gyroMean += delta / n;
//    double delta2 = new_value - state->mean;
//    state->M2 += delta * delta2;
}

void AHRS::init(SPI_Config config1, SPI_Config config2, I2C_config mag_config) {
//	double ita = 1.0/ tan(M_PI * cFilterSetup.gyroFc / cFilterSetup.gyroFs); //0.726542528005361;//
//	double q=sqrt(2.0);
//	b0 = 1.0 / (1.0 + q*ita + ita*ita);
//	b1= 2*b0;
//	b2= b0;
//	a1 = 2.0 * (ita*ita - 1.0) * b0;
//	a2 = -(1.0 - q*ita + ita*ita) * b0;

#if USE_MADGWICK_FUSION
	filter.init();
#endif

	gxf.init(100, 500);
	gyf.init(100, 500);
	gzf.init(100, 500);

	m_imuSensor1.init(config1.hspi, config1.csPort, config1.csPin, config1.SPI_HS_CLK);
	m_imuSensor2.init(config2.hspi, config2.csPort, config2.csPin, config2.SPI_HS_CLK);
	m_compassSensor.initCompass(mag_config.i2cBus, mag_config.devAddr);



//	waitForSteadyGyro();
	calibrateGyro();

	if(Configurator::getConfig().settings.UseMagnetometer) {
		setCompassRef();
	}

//	if(Configurator::getConfig().settings.UseEKF) {
//		(EKF)filter.init(getCompassRef());
//	}
}

static void sendData(uint8_t* data, uint16_t len) {
	HAL_UART_Transmit_DMA(&huart3, data, len);
}


void AHRS::updateSensorData() {
	static char data[80];
	static timetick_us lastTime = 0, currentTime;
	static timetick_us t2=0;

	int NUM_ACC = 2;
	int NUM_GYRO = 2;

	Vector_t<float> acc1 = m_imuSensor1.getAccelData();
	Vector_t<float> gyro1 = m_imuSensor1.getGyroData();

	Vector_t<float> acc2 = m_imuSensor2.getAccelData();
	Vector_t<float> gyro2 = m_imuSensor2.getGyroData();

	m_sensorData.acc.x = (acc1.x - acc2.x)/NUM_ACC;
	m_sensorData.acc.y = (acc1.y - acc2.y)/NUM_ACC;
	m_sensorData.acc.z = (acc1.z + acc2.z)/NUM_ACC;


	m_sensorData.gyro.x = (gyro1.x - gyro2.x)/2 - gyroOffset.x;
	m_sensorData.gyro.y = (gyro1.y - gyro2.y)/2 - gyroOffset.y;
	m_sensorData.gyro.z = (gyro1.z + gyro2.z)/2 - gyroOffset.z;
//
//
//
//
//	m_sensorData.acc.x = acc1.x;
//	m_sensorData.acc.y = acc1.y;
//	m_sensorData.acc.z = acc1.z;
//
//
//	m_sensorData.gyro.x = gyro1.x - gyroOffset.x;
//	m_sensorData.gyro.y = gyro1.y - gyroOffset.y;
//	m_sensorData.gyro.z = gyro1.z - gyroOffset.z;



//	m_sensorData.acc.x = acc2.x;
//	m_sensorData.acc.y = acc2.y;
//	m_sensorData.acc.z = acc2.z;
//
//
//	m_sensorData.gyro.x = gyro2.x - gyroOffset.x;
//	m_sensorData.gyro.y = gyro2.y - gyroOffset.y;
//	m_sensorData.gyro.z = gyro2.z - gyroOffset.z;

//	filterGyro();
	currentTime = TimeTick::getTimeUs();

//	snprintf(data,80, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",(float)(currentTime-t2)/1000000,gyro2.x,gyro2.y,gyro2.z,m_sensorData.gyro.x,m_sensorData.gyro.y,m_sensorData.gyro.z);
//	sendData((uint8_t*)data, strlen(data));
	t2 = currentTime;

	if(Configurator::getConfig().settings.UseMagnetometer) {
		if(currentTime - lastTime >= MAG_ACQ_TIME_US) {
	//		HAL_GPIO_WritePin(TP_GPIO_Port, TP_Pin, GPIO_PIN_SET);
			CompassData compassData = m_compassSensor.getCompass();
			if(compassData.status) {
				m_sensorData.mag.x = compassData.mag.x;
				m_sensorData.mag.y = compassData.mag.y;
				m_sensorData.mag.z = compassData.mag.z;
	//			m_Heading = compassData.heading;
				_getHeading();
				m_magAvailable = true;
				lastTime = currentTime;
			}
	//		HAL_GPIO_WritePin(TP_GPIO_Port, TP_Pin, GPIO_PIN_RESET);
		}
		else {
			m_magAvailable = false;
		}
	}

	m_attitude.gyro = getGyro();
	m_attitude.accel = getAccel();
}

AHRS::SensorData AHRS::getSensorData() {
	return m_sensorData;
}

float AHRS::getHeading() {
	return m_Heading;
}

Vector_t<float> AHRS::getGroundAcc(Vector_t<float> a) {
	Quat q = m_attitude.quat;
	Vector_t<float> gAcc;

	float q02 = q.q0*q.q0;
	float q12 = q.q1*q.q1;
	float q22 = q.q2*q.q2;
	float q32 = q.q3*q.q3;

	float _2q1q3 = 2*q.q1*q.q3;
	float _2q0q2 = 2*q.q0*q.q2;
	float _2q2q3 = 2*q.q2*q.q3;
	float _2q0q1 = 2*q.q0*q.q1;
	float _2q1q2 = 2*q.q1*q.q2;
	float _2q0q3 = 2*q.q0*q.q3;

	gAcc.x = a.x*(q02 + q12 - q22 - q32) + a.y*(_2q1q2 - _2q0q3) + a.z*(_2q1q3 + _2q0q2);
	gAcc.y = a.x*(_2q1q2 + _2q0q3) + a.y*(q02 - q12 + q22 - q32) + a.z*(_2q2q3 - _2q0q1);
	gAcc.z = a.x*(_2q1q3 - _2q0q2) + a.y*(_2q2q3 + _2q0q1) + a.z*(q02 - q12 - q22 + q32) - 1;

	return gAcc;
}

Attitude AHRS::fushionUpdate(float dT) {
	m_dT = dT;
	updateSensorData();
	filter.update(_SensorData(m_sensorData), dT, m_magAvailable);

#if USE_MADGWICK_FUSION

#elif USE_COMP || defined USE_EKFT
	m_attitude.quat = euler2Quaternion(m_attitude.euler);
#else
	m_attitude.euler = quat2Euler(m_attitude.quat);
#endif

	return m_attitude;
}

void AHRS::filterGyro() {
	m_sensorData.gyro.x = gxf.apply(m_sensorData.gyro.x);
	m_sensorData.gyro.y = gyf.apply(m_sensorData.gyro.y);
	m_sensorData.gyro.z = gzf.apply(m_sensorData.gyro.z);
}

float AHRS::_getHeading() {
	float r = atan2f(-m_sensorData.acc.y, m_sensorData.acc.z);//m_attitude.euler.r;
	float p = atan2f(m_sensorData.acc.x, sqrtf(m_sensorData.acc.y * m_sensorData.acc.y + m_sensorData.acc.z * m_sensorData.acc.z)) - M_PI/6;
//	float p = (m_attitude.euler.p - 30) * DEG2RAD;
//	float r = m_attitude.euler.r * DEG2RAD;
	float mx = -m_sensorData.mag.y;
	float my = m_sensorData.mag.x;
	float mz = m_sensorData.mag.z;
	// Tilt compensation
	float mag_x = mx * cos(p) + mz * sin(p);
	float mag_y = mx * sin(r) * sin(p) + my * cos(r) - mz * sin(r) * cos(p);

	m_Heading = atan2(mag_y, mag_x) * RAD2DEG;
	m_Heading += 180;
//	m_Heading = (int)( m_Heading + 360) % 360;
//	if(m_Heading < 0) {
//		m_Heading += 360;
//	}
	m_Heading = (float)(360 - (int)m_Heading);

	m_osd->setHeading(m_Heading);

	return m_Heading;
//	return m_Heading;
}


Euler AHRS::quat2Euler(Quat data)
{
	Euler ans;

    double q2sqr = data.q2 * data.q2;
    double t0 = -2.0 * (q2sqr + data.q3 * data.q3) + 1.0;
    double t1 = +2.0 * (data.q1 * data.q2 + data.q0 * data.q3);
    double t2 = -2.0 * (data.q1 * data.q3 - data.q0 * data.q2);
    double t3 = +2.0 * (data.q2 * data.q3 + data.q0 * data.q1);
    double t4 = -2.0 * (data.q1 * data.q1 + q2sqr) + 1.0;

    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;

    ans.p = asin(t2)*RAD2DEG;
    ans.r = atan2(t3, t4)*RAD2DEG;
    ans.y = atan2(t1, t0)*RAD2DEG;

    return ans;
}


Quat AHRS::euler2Quaternion(Euler data)
{
	Quat ans;
    double t0 = cos(data.y * DEG2RAD * 0.5);
    double t1 = sin(data.y * DEG2RAD * 0.5);
    double t2 = cos(data.r * DEG2RAD * 0.5);
    double t3 = sin(data.r * DEG2RAD * 0.5);
    double t4 = cos(data.p * DEG2RAD * 0.5);
    double t5 = sin(data.p * DEG2RAD * 0.5);

    ans.q0 = t2 * t4 * t0 + t3 * t5 * t1;
    ans.q1 = t3 * t4 * t0 - t2 * t5 * t1;
    ans.q2 = t2 * t5 * t0 + t3 * t4 * t1;
    ans.q3 = t2 * t4 * t1 - t3 * t5 * t0;
    return ans;
}


void AHRS::taskFunc(timetick_us currenTimeUs) {
	float dT = 0;
	static bool first = true;
	if(first) {
		dT = 0.001;
		first = false;
	}
	else{
		dT = (float)(currenTimeUs - lastRunTime) / 1000000;
	}
	HAL_GPIO_WritePin(TP_GPIO_Port, TP_Pin, GPIO_PIN_SET);
	fushionUpdate(dT);
	HAL_GPIO_WritePin(TP_GPIO_Port, TP_Pin, GPIO_PIN_RESET);
}
