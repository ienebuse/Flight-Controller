/*****************************************************************************/
//    Function:     Cpp file for HMC5883L
//  Hardware:    Grove - 3-Axis Digital Compass
//    Arduino IDE: Arduino-1.0
//    Author:     FrankieChu
//    Date:      Jan 10,2013
//    Version: v1.0
//    by www.seeedstudio.com
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
/*******************************************************************************/

//#include <Arduino.h>
#include "HMC5883L.h"
#include <TimeTick.h>
#include <stdio.h>
#include <string.h>
#include <usart.h>
#include <cmath>

HMC5883L::HMC5883L()
{
    m_Sensitivity = 1;
}

bool HMC5883L::initCompass(I2C_Bus* i2cBus, devAddr_t devAddr)
{
	m_i2cBus = i2cBus;
	m_devAddr = devAddr;
    
    TimeTick::delay_us(5000);
    
    uint8_t id[3];

    getID(id);

    if(id[0] != 0x48 || id[1] != 0x34 || id[2] != 0x33) return false;

    setScale(Scale_1_3Ga);
    
    setMeasurementMode(Mode_Continuous);

    calibrate();
//    calibrateExt();

    setMagneticDeclination(Configurator::getConfig().MagDeclination);

    return true;
}

void HMC5883L::correctDeclination(Vector_t<float> &mag) {
	mag.x = mag.x * cos(decl) + mag.y * sin(decl);
	mag.y = -mag.x * sin(decl) + mag.y * cos(decl);
}


CompassData HMC5883L::getCompass()
{
	CompassData compass;
//	Vector_t<int8_t> raw = readRawAxis();
    // Retrived the scaled values from the compass (scaled to the configured scale).
	Vector_t<float> mag = readScaledAxis();

    // Values are accessed like so:
//    int MilliGauss_OnThe_XAxis = mag.x;// (or YAxis, or ZAxis)

    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    float heading = atan2(mag.y, mag.x);

    // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
    // Find yours here: http://www.magnetic-declination.com/
    // Mine is: -2??37' which is -2.617 Degrees, or (which we need) -0.0456752665 radians, I will use -0.0457
    // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
//    float declinationAngle = MAG_DECLINATION * DEG2RAD;
    heading += decl;

    // Correct for when signs are reversed.
    if(heading < 0)
    heading += 2*M_PI;

    // Check for wrap due to addition of declination.
    if(heading > 2*M_PI)
    heading -= 2*M_PI;

    // Convert radians to degrees for readability.
    m_Heading = heading * RAD2DEG;
    
    compass.mag = mag;
    compass.heading = m_Heading;
	return compass;
}



Vector_t<int16_t> HMC5883L::readRawAxis()
{
    uint8_t buffer[6];
    read(DATA_REGISTER_BEGIN, buffer, 6);
    Vector_t<int16_t> raw;;
    raw.x = (int16_t)((buffer[0] << 8) | buffer[1]);
    raw.z = (int16_t)((buffer[2] << 8) | buffer[3]);
    raw.y = (int16_t)((buffer[4] << 8) | buffer[5]);
    return raw;
}

Vector_t<float> HMC5883L::readScaledAxis()
{
	Vector_t<int16_t> raw = readRawAxis();
	Vector_t<float> scaled;
    scaled.x = (raw.x * m_Sensitivity - m_calibData.offset.x)*m_calibData.scale.x;
    scaled.y = (raw.y * m_Sensitivity - m_calibData.offset.y)*m_calibData.scale.y;
    scaled.z = (raw.z * m_Sensitivity - m_calibData.offset.z)*m_calibData.scale.z;
    return scaled;
}

//Vector_t<float> HMC5883L::readScaledAxis()
//{
//	Vector_t<int16_t> raw = readRawAxis();
//	Vector_t<float> scaled;
//
//	raw.x *= m_Sensitivity;
//	raw.y *= m_Sensitivity;
//	raw.z *= m_Sensitivity;
//
//	scaled.x = raw.x * m_calibData.scale[0][0] + raw.y * m_calibData.scale[0][1] + raw.z * m_calibData.scale[0][2] - m_calibData.offset.x;
//    scaled.y = raw.x * m_calibData.scale[1][0] + raw.y * m_calibData.scale[1][1] + raw.z * m_calibData.scale[1][2] - m_calibData.offset.y;
//    scaled.z = raw.x * m_calibData.scale[2][0] + raw.y * m_calibData.scale[2][1] + raw.z * m_calibData.scale[2][2] - m_calibData.offset.z;
//    correctDeclination(scaled);
//    return scaled;
//}

void HMC5883L::setScale(Scale scale)
{
    uint8_t regValue = 0x00;
    switch(scale)
    {
    	case Scale_0_88Ga:
		{
			regValue = 0x00;
			m_Sensitivity = 0.73;
		}
		break;
    	case Scale_1_3Ga:
		{
			regValue = 0x01;
			m_Sensitivity = 0.92;
		}
    	break;
    	case Scale_1_9Ga:
		{
			regValue = 0x02;
			m_Sensitivity = 1.22;
		}
		break;
    	case Scale_2_5Ga:
		{
			regValue = 0x03;
			m_Sensitivity = 1.52;
		}
		break;
    	case Scale_4_0Ga:
		{
			regValue = 0x04;
			m_Sensitivity = 2.27;
		}
		break;
    	case Scale_4_7Ga:
		{
			regValue = 0x05;
			m_Sensitivity = 2.56;
		}
		break;
    	case Scale_5_6Ga:
		{
			regValue = 0x06;
			m_Sensitivity = 3.03;
		}
		break;
    	case Scale_8_1Ga:
		{
			regValue = 0x07;
			m_Sensitivity = 4.35;
		}
    	break;
    	default:
    		return;
    }

    // Setting is in the top 3 bits of the register.
    regValue = (uint8_t)scale << 5;
    write(CONFIGURATION_REGISTERB, regValue);
}

void HMC5883L::setMeasurementMode(Mode mode)
{
	uint8_t modeReg;
    write(MODE_REGISTER, (uint8_t)mode);
    read(MODE_REGISTER, &modeReg,1);
    if(modeReg == (uint8_t)mode) {
    	int x = 0;
    	x++;
    }
}

void HMC5883L::getID(uint8_t devID[3]) {
	read(IDENTIFICATION_REGISTERA, devID, 3);
}

void HMC5883L::calibrate() {
	uint16_t ii = 0, sample_count = 0;
//	int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
//	int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

	Vector_t<int16_t> mag_temp;
	Vector_t<int16_t> mag_max = {.x = -32767, .y = -32767, .z = -32767};
	Vector_t<int16_t> mag_min = {.x = 32767, .y = 32767, .z = 32767};

	for(int i = 0; i < 40; i++) {
		HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
		TimeTick::delay_us(50000);
	}

	// shoot for ~fifteen seconds of mag data
	sample_count = 255;  // at 15 Hz ODR, new mag data is available every 66667 us
	for(ii = 0; ii < sample_count; ii++) {
		mag_temp = readRawAxis();  // Read the mag data
		if(mag_temp.x > mag_max.x) mag_max.x = mag_temp.x;
		if(mag_temp.y > mag_max.y) mag_max.y = mag_temp.y;
		if(mag_temp.z > mag_max.z) mag_max.z = mag_temp.z;

		if(mag_temp.x < mag_min.x) mag_min.x = mag_temp.x;
		if(mag_temp.y < mag_min.y) mag_min.y = mag_temp.y;
		if(mag_temp.z < mag_min.z) mag_min.z = mag_temp.z;

		TimeTick::delay_us(66667);
	}

	// Get hard iron correction
	m_calibData.offset.x = (float)(mag_max.x + mag_min.x)/2 * m_Sensitivity;
	m_calibData.offset.y = (float)(mag_max.y + mag_min.y)/2 * m_Sensitivity;
	m_calibData.offset.z = (float)(mag_max.z + mag_min.z)/2 * m_Sensitivity;

	// Get soft iron correction estimate
	m_calibData.scale.x = (float)(mag_max.x - mag_min.x)/2;
	m_calibData.scale.y = (float)(mag_max.y - mag_min.y)/2;
	m_calibData.scale.z = (float)(mag_max.z - mag_min.z)/2;

	float avg_rad = (float)(m_calibData.scale.x + m_calibData.scale.y + m_calibData.scale.z)/3;

	m_calibData.scale.x = avg_rad/m_calibData.scale.x;
	m_calibData.scale.y = avg_rad/m_calibData.scale.y;
	m_calibData.scale.z = avg_rad/m_calibData.scale.z;
}


void HMC5883L::calibrateExt() {
	uint16_t ii = 0, sample_count = 0;

	Vector_t<int16_t> mag_temp;
	char data[60];

	for(int i = 0; i < 40; i++) {
		HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
		TimeTick::delay_us(50000);
	}

	// shoot for ~fifteen seconds of mag data
	sample_count = 320;  // at 15 Hz ODR, new mag data is available every 66667 us
	for(ii = 0; ii < sample_count; ii++) {
		mag_temp = readRawAxis();  // Read the mag data

		snprintf(data,60,"%.4f,%.4f,%.4f\n", mag_temp.x*m_Sensitivity, mag_temp.y*m_Sensitivity, mag_temp.z*m_Sensitivity);
		HAL_UART_Transmit_IT(&huart3, (const uint8_t *)data, strlen(data));

		TimeTick::delay_us(66667);
	}
}

void HMC5883L::setMagneticDeclination(float degrees) {
	decl = DEG2RAD*degrees;
}

void HMC5883L::write(uint8_t address, uint8_t data)
{
    m_i2cBus->mem_write(m_devAddr, address, &data, 1);
}

void HMC5883L::read(uint8_t address, uint8_t* data, uint8_t length)
{
    m_i2cBus->mem_read(m_devAddr, address, data, length);
}


