/*****************************************************************************/
//    Function:     Header file for HMC5883L
//  Hardware:    Grove - 3-Axis Digital Compass
//    Arduino IDE: Arduino-1.0
//    Author:     Frankie.Chu
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

#ifndef __HMC5883L_H__
#define __HMC5883L_H__

//#include <Arduino.h>
//#include <Wire.h>
#include <typedefs.h>
#include <Configurator.h>


#define HMC5883L_ADDRESS 0x1E
#define CONFIGURATION_REGISTERA 0x00
#define CONFIGURATION_REGISTERB 0x01
#define MODE_REGISTER 0x02
#define DATA_REGISTER_BEGIN 0x03
#define IDENTIFICATION_REGISTERA 0x0A
#define IDENTIFICATION_REGISTERB 0x0B
#define IDENTIFICATION_REGISTERC 0x0C

#define MEASUREMENT_CONTINUOUS 0x00
#define MEASUREMENT_SINGLE_SHOT 0x01
#define MEASUREMENT_IDLE 0x03

#define ERRORCODE_1 "Entered scale was not valid, valid gauss values are: 0.88, 1.3, 1.9, 2.5, 4.0, 4.7, 5.6, 8.1"
#define ERRORCODE_1_NUM 1

//typedef struct CalibData {
//	float scale[3][3] = {{1.005367, -0.000931, -0.000440},
//					   {-0.000931, 0.989576, -0.015483},
//					   {-0.000440, -0.015483, 1.018926}};
//	Vector_t<float> offset = {.x = -0.989077, .y = 157.014939, .z = 126.731613};
//}magCalibData_t;

//typedef struct CalibData {
//	float scale[3][3] = {{0.003604, 0.000072, -0.000002},
//					   {0.000072, 0.001453, 0.000000},
//					   {-0.000002, 0.000000, 0.003888}};
//
//	Vector_t<float> offset = {.x = -12796.209484, .y = -3254.625648, .z = 427.563262};
//}magCalibData_t;

//typedef struct CalibData {
//	float scale[3][3] = {{0.107932, -0.001072, 0.000457},
//					   {-0.001072, 0.103104, -0.001992},
//					   {0.000457, -0.001992, 0.103650}};
//
//	Vector_t<float> offset = {.x = 20.010780, .y = 160.228349, .z = 117.908799};
//}magCalibData_t;

class HMC5883L
{
	enum Mode {
		Mode_Continuous,
		Mode_Single_Shot,
		Mode_Idle = 3
	};

	enum Scale {
		Scale_0_88Ga,
		Scale_1_3Ga,
		Scale_1_9Ga,
		Scale_2_5Ga,
		Scale_4_0Ga,
		Scale_4_7Ga,
		Scale_5_6Ga,
		Scale_8_1Ga,
	};

public:         // used by xadow phone

    bool initCompass(I2C_Bus* i2cBus, devAddr_t devAddr = MAG_DEV_ADDR);
    CompassData getCompass();
    
public:
    HMC5883L();

    Vector_t<int16_t> readRawAxis();
    Vector_t<float> readScaledAxis();

    void setMeasurementMode(Mode mode);

    void setScale(Scale gauss);

    void getID(uint8_t[3]);

    void calibrate();
    
    void calibrateExt();

    void setMagneticDeclination(float degrees);

    
protected:

    typedef struct CalibData {
    		Vector_t<float> scale = {.x = 1.01740813, .y = 1.01251209, .z = 0.971375823};
    		Vector_t<float> offset = {.x = -2.75999999, .y = 127.420006, .z = 124.660004};
    }CalibData;

    void correctDeclination(Vector_t<float> &mag);
    void write(uint8_t address, uint8_t data);
    void read(uint8_t address, uint8_t* data, uint8_t length);

    private:
    float m_Sensitivity;
    float m_Heading;
    I2C_Bus* m_i2cBus;
    devAddr_t m_devAddr;
    float decl;

    CalibData m_calibData;
};

#endif
