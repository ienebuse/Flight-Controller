/*
 * QMC5883L.cpp
 *
 *  Created on: Nov 8, 2024
 *      Author: Ikenna
 */

#include <TimeTick.h>
#include <stdio.h>
#include <string.h>
#include <usart.h>
#include <cmath>
#include <Configurator.h>
#include <sensors/QMC5883L.h>

#define OSR_512		0 << 6
#define OSR_256		1 << 6
#define OSR_128		2 << 6
#define OSR_64		3 << 6

#define RNG_2G 		0 << 4
#define RNG_8G		1 << 4

#define ODR_10		0 << 2
#define ODR_50		1 << 2
#define ODR_100		2 << 2
#define ODR_200		3 << 2

#define MODE_CONT	1
#define MODE_STBY	0




/*
===============================================================================================================
QMC5883LCompass.h
Library for using QMC5583L series chip boards as a compass.
Learn more at [https://github.com/mprograms/QMC5883LCompass]

Supports:

- Getting values of XYZ axis.
- Calculating Azimuth.
- Getting 16 point Azimuth bearing direction (0 - 15).
- Getting 16 point Azimuth bearing Names (N, NNE, NE, ENE, E, ESE, SE, SSE, S, SSW, SW, WSW, W, WNW, NW, NNW)
- Smoothing of XYZ readings via rolling averaging and min / max removal.
- Optional chipset modes (see below)

===============================================================================================================

v1.0 - June 13, 2019
Written by MPrograms
Github: [https://github.com/mprograms/]

Release under the GNU General Public License v3
[https://www.gnu.org/licenses/gpl-3.0.en.html]

===============================================================================================================



FROM QST QMC5883L Datasheet [https://nettigo.pl/attachments/440]
-----------------------------------------------
 MODE CONTROL (MODE)
	Standby			0x00
	Continuous		0x01

OUTPUT DATA RATE (ODR)
	10Hz        	0x00
	50Hz        	0x04
	100Hz       	0x08
	200Hz       	0x0C

FULL SCALE (RNG)
	2G          	0x00
	8G          	0x10

OVER SAMPLE RATIO (OSR)
	512         	0x00
	256         	0x40
	128         	0x80
	64          	0xC0

*/



QMC5883L::QMC5883L() {
	// TODO Auto-generated constructor stub

}

QMC5883L::~QMC5883L() {
	// TODO Auto-generated destructor stub
}


/**
	INIT
	Initialize Chip - This needs to be called in the sketch setup() function.

	@since v0.1;
**/
bool QMC5883L::initCompass(I2C_Bus* i2cBus, devAddr_t devAddr) {
	m_i2cBus = i2cBus;
	m_devAddr = devAddr;

    TimeTick::delay_us(5000);

    uint8_t id = getID();

    if(id != 0xFF) return false;

	_writeReg(0x0B,0x01);
//	setMode(0x01,0x0C,0x10,0X00);

	//	// OSR[7:6]
	//	OSR_512         = Bit7 | Bit6, // 00
	//
	//	// RNG[5:4]
	//	RNG_2G          = Bit5 | Bit4, // 00
	//
	//	// ODR[3:2]
	//	ODR_50HZ        = Bit2,        // 01
	//
	//	// MODE[1:0]
	//	Mode_Continuous = Bit0,        // 01
//	setMode(0x05);

//	setMode(0x1D);	// OSR_512, RNG_8G, ODR_200Hz, MODE_CONT

	setMode(OSR_512 | RNG_8G | ODR_100 | MODE_CONT);

	setMagneticDeclination(Configurator::getConfig().settings.MagDeclination);

//	calibrate();
	return true;
}

uint8_t QMC5883L::getID() {
	uint8_t id;
	_readReg(0x0D, &id, 1);
	return id;
}


/**
	SET ADDRESS
	Set the I2C Address of the chip. This needs to be called in the sketch setup() function.

	@since v0.1;
**/
// Set I2C Address if different then default.
void QMC5883L::setADDR(uint8_t b){
	_ADDR = b;
}

/**
	REGISTER
	Write the register to the chip.

	@since v0.1;
**/
// Write register values to chip
void QMC5883L::_writeReg(uint8_t r, uint8_t v){
	m_i2cBus->mem_write(m_devAddr, r, &v, 1);
}

void QMC5883L::_readReg(uint8_t r, uint8_t* v, uint16_t l) {
	m_i2cBus->mem_read(m_devAddr, r, v, l);
}


/**
	CHIP MODE
	Set the chip mode.

	@since v0.1;
**/
// Set chip mode
void QMC5883L::setMode(uint8_t mode, uint8_t odr, uint8_t rng, uint8_t osr){
	_writeReg(0x09,mode|odr|rng|osr);
}

void QMC5883L::setMode(uint8_t mode) {
	_writeReg(0x09, mode);
}


/**
 * Define the magnetic declination for accurate degrees.
 * https://www.magnetic-declination.com/
 *
 * @example
 * For: Londrina, PR, Brazil at date 2022-12-05
 * The magnetic declination is: -19º 43'
 *
 * then: setMagneticDeclination(-19, 43);
 */
void QMC5883L::setMagneticDeclination(int degrees, uint8_t minutes) {
	_magneticDeclinationDegrees = degrees + minutes / 60;
}

void QMC5883L::setMagneticDeclination(float degrees) {
	_magneticDeclinationDegrees = degrees;
}


/**
	RESET
	Reset the chip.

	@since v0.1;
**/
// Reset the chip
void QMC5883L::setReset(){
	_writeReg(0x0A,0x80);
}

// 1 = Basic 2 = Advanced
void QMC5883L::setSmoothing(uint8_t steps, bool adv){
	_smoothUse = true;
	_smoothSteps = ( steps > 10) ? 10 : steps;
	_smoothAdvanced = (adv == true) ? true : false;
}

//void QMC5883L::calibrate() {
//	clearCalibration();
//	long calibrationData[3][2] = {{65000, -65000}, {65000, -65000}, {65000, -65000}};
//  	long	x = calibrationData[0][0] = calibrationData[0][1] = getX();
//  	long	y = calibrationData[1][0] = calibrationData[1][1] = getY();
//  	long	z = calibrationData[2][0] = calibrationData[2][1] = getZ();
//
//	unsigned long startTime = millis();
//
//	while((millis() - startTime) < 10000) {
//		readXYZ();
//
//  		x = getX();
//  		y = getY();
//  		z = getZ();
//
//		if(x < calibrationData[0][0]) {
//			calibrationData[0][0] = x;
//		}
//		if(x > calibrationData[0][1]) {
//			calibrationData[0][1] = x;
//		}
//
//		if(y < calibrationData[1][0]) {
//			calibrationData[1][0] = y;
//		}
//		if(y > calibrationData[1][1]) {
//			calibrationData[1][1] = y;
//		}
//
//		if(z < calibrationData[2][0]) {
//			calibrationData[2][0] = z;
//		}
//		if(z > calibrationData[2][1]) {
//			calibrationData[2][1] = z;
//		}
//	}
//
//	setCalibration(
//		calibrationData[0][0],
//		calibrationData[0][1],
//		calibrationData[1][0],
//		calibrationData[1][1],
//		calibrationData[2][0],
//		calibrationData[2][1]
//	);
//}

void QMC5883L::calibrate() {
	uint16_t ii = 0, sample_count = 0;
//	int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
//	int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

	Vector_t<int16_t> mag_temp;
	Vector_t<int32_t> mag_max = {.x = -65000, .y = -65000, .z = -65000};
	Vector_t<int32_t> mag_min = {.x = 65000, .y = 65000, .z = 65000};

	for(int i = 0; i < 40; i++) {
		HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
		TimeTick::delay_us(50000);
	}

	// shoot for ~fifteen seconds of mag data
	sample_count = 255;  // at 15 Hz ODR, new mag data is available every 66667 us
	for(ii = 0; ii < sample_count; ii++) {
		mag_temp = getRawXYZ();  // Read the mag data
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

/**
    SET CALIBRATION
	Set calibration values for more accurate readings

	@author Claus Näveke - TheNitek [https://github.com/TheNitek]

	@since v1.1.0

	@deprecated Instead of setCalibration, use the calibration offset and scale methods.
**/
void QMC5883L::setCalibration(int x_min, int x_max, int y_min, int y_max, int z_min, int z_max){
	setCalibrationOffsets(
		(x_min + x_max)/2,
		(y_min + y_max)/2,
		(z_min + z_max)/2
	);

	float x_avg_delta = (x_max - x_min)/2;
	float y_avg_delta = (y_max - y_min)/2;
	float z_avg_delta = (z_max - z_min)/2;

	float avg_delta = (x_avg_delta + y_avg_delta + z_avg_delta) / 3;

	setCalibrationScales(
		avg_delta / x_avg_delta,
		avg_delta / y_avg_delta,
		avg_delta / z_avg_delta
	);
}

void QMC5883L::setCalibrationOffsets(float x_offset, float y_offset, float z_offset) {
	_offset[0] = x_offset;
	_offset[1] = y_offset;
	_offset[2] = z_offset;
}

void QMC5883L::setCalibrationScales(float x_scale, float y_scale, float z_scale) {
	_scale[0] = x_scale;
	_scale[1] = y_scale;
	_scale[2] = z_scale;
}

float QMC5883L::getCalibrationOffset(uint8_t index) {
	return _offset[index];
}

float QMC5883L::getCalibrationScale(uint8_t index) {
	return _scale[index];
}

void QMC5883L::clearCalibration(){
	setCalibrationOffsets(0., 0., 0.);
	setCalibrationScales(1., 1., 1.);
}

/**
	READ
	Read the XYZ axis and save the values in an array.

	@since v0.1;
**/
bool QMC5883L::readXYZ(){
	uint8_t raw[6] = {0,0,0,0,0,0};
	uint8_t status;
	_readReg(0x06, &status, 1);
	if(!(status & 0x02) && (status & 0x01)) {
		_readReg(0x00, raw, 6);

		_vRaw[0] = (int)(int16_t)(raw[0] | raw[1] << 8);
		_vRaw[1] = (int)(int16_t)(raw[2] | raw[3] << 8);
		_vRaw[2] = (int)(int16_t)(raw[4] | raw[5] << 8);

		_applyCalibration();

		if ( _smoothUse ) {
			_smoothing();
		}
		return true;
	}

	return false;

		//byte overflow = Wire.read() & 0x02;
		//return overflow << 2;
}

Vector_t<int16_t> QMC5883L::getRawXYZ()
{
    uint8_t buffer[6] = {0,0,0,0,0,0};
    _readReg(0x00, buffer, 6);
    Vector_t<int16_t> raw;;
    raw.x = (int16_t)((buffer[0]) | buffer[1] << 8);
    raw.z = (int16_t)((buffer[2]) | buffer[3] << 8);
    raw.y = (int16_t)((buffer[4]) | buffer[5] << 8);
    return raw;
}

CompassData QMC5883L::getCompass() {
	CompassData compass;

	bool success = readXYZ();
	if(!success) {
		compass.status = false;
		return compass;
	}

	compass.status = true;

	compass.mag.x = _vCalibrated[0];
	compass.mag.y = _vCalibrated[1];
	compass.mag.z = _vCalibrated[2];

	compass.heading = getAzimuth();
	return compass;
}

/**
    APPLY CALIBRATION
	This function uses the calibration data provided via @see setCalibration() to calculate more
	accurate readings

	@author Claus Näveke - TheNitek [https://github.com/TheNitek]

	Based on this awesome article:
	https://appelsiini.net/2018/calibrate-magnetometer/

	@since v1.1.0

**/
void QMC5883L::_applyCalibration(){
	_vCalibrated[0] = (_vRaw[0] - m_calibData.offset.x) * m_calibData.scale.x;
	_vCalibrated[1] = (_vRaw[1] - m_calibData.offset.y) * m_calibData.scale.y;
	_vCalibrated[2] = (_vRaw[2] - m_calibData.offset.z) * m_calibData.scale.z;
}


/**
	SMOOTH OUTPUT
	This function smooths the output for the XYZ axis. Depending on the options set in
	@see setSmoothing(), we can run multiple methods of smoothing the sensor readings.

	First we store (n) samples of sensor readings for each axis and store them in a rolling array.
	As each new sensor reading comes in we replace it with a new reading. Then we average the total
	of all (n) readings.

	Advanced Smoothing
	If you turn advanced smoothing on, we will select the min and max values from our array
	of (n) samples. We then subtract both the min and max from the total and average the total of all
	(n - 2) readings.

	NOTE: This function does several calculations and can cause your sketch to run slower.

	@since v0.3;
**/
void QMC5883L::_smoothing(){
	uint8_t max = 0;
	uint8_t min = 0;

	if ( _vScan > _smoothSteps - 1 ) { _vScan = 0; }

	for ( int i = 0; i < 3; i++ ) {
		if ( _vTotals[i] != 0 ) {
			_vTotals[i] = _vTotals[i] - _vHistory[_vScan][i];
		}
		_vHistory[_vScan][i] = _vCalibrated[i];
		_vTotals[i] = _vTotals[i] + _vHistory[_vScan][i];

		if ( _smoothAdvanced ) {
			max = 0;
			for (int j = 0; j < _smoothSteps - 1; j++) {
				max = ( _vHistory[j][i] > _vHistory[max][i] ) ? j : max;
			}

			min = 0;
			for (int k = 0; k < _smoothSteps - 1; k++) {
				min = ( _vHistory[k][i] < _vHistory[min][i] ) ? k : min;
			}

			_vSmooth[i] = ( _vTotals[i] - (_vHistory[max][i] + _vHistory[min][i]) ) / (_smoothSteps - 2);
		} else {
			_vSmooth[i] = _vTotals[i]  / _smoothSteps;
		}
	}

	_vScan++;
}


/**
	GET X AXIS
	Read the X axis

	@since v0.1;
	@return int x axis
**/
int QMC5883L::getX(){
	return _get(0);
}


/**
	GET Y AXIS
	Read the Y axis

	@since v0.1;
	@return int y axis
**/
int QMC5883L::getY(){
	return _get(1);
}


/**
	GET Z AXIS
	Read the Z axis

	@since v0.1;
	@return int z axis
**/
int QMC5883L::getZ(){
	return _get(2);
}

/**
	GET SENSOR AXIS READING
	Get the smoothed, calibration, or raw data from a given sensor axis

	@since v1.1.0
	@return int sensor axis value
**/
int QMC5883L::_get(int i){
	if ( _smoothUse )
		return _vSmooth[i];

	return _vCalibrated[i];
}



/**
	GET AZIMUTH
	Calculate the azimuth (in degrees);
	Correct the value with magnetic declination if defined.

	@since v0.1;
	@return int azimuth
**/
int QMC5883L::getAzimuth(){
//	float heading = atan2( getY(), getX() ) * 180.0 / M_PI;
	float heading = atan2( getX(), -getY() ) * 180.0 / M_PI;
	heading += _magneticDeclinationDegrees;
	heading = (int)( heading + 360 ) % 360;
//	if(heading < 0) {
//		heading += 360;
//	}
	return (int)heading;
}


/**
	GET BEARING
	Divide the 360 degree circle into 16 equal parts and then return the a value of 0-15
	based on where the azimuth is currently pointing.


	@since v1.2.1 - function takes into account negative azimuth values. Credit: https://github.com/prospark
	@since v1.0.1 - function now requires azimuth parameter.
	@since v0.2.0 - initial creation

	@return byte direction of bearing
*/
uint8_t QMC5883L::getBearing(int azimuth){
	unsigned long a = ( azimuth > -0.5 ) ? azimuth / 22.5 : (azimuth+360)/22.5;
	unsigned long r = a - (int)a;
	uint8_t sexdec = 0;
	sexdec = ( r >= .5 ) ? ceil(a) : floor(a);
	return sexdec;
}


/**
	This will take the location of the azimuth as calculated in getBearing() and then
	produce an array of chars as a text representation of the direction.

	NOTE: This function does not return anything since it is not possible to return an array.
	Values must be passed by reference back to your sketch.

	Example:

	( if direction is in 1 / NNE)

	char myArray[3];
	compass.getDirection(myArray, azimuth);

	Serial.print(myArray[0]); // N
	Serial.print(myArray[1]); // N
	Serial.print(myArray[2]); // E


	@see getBearing();

	@since v1.0.1 - function now requires azimuth parameter.
	@since v0.2.0 - initial creation
*/
void QMC5883L::getDirection(char* myArray, int azimuth){
	int d = getBearing(azimuth);
	myArray[0] = _bearings[d][0];
	myArray[1] = _bearings[d][1];
	myArray[2] = _bearings[d][2];
}


