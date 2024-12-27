/*
 * QMC5883L.h
 *
 *  Created on: Nov 8, 2024
 *      Author: Ikenna
 */

#ifndef SENSORS_QMC5883L_H_
#define SENSORS_QMC5883L_H_

#include <typedefs.h>


class QMC5883L {
public:
	QMC5883L();
	virtual ~QMC5883L();

	bool initCompass(I2C_Bus* i2cBus, devAddr_t devAddr = 0x0D);
	uint8_t getID();
    void setADDR(uint8_t b);
    void setMode(uint8_t mode, uint8_t odr, uint8_t rng, uint8_t osr);
    void setMode(uint8_t mode);
	void setMagneticDeclination(int degrees, uint8_t minutes);
	void setMagneticDeclination(float degrees);
	void setSmoothing(uint8_t steps, bool adv);
	void calibrate();
	void setCalibration(int x_min, int x_max, int y_min, int y_max, int z_min, int z_max);
	void setCalibrationOffsets(float x_offset, float y_offset, float z_offset);
	void setCalibrationScales(float x_scale, float y_scale, float z_scale);
    float getCalibrationOffset(uint8_t index);
	float getCalibrationScale(uint8_t index);
	void clearCalibration();
	void setReset();
    bool readXYZ();
    Vector_t<int16_t> getRawXYZ();
    CompassData getCompass();
	int getX();
	int getY();
	int getZ();
	int getAzimuth();
	uint8_t getBearing(int azimuth);
	void getDirection(char* myArray, int azimuth);

private:
	typedef struct CalibData {
		Vector_t<float> scale = {.x = 1.03822351, .y = 0.905684352, .z = 1.07218051};
		Vector_t<float> offset = {.x = -153.5, .y = 188.5, .z = -48};
	}CalibData;


    void _writeReg(uint8_t reg,uint8_t val);
    void _readReg(uint8_t r, uint8_t* v, uint16_t l);
	int _get(int index);

    float m_Heading;
    I2C_Bus* m_i2cBus;
    devAddr_t m_devAddr;

	float _magneticDeclinationDegrees;
	bool _smoothUse = false;
	uint8_t _smoothSteps = 5;
	bool _smoothAdvanced = false;
    uint8_t _ADDR = 0x0D;
	int _vRaw[3] = {0,0,0};
	int _vHistory[10][3];
	int _vScan = 0;
	long _vTotals[3] = {0,0,0};
	int _vSmooth[3] = {0,0,0};
	void _smoothing();
	float _offset[3] = {0.,0.,0.};
	float _scale[3] = {1.,1.,1.};
	int _vCalibrated[3];
	void _applyCalibration();
	CalibData m_calibData;
	float m_Sensitivity = 1;
	const char _bearings[16][3] =  {
		{' ', ' ', 'N'},
		{'N', 'N', 'E'},
		{' ', 'N', 'E'},
		{'E', 'N', 'E'},
		{' ', ' ', 'E'},
		{'E', 'S', 'E'},
		{' ', 'S', 'E'},
		{'S', 'S', 'E'},
		{' ', ' ', 'S'},
		{'S', 'S', 'W'},
		{' ', 'S', 'W'},
		{'W', 'S', 'W'},
		{' ', ' ', 'W'},
		{'W', 'N', 'W'},
		{' ', 'N', 'W'},
		{'N', 'N', 'W'},
	};
};

#endif /* SENSORS_QMC5883L_H_ */
