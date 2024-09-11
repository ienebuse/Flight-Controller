/*
 * DPS3.cpp
 *
 *  Created on: Jul 30, 2024
 *      Author: Ikenna
 */

#include <sensors/DPS310.h>
//#include <i2c.h>

#define DPS__BUSYTIME_SCALING 10U
// DPS310 has 10 milliseconds of spare time for each synchronous measurement / per second for asynchronous measurements
// this is for error prevention on friday-afternoon-products :D
// you can set it to 0 if you dare, but there is no warranty that it will still work
#define DPS310__BUSYTIME_FAILSAFE 10U
#define DPS310__MAX_BUSYTIME ((1000U - DPS310__BUSYTIME_FAILSAFE) * DPS__BUSYTIME_SCALING)

const int32_t DPS310::scaling_facts[DPS__NUM_OF_SCAL_FACTS] = {524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960};

DPS310::DPS310() {
	// TODO Auto-generated constructor stub

}

DPS310::~DPS310() {
	// TODO Auto-generated destructor stub
}

bool DPS310::init(I2C_Bus* i2cBus) {
	m_i2cBus = i2cBus;
	begin();
	return true;
}

void DPS310::standby(void)
{
	setOpMode(IDLE);
//	disableFIFO();
}

void DPS310::begin(void)
{
	reset();
	HAL_Delay(100);
	readID(m_productID, m_revisionID);
	HAL_Delay(50);

	//find out which temperature sensor is calibrated with coefficients...
	//...and use this sensor for temperature measurement
	m_tempSensor = setTempSensor();
	HAL_Delay(50);

	//read coefficients
	readcoeffs();
	HAL_Delay(50);

	//set to standby for further configuration
	standby();
	HAL_Delay(50);

	//set measurement precision and rate to standard values;
//	configTemp(MR_4, OSR_8);
//	HAL_Delay(50);
//	configPressure(MR_4, OSR_8);
//	HAL_Delay(50);

	//perform a first temperature measurement
	//the most recent temperature will be saved internally
	//and used for compensation when calculating pressure
//	float trash;
//	measureTempOnce(&trash, OSR_8);
//	HAL_Delay(50);
//
//	HAL_Delay(50);

//	disableFIFO();
#if USE_BARO_CONT_UPDATE
	enableFIFO();
	HAL_Delay(50);
#else
	disableFIFO();
#endif

	//make sure the DPS310 is in standby after initialization
	standby();
	HAL_Delay(50);

	// Fix IC with a fuse bit problem, which lead to a wrong temperature
	// Should not affect ICs without this problem
//	correctTemp();


	standby();

	HAL_Delay(50);

#if USE_BARO_CONT_UPDATE
	startMeasureBothCont(MR_1, OSR_8, MR_2, OSR_16);
#endif

}

void DPS310::readID(uint8_t& prodID, uint8_t& revID) {
	uint8_t data = readByte((uint8_t)REG_ID);

	prodID = (data >> 4) & 0x0F;
	revID = data & 0x0F;
}

void DPS310::reset() {
	writeByte((uint8_t)REG_RESET, 0x09);
}

void DPS310::readcoeffs(void)
{
	// TODO: remove magic number
	uint8_t buffer[18];

	//read COEF registers to buffer
	readRegister((uint8_t)REG_COEF, buffer, 18);

	//compose coefficients from buffer content
	m_c0Half = ((uint32_t)buffer[0] << 4) | (((uint32_t)buffer[1] >> 4) & 0x0F);
	getTwosComplement(&m_c0Half, 12);
	//c0 is only used as c0*0.5, so c0_half is calculated immediately
	m_c0Half = m_c0Half / 2U;

	//now do the same thing for all other coefficients
	m_c1 = (((uint32_t)buffer[1] & 0x0F) << 8) | (uint32_t)buffer[2];
	getTwosComplement(&m_c1, 12);
	m_c00 = ((uint32_t)buffer[3] << 12) | ((uint32_t)buffer[4] << 4) | (((uint32_t)buffer[5] >> 4) & 0x0F);
	getTwosComplement(&m_c00, 20);
	m_c10 = (((uint32_t)buffer[5] & 0x0F) << 16) | ((uint32_t)buffer[6] << 8) | (uint32_t)buffer[7];
	getTwosComplement(&m_c10, 20);

	m_c01 = ((uint32_t)buffer[8] << 8) | (uint32_t)buffer[9];
	getTwosComplement(&m_c01, 16);

	m_c11 = ((uint32_t)buffer[10] << 8) | (uint32_t)buffer[11];
	getTwosComplement(&m_c11, 16);
	m_c20 = ((uint32_t)buffer[12] << 8) | (uint32_t)buffer[13];
	getTwosComplement(&m_c20, 16);
	m_c21 = ((uint32_t)buffer[14] << 8) | (uint32_t)buffer[15];
	getTwosComplement(&m_c21, 16);
	m_c30 = ((uint32_t)buffer[16] << 8) | (uint32_t)buffer[17];
	getTwosComplement(&m_c30, 16);
}

float DPS310::calcTemp(int32_t raw)
{
	float temp = raw;

	//scale temperature according to scaling table and oversampling
	temp /= scaling_facts[m_tempOsr];
//	temp /= scaling_facts[m_tempOsr];

	//update last measured temperature
	//it will be used for pressure compensation
	m_lastTempScal = temp;

	//Calculate compensated temperature
	temp = m_c0Half + m_c1 * temp;

	return temp;
}

float DPS310::calcPressure(int32_t raw)
{
	float prs = raw;

	//scale pressure according to scaling table and oversampling
	prs /= scaling_facts[m_prsOsr];

	//Calculate compensated pressure
	prs = m_c00 + prs * (m_c10 + prs * (m_c20 + prs * m_c30)) + m_lastTempScal * (m_c01 + prs * (m_c11 + prs * m_c21));

	//return pressure
	return prs;
}

void DPS310::flushFIFO()
{
	uint8_t data;
	readRegister((uint8_t)REG_RESET, &data, 1);
	data |= 0x80;
	writeByte((uint8_t)REG_RESET, data);
}

bool DPS310::getContResults(float *tempBuffer, uint8_t &tempCount, float *prsBuffer, uint8_t &prsCount)
{
	tempCount = 0;
	prsCount = 0;
	uint8_t numPasses = 0;

//	//while FIFO is not empty
	while (!fifoEmpty())
	{
		if(++numPasses > 4) {
			MX_I2C1_Init();
//			reset();
//			setTempSensor();
//			enableFIFO();
//			standby();
//			startMeasureBothCont(MR_1, OSR_8, MR_2, OSR_16);
			break;
		}
		int32_t raw_result;
		float result;
		//read next result from FIFO
		eType type = getFIFOValue(&raw_result);
		switch (type)
		{
		case PRS: //temperature
			if(prsCount >= 2) {
				break;
			}
			result = calcPressure(raw_result);
			prsBuffer[prsCount] = result;
			prsCount++;
			break;
		case TEMP:
			if(tempCount >= 2) {
				break;
			}
			result = calcTemp(raw_result);
			tempBuffer[tempCount] = result;
			tempCount++;
			break;
		}
		if(prsCount >= 2) {
			flushFIFO();
			break;
		}
	}
	return prsCount > 0;
}

int8_t DPS310::measureTempOnce(float* temp, eOSRate osRate) {
	int8_t resp = startMeasureTempOnce(osRate);
	if(resp < 0) {
		return -1;
	}

	HAL_Delay(50);

	resp = getSingleResult(temp);
	return resp;
}


int8_t DPS310::startMeasureTempOnce(eOSRate oversamplingRate)
{
//	static uint8_t tries = 2;
//	eMode opMode = getOpMode();
//	if(opMode != IDLE && tries > 0) {
//		tries--;
//		return -1;
//	}
//	if(tries <= 0) {
//		tries = 0;
////		res
//	}
	if(m_opMode != IDLE) {
		return -1;
	}
	if (oversamplingRate != m_tempOsr)
	{
		//configuration of oversampling rate
		configTemp((eMRate)0, oversamplingRate);
		m_tempOsr = oversamplingRate;
	}

	//set device to temperature measuring mode
	setOpMode(CMD_TEMP);
	return 0;
}

int8_t DPS310::startMeasurePressureOnce(eOSRate oversamplingRate)
{
//	static uint8_t tries = 2;
//	eMode opMode = getOpMode();
//	if(m_opMode != IDLE && tries > 0) {
//		tries--;
//		return -1;
//	}
//	if(tries <= 0) {
//		tries = 0;
////		res
//	}
	if(m_opMode != IDLE) {
		return -1;
	}
	if (oversamplingRate != m_prsOsr)
	{
		//configuration of oversampling rate
		configPressure((eMRate)0, oversamplingRate);
		m_prsOsr = oversamplingRate;
	}

	//set device to temperature measuring mode
	setOpMode(CMD_PRS);
	return 0;
}

bool DPS310::startMeasureTempCont(eMRate measureRate, eOSRate oversamplingRate)
{

	setOpMode(IDLE);

	//abort if speed and precision are too high
	if (calcBusyTime(measureRate, oversamplingRate) >= DPS310__MAX_BUSYTIME)
	{
		return false;
	}
	//update precision and measuring rate
	configTemp(measureRate, oversamplingRate);

	//enable result FIFO
	enableFIFO();

	//Start measuring in background mode
	setOpMode(CONT_TMP);

	return true;
}

bool DPS310::startMeasurePressureCont(eMRate measureRate, eOSRate oversamplingRate)
{

	setOpMode(IDLE);

	//abort if speed and precision are too high
	if (calcBusyTime(measureRate, oversamplingRate) >= DPS310__MAX_BUSYTIME)
	{
		return false;
	}

	//update precision and measuring rate
	configPressure(measureRate, oversamplingRate);

	//enable result FIFO
	enableFIFO();

	//Start measuring in background mode
	setOpMode(CONT_PRS);

	return true;
}

bool DPS310::startMeasureBothCont(eMRate tempMr, eOSRate tempOsr, eMRate prsMr, eOSRate prsOsr)
{

	setOpMode(IDLE);

	//abort if speed and precision are too high
	if (calcBusyTime(tempMr, tempOsr) + calcBusyTime(prsMr, prsOsr) >= DPS310__MAX_BUSYTIME)
	{
		return false;
	}
	//update precision and measuring rate
	configTemp(tempMr, tempOsr);

	//update precision and measuring rate
	configPressure(prsMr, prsOsr);

	//enable result FIFO
//	enableFIFO();

	//Start measuring in background mode
	setOpMode(CONT_BOTH);

	return true;
}

void DPS310::correctTemp(void)
{
	writeByte(0x0E, 0xA5);
	writeByte(0x0F, 0x96);
	writeByte(0x62, 0x02);
	writeByte(0x0E, 0x00);
	writeByte(0x0F, 0x00);

	//perform a first temperature measurement (again)
	//the most recent temperature will be saved internally
	//and used for compensation when calculating pressure

	//todo
	float trash;
	measureTempOnce(&trash, OSR_8);
//
//	return DPS__SUCCEEDED;
}

uint8_t DPS310::setTempSensor() {
	uint8_t tmpSrc = readByte((uint8_t)REG_COEF_SRCE);
	tmpSrc &= 0x80;
	uint8_t cfg = readByte((uint8_t)REG_TMP_CFG);
	cfg =  tmpSrc > 0 ? cfg | 0x80 : cfg & 0x7F;
	writeByte((uint8_t)REG_TMP_CFG, cfg);
	return tmpSrc > 0 ? 1 : 0;
}

DPS310::eMode DPS310::getOpMode()
{
	uint8_t opMode;
	readRegister((uint8_t)REG_MEAS_CFG, &opMode, 1);
	return (eMode)(opMode & 0x07);
}


void DPS310::setOpMode(eMode opMode)
{
	writeRegister((uint8_t)REG_MEAS_CFG, (uint8_t*)&opMode, 1);
	m_opMode = opMode;
}

void DPS310::configTemp(eMRate tempMr, eOSRate tempOsr)
{
	uint8_t cfg = readByte((uint8_t)REG_TMP_CFG);
	cfg &= 0x80;
	uint8_t data = (uint8_t)tempMr << 4 | (uint8_t)tempOsr | cfg;
	writeRegister((uint8_t)REG_TMP_CFG, &data, 1);
	m_tempMr = tempMr;
	m_tempOsr = tempOsr;
}

void DPS310::configPressure(eMRate prsMr, eOSRate prsOsr)
{
	uint8_t data = (uint8_t)prsMr << 4 | (uint8_t)prsOsr;
	writeRegister((uint8_t)REG_PSR_CFG, &data, 1);
	m_prsMr = prsMr;
	m_prsOsr = prsOsr;
}

uint16_t DPS310::calcBusyTime(uint16_t mr, uint16_t osr)
{
	//formula from datasheet (optimized)
	return ((uint32_t)20U << mr) + ((uint32_t)16U << (osr + mr));
}

void DPS310::getTwosComplement(int32_t *raw, uint8_t length)
{
	if (*raw & ((uint32_t)1 << (length - 1)))
	{
		*raw -= (uint32_t)1 << length;
	}
}

int32_t DPS310::getRawResult(eType type)
{
	uint8_t reg = 0;
	switch(type) {
		case PRS:
			reg = REG_PSR;
			break;
		case TEMP:
			reg = REG_TEMP;
			break;
	}
	uint8_t buffer[3] = {0,0,0};
	readRegister((uint8_t)reg,buffer,3);

	int32_t raw = (uint32_t)buffer[0] << 16 | (uint32_t)buffer[1] << 8 | (uint32_t)buffer[2];
	getTwosComplement(&raw, 24);
	return raw;
}

int8_t DPS310::getSingleResult(float* result) {
	uint8_t measCfg = readByte((uint8_t)REG_MEAS_CFG);

//	uint8_t mode = measCfg & 0x07;
//	if(mode < 1 || mode > 2) {
//		return -1;
//	}
//
//	eType type = (mode == 1) ? PRS : TEMP;
	int32_t raw_val;
	switch(m_opMode){
		case CMD_PRS:
			if((measCfg & 0x10) == 0) {
				return -1;
			}
			raw_val = getRawResult(PRS);
			*result = calcPressure(raw_val);
			break;
		case CMD_TEMP:
			if((measCfg & 0x20) == 0) {
				return -1;
			}
			raw_val = getRawResult(TEMP);
			*result = calcTemp(raw_val);
			break;
		default:
			standby();
			return -1;

	}

	standby();
	return 0;
}


DPS310::eType DPS310::getSingleResult(float* prsResult, float* tmpResult) {
	uint8_t measCfg = readByte((uint8_t)REG_MEAS_CFG);
	int32_t raw_val;

	eType type = UNKNOWN;

	if(measCfg & 0x10) {
		raw_val = getRawResult(PRS);
		*prsResult = calcPressure(raw_val);
		type = PRS;
	}

	if(measCfg & 0x20) {
		raw_val = getRawResult(TEMP);
		*tmpResult = calcTemp(raw_val);
		type = TEMP;
	}



	if(type != UNKNOWN) {
		standby();
	}

	return type;
}

DPS310::eType DPS310::getFIFOValue(int32_t* value) {
	uint8_t buffer[3] = {0,0,0};
	uint8_t reg = REG_PSR;
	readRegister((uint8_t)reg,buffer,3);

	*value = (uint32_t)buffer[0] << 16 | (uint32_t)buffer[1] << 8 | (uint32_t)buffer[2];
	getTwosComplement(value, 24);
	return (DPS310::eType)(buffer[2] & 0x01) ? PRS : TEMP;
}

void DPS310::enableFIFO()
{
	uint8_t regValue;
	readRegister(REG_CFG_REG, &regValue, 1);

	regValue |= 0x0E;
	writeRegister(REG_CFG_REG, &regValue, 1);
}

void DPS310::disableFIFO()
{
	uint8_t regValue;
	readRegister(REG_CFG_REG, &regValue, 1);

	regValue &= 0xF1;
	writeRegister(REG_CFG_REG, &regValue, 1);
}

bool DPS310::fifoFull() {
	uint8_t data = readByte(REG_FIFO_STS);
	return (data & 0x02) > 0;
}

bool DPS310::fifoEmpty() {
	uint8_t data = readByte(REG_FIFO_STS);
	return (data & 0x01) > 0;
}



void DPS310::readRegister(uint8_t reg, uint8_t* data, uint16_t len) {
	m_i2cBus->mem_read(m_devAddr, reg, data, len);
}

void DPS310::writeRegister(uint8_t reg, uint8_t* data, uint16_t len) {
	m_i2cBus->mem_write(m_devAddr, reg, data, len);
}

