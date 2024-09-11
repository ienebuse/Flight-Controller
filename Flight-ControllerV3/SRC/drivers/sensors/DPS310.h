/*
 * DPS3.h
 *
 *  Created on: Jul 30, 2024
 *      Author: Ikenna
 */



#include <I2CBus.h>

#include <typedefs.h>
#include <TimeTick.h>
#include <i2c.h>

#ifndef DRIVERS_SENSORS_DPS3_H_
#define DRIVERS_SENSORS_DPS3_H_

#define DPS310_ADDRESS		0x76
#define DPS__NUM_OF_SCAL_FACTS 8

typedef struct {
	timetick_us acqTimeUs = 0;
	uint8_t acqState = 0;
	float altitude = 0;
	float temperature = 0;
	bool altAvailable = false;
}AltData;

class DPS310 {
public:

    enum eType {
        PRS = 0, // pressure value
        TEMP,    // temperature value
		UNKNOWN,
    };

    enum eMRate {
    	MR_1,
		MR_2,
		MR_4,
		MR_8,
		MR_16,
		MR_32,
		MR_64,
		MR_128
    };

    enum eOSRate {
    	OSR_1,
		OSR_2,
		OSR_4,
		OSR_8,
		OSR_16,
		OSR_32,
		OSR_64,
		OSR_128
    };

    /**
        @brief Operating mode.

    */
    enum eMode {
        IDLE = 0x00,
        CMD_PRS = 0x01,
        CMD_TEMP = 0x02,
        CMD_BOTH = 0x03, // only for DPS422
        CONT_PRS = 0x05,
        CONT_TMP = 0x06,
        CONT_BOTH = 0x07
    };



    /**
        @brief registers for configuration and flags; these are the same for both 310 and 422, might need to be adapted for future sensors

    */
    enum eConfig_Registers {
        TEMP_MR = 0, // temperature measure rate
        TEMP_OSR,    // temperature measurement resolution
        PRS_MR,      // pressure measure rate
        PRS_OSR,     // pressure measurement resolution
        MSR_CTRL,    // measurement control
        FIFO_EN,

        TEMP_RDY,
        PRS_RDY,
        INT_FLAG_FIFO,
        INT_FLAG_TEMP,
        INT_FLAG_PRS,
    };

    enum eRegister {
    	REG_PSR = 0x00,
		REG_TEMP = 0x03,
		REG_PSR_CFG = 0x06,
		REG_TMP_CFG,
		REG_MEAS_CFG,
		REG_CFG_REG,
		REG_INT_STS,
		REG_FIFO_STS,
		REG_RESET,
		REG_ID,
		REG_COEF = 0x10,
		REG_COEF_SRCE = 0x28

    };

	DPS310();
	virtual ~DPS310();

	bool init(I2C_Bus* i2cBus);

	void begin(void);

	void reset();

	void readcoeffs(void);

	void flushFIFO();

	float calcTemp(int32_t raw);

	float calcPressure(int32_t raw);

	bool getContResults(float *tempBuffer, uint8_t &tempCount, float *prsBuffer, uint8_t &prsCount);

	void standby(void);

	int8_t measureTempOnce(float* temp, eOSRate osRate);

	int8_t startMeasureTempOnce(eOSRate oversamplingRate);

	int8_t startMeasurePressureOnce(eOSRate oversamplingRate);

	bool startMeasureTempCont(eMRate measureRate, eOSRate oversamplingRate);

	bool startMeasurePressureCont(eMRate measureRate, eOSRate oversamplingRate);

	bool startMeasureBothCont(eMRate tempMr, eOSRate tempOsr, eMRate prsMr, eOSRate prsOsr);

	eMode getOpMode();

	void setOpMode(eMode opMode);

	void configTemp(eMRate tempMr, eOSRate tempOsr);

	void configPressure(eMRate prsMr, eOSRate prsOsr);

	void readID(uint8_t& prodID, uint8_t& revID);

	inline AltData getContAltitude() {
		uint8_t count = 2;
		uint8_t pressureCount = 2;
		float pressure[count];
		uint8_t temperatureCount = 2;
		float temperature[count];

//		if(m_Count < 6) {
//			m_Count++;
//		}
		for(int i = 0; i < count; i++) {
			pressure[i] = 0;
			temperature[i] = 0;
		}
		bool resp = getContResults(temperature, temperatureCount, pressure, pressureCount);
		if(resp && pressureCount >= 1 && pressureCount <= 2) {
			m_altData.altAvailable = true;
			m_altData.altitude = 44330 * (1.0 - pow((pressure[pressureCount-1]) / SEA_LEVEL_hPA, 0.1903));
			return m_altData;
		}
		else {
			m_altData.altAvailable = false;
		}

		return m_altData;
	}


	inline AltData getAltitude() {
		if(m_altData.acqState == 0) {
			startMeasureTempOnce(OSR_4);
			m_altData.acqState = 1;
			m_altData.acqTimeUs = 20000;
			m_altData.altAvailable = false;
		}
		else if(m_altData.acqState == 1){
			float pressure;
			m_altData.altAvailable = false;
			eType type = getSingleResult(&pressure, &m_altData.temperature);
			if(type == TEMP) {
				m_altData.acqState = 2;
				m_altData.acqTimeUs = 5000;
			}
			else if(type == PRS) {
				m_altData.acqTimeUs = 5000;
				float altitude = 44330 * (1.0 - pow((pressure) / SEA_LEVEL_hPA, 0.1903));
				m_altData.altitude = altitude;
//				if(abs(altitude - m_lastAltitude) < 100) {
//					m_altData.altitude = altitude;
//					m_lastAltitude = altitude;
//				}
				m_altData.altAvailable = true;
				++m_prsCount;
				if(m_prsCount == mc_prsTmpRatio) {
					m_altData.acqState = 0;
					m_prsCount = 0;
				}
				else {
					m_altData.acqState = 2;
				}
			}
			else {
				MX_I2C1_Init();
				standby();
				m_altData.acqState = 0;
//				++errorCount;
//
//				if(errorCount >= 5) {
//					errorCount = 0;
//					MX_I2C1_Init();
////					reset();
//					m_altData.acqState = 3;
//					m_altData.acqTimeUs = 500000;
//				}
//				else {
//					m_altData.acqTimeUs = 50000;
//				}
			}
		}
		else if(m_altData.acqState == 2) {
			startMeasurePressureOnce(OSR_8);
			m_altData.acqState = 1;
			m_altData.acqTimeUs = 80000;
			m_altData.altAvailable = false;
		}
//		else if(m_altData.acqState == 3) {
//			setTempSensor();
//			standby();
//			configTemp(MR_4, OSR_8);
//			configPressure(MR_4, OSR_8);
//
//			startMeasureTempOnce(OSR_8);
//
////			float trash;
////			measureTempOnce(&trash, OSR_8);
////			correctTemp();
//
//			m_altData.acqState = 4;
//			m_altData.acqTimeUs = 50000;
//			m_altData.altAvailable = false;
//		}
//		else if(m_altData.acqState == 4) {
//			float pressure;
//			m_altData.altAvailable = false;
//			getSingleResult(&pressure, &m_altData.temperature);
//			correctTemp();
//			m_altData.acqState = 0;
//			m_altData.acqTimeUs = 50000;
//		}

	  return m_altData;
	}

 	I2C_Bus* m_i2cBus;
	uint8_t m_devAddr{DPS310_ADDRESS};
	//compensation coefficients
	int32_t m_c0Half;
	int32_t m_c1;

	int32_t m_c00;
	int32_t m_c10;
	int32_t m_c01;
	int32_t m_c11;
	int32_t m_c20;
	int32_t m_c21;
	int32_t m_c30;

	//settings
	uint8_t m_tempMr;
	uint8_t m_tempOsr;
	uint8_t m_prsMr;
	uint8_t m_prsOsr;

	eMode m_opMode;
	uint8_t errorCount = 0;

	static const int32_t scaling_facts[DPS__NUM_OF_SCAL_FACTS];

	uint8_t m_productID;
	uint8_t m_revisionID;

	// last measured scaled temperature (necessary for pressure compensation)
	float m_lastTempScal;
	 uint8_t m_tempSensor;


	AltData m_altData;
	static bool firstMeasurement;
	float m_lastAltitude = 0;
	uint8_t m_Count = 0;
	const uint8_t mc_prsTmpRatio = 10;
	uint8_t m_prsCount = 0;

	void correctTemp(void);

	uint8_t setTempSensor();

	uint16_t calcBusyTime(uint16_t mr, uint16_t osr);

    void getTwosComplement(int32_t *raw, uint8_t length);

    int32_t getRawResult(eType reg);

    int8_t getSingleResult(float* result);

    eType getSingleResult(float* prsResult, float* tmpResult);

    eType getFIFOValue(int32_t* value);

    void enableFIFO();

    void disableFIFO();

    bool fifoFull();

    bool fifoEmpty();

    inline void writeByte(uint8_t reg, uint8_t data) {
    	writeRegister(reg, &data, 1);
    }

    inline uint8_t readByte(uint8_t reg) {
    	uint8_t data;
		readRegister(reg, &data, 1);
		return data;
	}

    void readRegister(uint8_t reg, uint8_t* data, uint16_t len);

    void writeRegister(uint8_t reg, uint8_t* data, uint16_t len);
};

#endif /* DRIVERS_SENSORS_DPS3_H_ */
