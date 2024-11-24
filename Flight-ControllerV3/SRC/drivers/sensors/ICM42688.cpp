/*!
 * @file DFRobot_ICM42688.cpp
 * @brief Define basic structure of DFRobot_ICM42688 class, the implementation of basic method
 * @copyright	Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [yangfeng]<feng.yang@dfrobot.com>
 * @version V1.0
 * @date 2021-05-13
 * @url  https://github.com/DFRobot/DFRobot_ICM42688
 */
#include <math.h>
//#include <cmsis_os2.h>
#include <drivers/sensors/ICM42688.h>
#include <TimeTick.h>
#include <Configurator.h>

static Config config;


ICM42688::ICM42688()
{
  accelConfig0.accelODR = 6;
  accelConfig0.accelFsSel = 0;
  gyroConfig0.gyroODR = 6;
  gyroConfig0.gyroFsSel = 0;
  _gyroRange = 4000 / 65535.0;
  _accelRange = 0.488f;
  FIFOMode = false;
}

int ICM42688::init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin, uint32_t SPI_HS_CLK)
{
	config = Configurator::getConfig();
	_useSPIHS = false;
	SPI_HS_CLOCK = SPI_HS_CLK;

	m_hSpi = hspi;
	m_csPort = csPort;
	m_csPin = csPin;

	gxf.init(100, 500);
	gyf.init(100, 500);
	gzf.init(100, 500);

	axf.init(100, 500);
	ayf.init(100, 500);
	azf.init(100, 500);

  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
  TimeTick::delay_us(1000);
  uint8_t id = 0;
  readReg(ICM42688_WHO_AM_I, &id, 1);

//  if (id != DFRobot_ICM42688_ID)
//  {
//    return ERR_IC_VERSION;
//  }
  uint8_t reset = 0;
  writeReg(ICM42688_DEVICE_CONFIG, &reset, 1);
  TimeTick::delay_us(10000);

//  for(int i = 0; i < 20; i++) {
//	  HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
//	  TimeTick::delay_us(100000);
//  }



//  setODRAndFSR(/* who= */GYRO,/* ODR= */ODR_1KHZ, /* FSR = */FSR_0);
//  setODRAndFSR(/* who= */ACCEL,/* ODR= */ODR_1KHZ, /* FSR = */FSR_0);


  setODRAndFSR(/* who= */GYRO,/* ODR= */ODR_500HZ, /* FSR = */FSR_0);
  TimeTick::delay_us(1000);
  setODRAndFSR(/* who= */ACCEL,/* ODR= */ODR_500HZ, /* FSR = */FSR_0);
  TimeTick::delay_us(1000);
  setAAFBandwidth(ALL, 7);
  TimeTick::delay_us(1000);
  setAAF(ALL, true);
  TimeTick::delay_us(1000);
  setUIFilter(ALL, 2, 1);
  TimeTick::delay_us(1000);
  startTempMeasure();
  startGyroMeasure(/* mode= */LN_MODE);
  TimeTick::delay_us(1000);
  startAccelMeasure(/* mode= */LN_MODE);

//  calibrateGyro();

  m_initialized = true;
  return ERR_OK;
}

/* estimates the gyro biases */
void ICM42688::calibrateGyro() {

	float _gyroBD[3];
	uint16_t NUM_CALIB_SAMPLES = 5000;
	Vector_t<float> gyroData;

	for(int i = 0; i < 10; i++) {
		HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
		TimeTick::delay_us(100000);
	}


	if(config.UseGyroFilter) {
		for(int i = 0; i < 1000; i++) {
			getGyroData();
			TimeTick::delay_us(IMU_SAMPLING_PERIOD_US);
		}
	}
  // set at a lower range (more resolution) since IMU not moving
//  const GyroFS current_fssel = _gyroFS;
//  if (setGyroFS(dps250) < 0) return -1;

  // take samples and find bias
  _gyroBD[0] = 0;
  _gyroBD[1] = 0;
  _gyroBD[2] = 0;
  for (size_t i=0; i < NUM_CALIB_SAMPLES; i++) {
    gyroData = getGyroData();
    _gyroBD[0] += gyroData.x;
    _gyroBD[1] += gyroData.y;
    _gyroBD[2] += gyroData.z;
    TimeTick::delay_us(IMU_SAMPLING_PERIOD_US);
  }
  m_gyrB[0] = _gyroBD[0]/NUM_CALIB_SAMPLES;
  m_gyrB[1] = _gyroBD[1]/NUM_CALIB_SAMPLES;
  m_gyrB[2] = _gyroBD[2]/NUM_CALIB_SAMPLES;

  // recover the full scale setting
//  if (setGyroFS(current_fssel) < 0) return -4;
//  return 1;
}

float ICM42688::getTemperature(void)
{
  float value;
  if (FIFOMode)
  {
    value = (_temp / 2.07) + 25;
  }
  else
  {
    uint8_t data[2];
    int16_t value2;
    readReg(ICM42688_TEMP_DATA1, data, 2);
    value2 = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
    value = value2 / 132.48 + 25;
  }
  return value;
}

Vector_t<float> ICM42688::getAccelData(void) {
	Vector_t<float> accData;
	  if (FIFOMode)
	  {
		  accData.x = _accelX;
		  accData.y = _accelY;
		  accData.z = _accelZ;
	  }
	  else
	  {
	    uint8_t data[6];
	    readReg(ICM42688_ACCEL_DATA_X1, data, 6);
	    accData.x = (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
	    accData.y = (int16_t)(((uint16_t)data[2] << 8) | (uint16_t)data[3]);
	    accData.z = (int16_t)(((uint16_t)data[4] << 8) | (uint16_t)data[5]);
	  }
//	  accData.x  = ((accData.x *_accelRange) - m_calibData.accB[0]) * m_calibData.accS[0];
//	  accData.y  = ((accData.y *_accelRange) - m_calibData.accB[1]) * m_calibData.accS[1];
//	  accData.z  = ((accData.z *_accelRange) - m_calibData.accB[2]) * m_calibData.accS[2];

//	  accData.x  *= _accelRange;
//	  accData.y  *= _accelRange;
//	  accData.z  *= _accelRange;


	  if(config.UseAccelFilter) {
		  accData.x  = axf.apply(accData.x * _accelRange);
		  accData.y  = ayf.apply(accData.y * _accelRange);
		  accData.z  = azf.apply(accData.z * _accelRange);
	  }
	  else {
		  accData.x  = (accData.x * _accelRange);
		  accData.y  = (accData.y * _accelRange);
		  accData.z  = (accData.z * _accelRange);
	  }

#if !PROTOTYPE
	  accData.x = -accData.x;
	  accData.z = -accData.z;
#endif

	  return accData;
}

float ICM42688::getAccelDataX(void)
{
  float value;
  if (FIFOMode)
  {
    value = _accelX;
  }
  else
  {
    uint8_t data[2];
    readReg(ICM42688_ACCEL_DATA_X1, data, 2);
    int16_t value1 = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
    value = value1;
  }
  return value * _accelRange;
}

float ICM42688::getAccelDataY(void)
{
  float value;
  if (FIFOMode)
  {
    value = _accelY;
  }
  else
  {
    uint8_t data[2];
    readReg(ICM42688_ACCEL_DATA_Y1, data, 2);
    int16_t value1 = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
    value = value1;
  }
  return value * _accelRange;
}

float ICM42688::getAccelDataZ(void)
{
  float value;
  if (FIFOMode)
  {
    value = _accelZ;
  }
  else
  {
    uint8_t data[2];
    readReg(ICM42688_ACCEL_DATA_Z1, data, 2);
    int16_t value1 = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
    value = value1;
  }
  return value * _accelRange;
}

Vector_t<float> ICM42688::getGyroData(void) {
	Vector_t <float> gyroData;
	  if (FIFOMode)
	  {
		  gyroData.x = _gyroX;
		  gyroData.y = _gyroY;
		  gyroData.z = _gyroZ;
	  }
	  else
	  {
	    uint8_t data[6];
	    readReg(ICM42688_GYRO_DATA_X1, data, 6);
	    gyroData.x = (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
	    gyroData.y = (int16_t)(((uint16_t)data[2] << 8) | (uint16_t)data[3]);
	    gyroData.z = (int16_t)(((uint16_t)data[4] << 8) | (uint16_t)data[5]);
	  }

	  if(config.UseGyroFilter) {
		  gyroData.x  = gxf.apply(gyroData.x * _gyroRange)  - m_gyrB[0];
		  gyroData.y  = gyf.apply(gyroData.y * _gyroRange)  - m_gyrB[1];
		  gyroData.z  = gzf.apply(gyroData.z * _gyroRange)  - m_gyrB[2];
	  }
	  else {
		  gyroData.x  = (gyroData.x * _gyroRange)  - m_gyrB[0];
		  gyroData.y  = (gyroData.y * _gyroRange)  - m_gyrB[1];
		  gyroData.z  = (gyroData.z * _gyroRange)  - m_gyrB[2];
	  }

#if !PROTOTYPE
	  gyroData.x = -gyroData.x;
	  gyroData.z = -gyroData.z;
#endif
	  return gyroData;
}

float ICM42688::getGyroDataX(void)
{
  float value;
  if (FIFOMode)
  {
    value = _gyroX;
  }
  else
  {
    uint8_t data[2];
    readReg(ICM42688_GYRO_DATA_X1, data, 2);
    int16_t value1 = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
    value = value1;
  }
  return value * _gyroRange;
}

float ICM42688::getGyroDataY(void)
{
  float value;
  if (FIFOMode)
  {
    value = _gyroY;
  }
  else
  {
    uint8_t data[2];
    readReg(ICM42688_GYRO_DATA_Y1, data, 2);
    int16_t value1 = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
    value = value1;
  }
  return value * _gyroRange;
}

float ICM42688::getGyroDataZ(void)
{
  float value;
  if (FIFOMode)
  {
    value = _gyroZ;
  }
  else
  {
    uint8_t data[2];
    readReg(ICM42688_GYRO_DATA_Z1, data, 2);
    int16_t value1 = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
    value = value1;
  }
  return value * _gyroRange;
}

void ICM42688::tapDetectionInit(uint8_t accelMode)
{
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
  if (accelMode == 0)
  {
    accelConfig0.accelODR = 15;
    writeReg(ICM42688_ACCEL_CONFIG0, &accelConfig0, 1);
    PWRMgmt0.accelMode = 2;
    writeReg(ICM42688_PWR_MGMT0, &PWRMgmt0, 1);
    TimeTick::delay_us(1000);
    INTFConfig1.accelLpClkSel = 0;
    writeReg(ICM42688_INTF_CONFIG1, &INTFConfig1, 1);
    accelConfig1.accelUIFiltORD = 2;
    writeReg(ICM42688_ACCEL_CONFIG1, &accelConfig1, 1);
    gyroAccelConfig0.accelUIFiltBW = 0;
    writeReg(ICM42688_GYRO_ACCEL_CONFIG0, &gyroAccelConfig0, 1);
  }
  else if (accelMode == 1)
  {
    accelConfig0.accelODR = 6;
    writeReg(ICM42688_ACCEL_CONFIG0, &accelConfig0, 1);
    PWRMgmt0.accelMode = 3;
    writeReg(ICM42688_PWR_MGMT0, &PWRMgmt0, 1);
    TimeTick::delay_us(1000);
    accelConfig1.accelUIFiltORD = 2;
    writeReg(ICM42688_ACCEL_CONFIG1, &accelConfig1, 1);
    gyroAccelConfig0.accelUIFiltBW = 0;
    writeReg(ICM42688_GYRO_ACCEL_CONFIG0, &gyroAccelConfig0, 1);
  }
  else
  {
    //    DBG("accelMode invalid !");
    return;
  }
  TimeTick::delay_us(1000);
  bank = 4;
  writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
  APEXConfig8.tapTmin = 3;
  APEXConfig8.tapTavg = 3;
  APEXConfig8.tapTmax = 2;
  writeReg(ICM42688_APEX_CONFIG8, &APEXConfig8, 1);
  APEXConfig7.tapMinJerkThr = 17;
  APEXConfig7.tapMaxPeakTol = 1;
  writeReg(ICM42688_APEX_CONFIG7, &APEXConfig7, 1);
  TimeTick::delay_us(1000);
  INTSource.tapDetIntEn = 1;
  if (_INTPin == 1)
  {
    writeReg(ICM42688_INT_SOURCE6, &INTSource, 1);
  }
  else
  {
    writeReg(ICM42688_INT_SOURCE7, &INTSource, 1);
  }
  TimeTick::delay_us(50000);
  bank = 0;
  writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
  APEXConfig0.tapEnable = 1;
  writeReg(ICM42688_APEX_CONFIG0, &APEXConfig0, 1);
}
void ICM42688::getTapInformation()
{
  uint8_t data;
  readReg(ICM42688_APEX_DATA4, &data, 1);
  _tapNum = data & 0x18;
  _tapAxis = data & 0x06;
  _tapDir = data & 0x01;
}
uint8_t ICM42688::numberOfTap()
{
  return _tapNum;
}
uint8_t ICM42688::axisOfTap()
{
  return _tapAxis;
}
void ICM42688::wakeOnMotionInit()
{
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
  accelConfig0.accelODR = 9;
  writeReg(ICM42688_ACCEL_CONFIG0, &accelConfig0, 1);
  PWRMgmt0.accelMode = 2;
  writeReg(ICM42688_PWR_MGMT0, &PWRMgmt0, 1);
  TimeTick::delay_us(1000);
  INTFConfig1.accelLpClkSel = 0;
  writeReg(ICM42688_INTF_CONFIG1, &INTFConfig1, 1);
  TimeTick::delay_us(1000);
}
void ICM42688::setWOMTh(uint8_t axis, uint8_t threshold)
{
  uint8_t bank = 4;
  writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
  uint8_t womValue = threshold;
  if (axis == X_AXIS)
  {
    writeReg(ICM42688_ACCEL_WOM_X_THR, &womValue, 1);
  }
  else if (axis == Y_AXIS)
  {
    writeReg(ICM42688_ACCEL_WOM_Y_THR, &womValue, 1);
  }
  else if (axis == Z_AXIS)
  {
    writeReg(ICM42688_ACCEL_WOM_Z_THR, &womValue, 1);
  }
  else if (axis == ALL)
  {
    writeReg(ICM42688_ACCEL_WOM_X_THR, &womValue, 1);
    writeReg(ICM42688_ACCEL_WOM_Y_THR, &womValue, 1);
    writeReg(ICM42688_ACCEL_WOM_Z_THR, &womValue, 1);
  }
  TimeTick::delay_us(1000);
  bank = 0;
  writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
}
void ICM42688::setWOMInterrupt(uint8_t axis)
{
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
  if (_INTPin == 1)
  {
    writeReg(ICM42688_INT_SOURCE1, &axis, 1);
  }
  else
  {
    writeReg(ICM42688_INT_SOURCE4, &axis, 1);
  }
  TimeTick::delay_us(50000);
  SMDConfig.SMDMode = 1;
  SMDConfig.WOMMode = 1;
  SMDConfig.WOMIntMode = 0;
  writeReg(ICM42688_SMD_CONFIG, &SMDConfig, 1);
}
void ICM42688::enableSMDInterrupt(uint8_t mode)
{
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
  uint8_t INT = 1 << 3;
  if (mode != 0)
  {
    if (_INTPin == 1)
    {
      writeReg(ICM42688_INT_SOURCE1, &INT, 1);
    }
    else
    {
      writeReg(ICM42688_INT_SOURCE4, &INT, 1);
    }
  }
  TimeTick::delay_us(50000);
  SMDConfig.SMDMode = mode;
  SMDConfig.WOMMode = 1;
  SMDConfig.WOMIntMode = 0;
  writeReg(ICM42688_SMD_CONFIG, &SMDConfig, 1);
}

uint8_t ICM42688::readInterruptStatus(uint8_t reg)
{
  uint8_t bank = 0;
  uint8_t status = 0;
  writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
  readReg(reg, &status, 1);
  return status;
}

bool ICM42688::setODRAndFSR(uint8_t who, uint8_t ODR, uint8_t FSR)
{
  bool ret = true;
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
  if (who == GYRO)
  {
    if (ODR > ODR_12_5KHZ || FSR > FSR_7)
    {
      ret = false;
    }
    else
    {
      gyroConfig0.gyroODR = ODR;
      gyroConfig0.gyroFsSel = FSR;
      writeReg(ICM42688_GYRO_CONFIG0, &gyroConfig0, 1);
      _gyroRange = (2000.0f / static_cast<float>(1 << FSR)) / 32768.0f;
//      switch (FSR)
//      {
//      case FSR_0:
//        _gyroRange = 4000 / 65535.0;
//        break;
//      case FSR_1:
//        _gyroRange = 2000 / 65535.0;
//        break;
//      case FSR_2:
//        _gyroRange = 1000 / 65535.0;
//        break;
//      case FSR_3:
//        _gyroRange = 500 / 65535.0;
//        break;
//      case FSR_4:
//        _gyroRange = 250 / 65535.0;
//        break;
//      case FSR_5:
//        _gyroRange = 125 / 65535.0;
//        break;
//      case FSR_6:
//        _gyroRange = 62.5 / 65535.0;
//        break;
//      case FSR_7:
//        _gyroRange = 31.25 / 65535.0;
//        break;
//      }
    }
  }
  else if (who == ACCEL)
  {
    if (ODR > ODR_500HZ || FSR > FSR_3)
    {
      ret = false;
    }
    else
    {
      accelConfig0.accelODR = ODR;
      accelConfig0.accelFsSel = FSR;
      writeReg(ICM42688_ACCEL_CONFIG0, &accelConfig0, 1);
      _accelRange = static_cast<float>(1 << (4 - FSR)) / 32768.0f;
//      switch (FSR)
//      {
//      case FSR_0:
//        _accelRange = 0.488f;
//        break;
//      case FSR_1:
//        _accelRange = 0.244f;
//        break;
//      case FSR_2:
//        _accelRange = 0.122f;
//        break;
//      case FSR_3:
//        _accelRange = 0.061f;
//        break;
//      }
    }
  }
  return ret;
}

void ICM42688::setFIFODataMode()
{
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
  FIFOConfig1.FIFOHiresEn = 0;
  FIFOConfig1.FIFOAccelEn = 1;
  FIFOConfig1.FIFOGyroEn = 1;
  FIFOConfig1.FIFOTempEn = 1;
  FIFOConfig1.FIFOTmstFsyncEn = 0;
  writeReg(ICM42688_FIFO_CONFIG1, &FIFOConfig1, 1);
}

void ICM42688::startFIFOMode()
{
  uint8_t bank = 0;
  FIFOMode = true;
  writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
  setFIFODataMode();
  uint8_t start = 1 << 6;
  writeReg(ICM42688_FIFO_CONFIG, &start, 1);
  getFIFOData();
}
void ICM42688::getFIFOData()
{
  uint8_t data[16];
  readReg(ICM42688_FIFO_DATA, data, 16);
  _accelX = (uint16_t)data[1] << 8 | (uint16_t)data[2];
  // DBG("_accelX");DBG(_accelX);
  _accelY = (uint16_t)data[3] << 8 | (uint16_t)data[4];
  // DBG("_accelY");DBG(_accelY);
  _accelZ = (uint16_t)data[5] << 8 | (uint16_t)data[6];
  // DBG("_accelZ");DBG(_accelZ);
  _gyroX = (uint16_t)data[7] << 8 | (uint16_t)data[8];
  // DBG("_gyroX");DBG(_gyroX);
  _gyroY = (uint16_t)data[9] << 8 | (uint16_t)data[10];
  // DBG("_gyroY");DBG(_gyroY);
  _gyroZ = (uint16_t)data[11] << 8 | (uint16_t)data[12];
  // DBG("_gyroZ");DBG(_gyroZ);
  _temp = (uint8_t)data[13];
  // DBG("_temp");DBG(data[13]);
}
void ICM42688::sotpFIFOMode()
{
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
  uint8_t start = 1 << 7;
  writeReg(ICM42688_FIFO_CONFIG, &start, 1);
}

void ICM42688::setINTMode(uint8_t INTPin, uint8_t INTmode, uint8_t INTPolarity, uint8_t INTDriveCircuit)
{
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
  if (INTPin == 1)
  {
    _INTPin = 1;
    INTConfig.INT1Mode = INTmode;
    INTConfig.INT1DriveCirCuit = INTDriveCircuit;
    INTConfig.INT1Polarity = INTPolarity;
  }
  else if (INTPin == 2)
  {
    _INTPin = 2;
    INTConfig.INT2Mode = INTmode;
    INTConfig.INT2DriveCirCuit = INTDriveCircuit;
    INTConfig.INT2Polarity = INTPolarity;
  }
  writeReg(ICM42688_INT_CONFIG, &INTConfig, 1);
}

void ICM42688::startTempMeasure()
{
  PWRMgmt0.tempDis = 0;
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
  writeReg(ICM42688_PWR_MGMT0, &PWRMgmt0, 1);
  TimeTick::delay_us(1000);
}
void ICM42688::startGyroMeasure(uint8_t mode)
{
  PWRMgmt0.gyroMode = mode;
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
  writeReg(ICM42688_PWR_MGMT0, &PWRMgmt0, 1);
  TimeTick::delay_us(1000);
}

void ICM42688::startAccelMeasure(uint8_t mode)
{
  PWRMgmt0.accelMode = mode;
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
  writeReg(ICM42688_PWR_MGMT0, &PWRMgmt0, 1);
  TimeTick::delay_us(10000);
}
void ICM42688::setGyroNotchFilterFHz(double freq, uint8_t axis)
{
  uint8_t bank = 1;
  writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
  double fdesired = freq * 1000;
  double coswz = cos(2 * 3.14 * fdesired / 32);
  int16_t nfCoswz;
  uint8_t nfCoswzSel;
  if (abs(coswz) <= 0.875)
  {
    nfCoswz = round(coswz * 256);
    nfCoswzSel = 0;
  }
  else
  {
    nfCoswzSel = 1;
    if (coswz > 0.875)
    {
      nfCoswz = round(8 * (1 - coswz) * 256);
    }
    else if (coswz < -0.875)
    {
      nfCoswz = round(-8 * (1 + coswz) * 256);
    }
  }
  if (axis == X_AXIS)
  {
    gyroConfigStatic9.gyroNFCoswzSelX = nfCoswzSel;
    gyroConfigStatic9.gyroNFCoswzX8 = nfCoswz >> 8;
    writeReg(ICM42688_GYRO_CONFIG_STATIC6, &nfCoswz, 1);
    writeReg(ICM42688_GYRO_CONFIG_STATIC9, &gyroConfigStatic9, 1);
  }
  else if (axis == Y_AXIS)
  {
    gyroConfigStatic9.gyroNFCoswzSelY = nfCoswzSel;
    gyroConfigStatic9.gyroNFCoswzY8 = nfCoswz >> 8;
    writeReg(ICM42688_GYRO_CONFIG_STATIC7, &nfCoswz, 1);
    writeReg(ICM42688_GYRO_CONFIG_STATIC9, &gyroConfigStatic9, 1);
  }
  else if (axis == Z_AXIS)
  {
    gyroConfigStatic9.gyroNFCoswzSelZ = nfCoswzSel;
    gyroConfigStatic9.gyroNFCoswzZ8 = nfCoswz >> 8;
    writeReg(ICM42688_GYRO_CONFIG_STATIC8, &nfCoswz, 1);
    writeReg(ICM42688_GYRO_CONFIG_STATIC9, &gyroConfigStatic9, 1);
  }
  else if (axis == ALL)
  {
    gyroConfigStatic9.gyroNFCoswzSelX = nfCoswzSel;
    gyroConfigStatic9.gyroNFCoswzX8 = nfCoswz >> 8;
    gyroConfigStatic9.gyroNFCoswzSelY = nfCoswzSel;
    gyroConfigStatic9.gyroNFCoswzY8 = nfCoswz >> 8;
    gyroConfigStatic9.gyroNFCoswzSelZ = nfCoswzSel;
    gyroConfigStatic9.gyroNFCoswzZ8 = nfCoswz >> 8;
    writeReg(ICM42688_GYRO_CONFIG_STATIC6, &nfCoswz, 1);
    writeReg(ICM42688_GYRO_CONFIG_STATIC7, &nfCoswz, 1);
    writeReg(ICM42688_GYRO_CONFIG_STATIC8, &nfCoswz, 1);
    writeReg(ICM42688_GYRO_CONFIG_STATIC9, &gyroConfigStatic9, 1);
  }
  bank = 0;
  writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
}

void ICM42688::setGyroNFbandwidth(uint8_t bw)
{
  uint8_t bank = 1;
  writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
  uint8_t bandWidth = (bw << 4) | 0x01;
  writeReg(ICM42688_GYRO_CONFIG_STATIC10, &bandWidth, 1);
  bank = 0;
  writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
}

void ICM42688::setGyroNotchFilter(bool mode)
{
  if (mode)
  {
    gyroConfigStatic2.gyroNFDis = 0;
  }
  else
  {
    gyroConfigStatic2.gyroNFDis = 1;
  }
  uint8_t bank = 1;
  writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
  writeReg(ICM42688_GYRO_CONFIG_STATIC2, &gyroConfigStatic2, 1);
  bank = 0;
  writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
}
void ICM42688::setAAFBandwidth(uint8_t who, uint8_t BWIndex)
{
  uint8_t bank = 0;
  uint16_t AAFDeltsqr = BWIndex * BWIndex;
  if (who == GYRO)
  {
    bank = 1;
    writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
    writeReg(ICM42688_GYRO_CONFIG_STATIC3, &BWIndex, 1);
    writeReg(ICM42688_GYRO_CONFIG_STATIC4, &AAFDeltsqr, 1);
    gyroConfigStatic5.gyroAAFDeltsqr = AAFDeltsqr >> 8;
    if (BWIndex == 1)
    {
      gyroConfigStatic5.gyroAAFBitshift = 15;
    }
    else if (BWIndex == 2)
    {
      gyroConfigStatic5.gyroAAFBitshift = 13;
    }
    else if (BWIndex == 3)
    {
      gyroConfigStatic5.gyroAAFBitshift = 12;
    }
    else if (BWIndex == 4)
    {
      gyroConfigStatic5.gyroAAFBitshift = 11;
    }
    else if (BWIndex == 5 || BWIndex == 6)
    {
      gyroConfigStatic5.gyroAAFBitshift = 10;
    }
    else if (BWIndex > 6 && BWIndex < 10)
    {
      gyroConfigStatic5.gyroAAFBitshift = 9;
    }
    else if (BWIndex > 9 && BWIndex < 14)
    {
      gyroConfigStatic5.gyroAAFBitshift = 8;
    }
    else if (BWIndex > 13 && BWIndex < 19)
    {
      gyroConfigStatic5.gyroAAFBitshift = 7;
    }
    else if (BWIndex > 18 && BWIndex < 27)
    {
      gyroConfigStatic5.gyroAAFBitshift = 6;
    }
    else if (BWIndex > 26 && BWIndex < 37)
    {
      gyroConfigStatic5.gyroAAFBitshift = 5;
    }
    else if (BWIndex > 36 && BWIndex < 53)
    {
      gyroConfigStatic5.gyroAAFBitshift = 4;
    }
    else if (BWIndex > 53 && BWIndex <= 63)
    {
      gyroConfigStatic5.gyroAAFBitshift = 3;
    }
    writeReg(ICM42688_GYRO_CONFIG_STATIC5, &gyroConfigStatic5, 1);
    bank = 0;
    writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
  }
  else if (who == ACCEL)
  {
    bank = 2;
    writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
    accelConfigStatic2.accelAAFDelt = BWIndex;
    writeReg(ICM42688_ACCEL_CONFIG_STATIC2, &accelConfigStatic2, 1);
    writeReg(ICM42688_ACCEL_CONFIG_STATIC3, &AAFDeltsqr, 1);
    accelConfigStatic4.accelAAFDeltsqr = AAFDeltsqr >> 8;
    if (BWIndex == 1)
    {
      accelConfigStatic4.accelAAFBitshift = 15;
    }
    else if (BWIndex == 2)
    {
      accelConfigStatic4.accelAAFBitshift = 13;
    }
    else if (BWIndex == 3)
    {
      accelConfigStatic4.accelAAFBitshift = 12;
    }
    else if (BWIndex == 4)
    {
      accelConfigStatic4.accelAAFBitshift = 11;
    }
    else if (BWIndex == 5 || BWIndex == 6)
    {
      accelConfigStatic4.accelAAFBitshift = 10;
    }
    else if (BWIndex > 6 && BWIndex < 10)
    {
      accelConfigStatic4.accelAAFBitshift = 9;
    }
    else if (BWIndex > 9 && BWIndex < 14)
    {
      accelConfigStatic4.accelAAFBitshift = 8;
    }
    else if (BWIndex > 13 && BWIndex < 19)
    {
      accelConfigStatic4.accelAAFBitshift = 7;
    }
    else if (BWIndex > 18 && BWIndex < 27)
    {
      accelConfigStatic4.accelAAFBitshift = 6;
    }
    else if (BWIndex > 26 && BWIndex < 37)
    {
      accelConfigStatic4.accelAAFBitshift = 5;
    }
    else if (BWIndex > 36 && BWIndex < 53)
    {
      accelConfigStatic4.accelAAFBitshift = 4;
    }
    else if (BWIndex > 53 && BWIndex <= 63)
    {
      accelConfigStatic4.accelAAFBitshift = 3;
    }
    writeReg(ICM42688_ACCEL_CONFIG_STATIC4, &accelConfigStatic4, 1);

    bank = 0;
    writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
  }
  else if (who == ALL)
  {
    bank = 1;
    writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
    writeReg(ICM42688_GYRO_CONFIG_STATIC3, &BWIndex, 1);
    writeReg(ICM42688_GYRO_CONFIG_STATIC4, &AAFDeltsqr, 1);
    gyroConfigStatic5.gyroAAFDeltsqr = AAFDeltsqr >> 8;
    if (BWIndex == 1)
    {
      gyroConfigStatic5.gyroAAFBitshift = 15;
    }
    else if (BWIndex == 2)
    {
      gyroConfigStatic5.gyroAAFBitshift = 13;
    }
    else if (BWIndex == 3)
    {
      gyroConfigStatic5.gyroAAFBitshift = 12;
    }
    else if (BWIndex == 4)
    {
      gyroConfigStatic5.gyroAAFBitshift = 11;
    }
    else if (BWIndex == 5 || BWIndex == 6)
    {
      gyroConfigStatic5.gyroAAFBitshift = 10;
    }
    else if (BWIndex > 6 && BWIndex < 10)
    {
      gyroConfigStatic5.gyroAAFBitshift = 9;
    }
    else if (BWIndex > 9 && BWIndex < 14)
    {
      gyroConfigStatic5.gyroAAFBitshift = 8;
    }
    else if (BWIndex > 13 && BWIndex < 19)
    {
      gyroConfigStatic5.gyroAAFBitshift = 7;
    }
    else if (BWIndex > 18 && BWIndex < 27)
    {
      gyroConfigStatic5.gyroAAFBitshift = 6;
    }
    else if (BWIndex > 26 && BWIndex < 37)
    {
      gyroConfigStatic5.gyroAAFBitshift = 5;
    }
    else if (BWIndex > 36 && BWIndex < 53)
    {
      gyroConfigStatic5.gyroAAFBitshift = 4;
    }
    else if (BWIndex > 53 && BWIndex <= 63)
    {
      gyroConfigStatic5.gyroAAFBitshift = 3;
    }
    writeReg(ICM42688_GYRO_CONFIG_STATIC5, &gyroConfigStatic5, 1);
    bank = 2;
    writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
    accelConfigStatic2.accelAAFDelt = BWIndex;
    writeReg(ICM42688_ACCEL_CONFIG_STATIC2, &accelConfigStatic2, 1);
    writeReg(ICM42688_ACCEL_CONFIG_STATIC3, &AAFDeltsqr, 1);
    accelConfigStatic4.accelAAFDeltsqr = AAFDeltsqr >> 8;
    if (BWIndex == 1)
    {
      accelConfigStatic4.accelAAFBitshift = 15;
    }
    else if (BWIndex == 2)
    {
      accelConfigStatic4.accelAAFBitshift = 13;
    }
    else if (BWIndex == 3)
    {
      accelConfigStatic4.accelAAFBitshift = 12;
    }
    else if (BWIndex == 4)
    {
      accelConfigStatic4.accelAAFBitshift = 11;
    }
    else if (BWIndex == 5 || BWIndex == 6)
    {
      accelConfigStatic4.accelAAFBitshift = 10;
    }
    else if (BWIndex > 6 && BWIndex < 10)
    {
      accelConfigStatic4.accelAAFBitshift = 9;
    }
    else if (BWIndex > 9 && BWIndex < 14)
    {
      accelConfigStatic4.accelAAFBitshift = 8;
    }
    else if (BWIndex > 13 && BWIndex < 19)
    {
      accelConfigStatic4.accelAAFBitshift = 7;
    }
    else if (BWIndex > 18 && BWIndex < 27)
    {
      accelConfigStatic4.accelAAFBitshift = 6;
    }
    else if (BWIndex > 26 && BWIndex < 37)
    {
      accelConfigStatic4.accelAAFBitshift = 5;
    }
    else if (BWIndex > 36 && BWIndex < 53)
    {
      accelConfigStatic4.accelAAFBitshift = 4;
    }
    else if (BWIndex > 53 && BWIndex <= 63)
    {
      accelConfigStatic4.accelAAFBitshift = 3;
    }
    writeReg(ICM42688_ACCEL_CONFIG_STATIC4, &accelConfigStatic4, 1);
    bank = 0;
    writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
  }
}
void ICM42688::setAAF(uint8_t who, bool mode)
{
  uint8_t bank = 0;
  if (who == GYRO)
  {
    if (mode)
    {
      gyroConfigStatic2.gyroAAFDis = 0;
    }
    else
    {
      gyroConfigStatic2.gyroAAFDis = 1;
    }
    bank = 1;
    writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
    writeReg(ICM42688_GYRO_CONFIG_STATIC2, &gyroConfigStatic2, 1);
  }
  else if (who == ACCEL)
  {
    if (mode)
    {
      accelConfigStatic2.accelAAFDis = 0;
    }
    else
    {
      accelConfigStatic2.accelAAFDis = 1;
    }
    bank = 2;
    writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
    writeReg(ICM42688_ACCEL_CONFIG_STATIC2, &accelConfigStatic2, 1);
  }
  else if (who == ALL)
  {
    if (mode)
    {
      gyroConfigStatic2.gyroAAFDis = 0;
      accelConfigStatic2.accelAAFDis = 0;
    }
    else
    {
      gyroConfigStatic2.gyroAAFDis = 1;
      accelConfigStatic2.accelAAFDis = 1;
    }
    bank = 1;
    writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
    writeReg(ICM42688_GYRO_CONFIG_STATIC2, &gyroConfigStatic2, 1);
    bank = 2;
    writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
    writeReg(ICM42688_ACCEL_CONFIG_STATIC2, &accelConfigStatic2, 1);
  }
  bank = 0;
  writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
}

bool ICM42688::setUIFilter(uint8_t who, uint8_t filterOrder, uint8_t UIFilterIndex)
{
  bool ret = true;
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
  if (filterOrder > 3 || UIFilterIndex > 15)
  {
    ret = false;
  }
  else
  {
    if (who == GYRO)
    {
      gyroConfig1.gyroUIFiltODR = filterOrder;
      writeReg(ICM42688_GYRO_CONFIG1, &gyroConfig1, 1);
      gyroAccelConfig0.gyroUIFiltBW = UIFilterIndex;
      writeReg(ICM42688_GYRO_ACCEL_CONFIG0, &gyroAccelConfig0, 1);
    }
    else if (who == ACCEL)
    {
      accelConfig1.accelUIFiltORD = filterOrder;
      writeReg(ICM42688_ACCEL_CONFIG1, &accelConfig1, 1);
      gyroAccelConfig0.accelUIFiltBW = UIFilterIndex;
      writeReg(ICM42688_GYRO_ACCEL_CONFIG0, &gyroAccelConfig0, 1);
    }
    else if (who == ALL)
    {
      gyroConfig1.gyroUIFiltODR = filterOrder;
      writeReg(ICM42688_GYRO_CONFIG1, &gyroConfig1, 1);
      accelConfig1.accelUIFiltORD = filterOrder;
      writeReg(ICM42688_ACCEL_CONFIG1, &accelConfig1, 1);
      gyroAccelConfig0.gyroUIFiltBW = UIFilterIndex;
      gyroAccelConfig0.accelUIFiltBW = UIFilterIndex;
      writeReg(ICM42688_GYRO_ACCEL_CONFIG0, &gyroAccelConfig0, 1);
    }
  }
  return ret;
}

void ICM42688::setSpeed(bool useHS) {
//	return;
	if(useHS == usingSPIHS) {
		return;
	}
	else {
		m_hSpi->Init.BaudRatePrescaler  = useHS ? SPI_BAUDRATEPRESCALER_8 : SPI_BAUDRATEPRESCALER_256;

		if (HAL_SPI_Init(m_hSpi) != HAL_OK)
		{
			Error_Handler();
		}
		usingSPIHS = useHS;
	}

}

void ICM42688::writeReg(uint8_t reg, void *pBuf, size_t size)
{
	_useSPIHS = false;
	setSpeed(_useSPIHS);

	HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(m_hSpi,&reg,1,1);
	HAL_SPI_Transmit(m_hSpi,(uint8_t*)pBuf,size,2);
	HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_SET);
}

uint8_t ICM42688::readReg(uint8_t reg, void *pBuf, size_t size)
{
	setSpeed(m_initialized);
  	uint8_t tx_data = reg | 0x80;

	HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(m_hSpi,&tx_data,1,1);
	HAL_SPI_Receive(m_hSpi,(uint8_t*)pBuf,size,2);
	HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_SET);
  return size;
}
