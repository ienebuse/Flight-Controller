/*
 * ADC.cpp
 *
 *  Created on: Mar 18, 2024
 *      Author: ienebuse
 */

#include <meter/Meter.h>
#include <typedefs.h>
#include <Application.h>
#include <Configurator.h>

/* ADC internal channels related definitions */
/* Internal voltage reference VrefInt */
//#define VREFINT_CAL_ADDR     ((uint16_t*) (0x1FFF75AA)) /* Internal voltage reference, address of parameter VREFINT_CAL:
//                                                            VrefInt ADC raw data acquired at temperature 30 DegC
//                                                            (tolerance: +-5 DegC), Vref+ = 3.3 V (tolerance: +-10 mV).
//                                                         */
//#define VREFINT_CAL_VREF     ( 3000UL)                   /* Analog voltage reference (Vref+) value with which temperature sensor
//                                                            has been calibrated in production (tolerance: +-10 mV) (unit: mV).
//                                                         */
///* Temperature sensor */
//#define TEMPSENSOR_CAL1_ADDR ((uint16_t*) (0x1FFF75A8)) /* Internal temperature sensor, address of parameter TS_CAL1: On STM32F4,
//                                                            temperature sensor ADC raw data acquired at temperature  30 DegC
//                                                            (tolerance: +-5 DegC), Vref+ = 3.3 V (tolerance: +-10 mV).
//                                                         */
//#define TEMPSENSOR_CAL2_ADDR ((uint16_t*) (0x1FFF75CA)) /* Internal temperature sensor, address of parameter TS_CAL2: On STM32F4,
//                                                            temperature sensor ADC raw data acquired at temperature 110 DegC
//                                                            (tolerance: +-5 DegC), Vref+ = 3.3 V (tolerance: +-10 mV).
//                                                         */
//#define TEMPSENSOR_CAL1_TEMP (( int32_t)   30)           /* Internal temperature sensor, temperature at which temperature sensor
//                                                            has been calibrated in production for data into TEMPSENSOR_CAL1_ADDR
//                                                            (tolerance: +-5 DegC) (unit: DegC).
//                                                         */
//#define TEMPSENSOR_CAL2_TEMP (( int32_t)  130)           /* Internal temperature sensor, temperature at which temperature sensor
//                                                            has been calibrated in production for data into TEMPSENSOR_CAL2_ADDR
//                                                            (tolerance: +-5 DegC) (unit: DegC).
//                                                         */
//#define TEMPSENSOR_CAL_VREFANALOG ( 3000UL)              /* Analog voltage reference (Vref+) voltage with which temperature sensor
//                                                            has been calibrated in production (+-10 mV) (unit: mV). */

//uint32_t ADC::m_Adc1Buff[2], ADC::m_Adc2Buff[5];
uint32_t Meter::m_Adc3Buff[3];
volatile uint32_t Meter::m_ChannelValues[3];
//float constexpr ADC::ADC_TO_VOLTAGE;
volatile float Meter::VREF = 2500;

//float ADC::BatteryVoltage = 0;

Meter::Meter() {
	// TODO Auto-generated constructor stub

}

Meter::~Meter() {
	// TODO Auto-generated destructor stub
}

void Meter::init() {
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
	HAL_Delay(10);
	HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
	HAL_Delay(10);
	HAL_ADC_Start_IT (&hadc1);
	HAL_ADC_Start_IT (&hadc3);
	HAL_TIM_Base_Start(&htim6);
}

float Meter::getBatteryVoltage() {
	return batteryVoltage;
}

float Meter::getCurrent() {
	float c = getAdcVoltage(ADC_CH_CURRENT) * Configurator::getConfig().CurrentScale;
	return c;
}

float Meter::getAdcVoltage(ADC_CHANNEL channel) {
	return static_cast<float>(m_ChannelValues[channel] * getVref()/65536.0f);//ADC_TO_VOLTAGE;
}

uint16_t Meter::getAdc(ADC_CHANNEL channel) {
	return static_cast<float>(m_ChannelValues[channel]);//ADC_TO_VOLTAGE;
}

void Meter::update(ADC_HandleTypeDef* hadc) {
	if(hadc->Instance == ADC3) {
		m_ChannelValues[ADC_CH_CURRENT] = HAL_ADC_GetValue(hadc);
	}

	else if(hadc->Instance == ADC1) {
		m_ChannelValues[ADC_CH_VOLTAGE] = HAL_ADC_GetValue(hadc);
	}
}

float Meter::getVref() {
	return static_cast<float>(VREF)/1000;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	Meter::update(hadc);
}

void Meter::taskFunc(timetick_us currentTimeUs) {
	float batterLowThreshold = 10;
	switch(Configurator::getConfig().NumCell) {
	case 4:
		batterLowThreshold = 14.0;
		break;
	case 6:
		batterLowThreshold = 21.0;
		break;
	}
	batteryVoltage = getAdcVoltage(ADC_CH_VOLTAGE) * Configurator::getConfig().VoltScale;
	if((batteryVoltage < batterLowThreshold) && (currentTimeUs - m_lastTime > batterLowThreshold)) {
		Application::buzzerToggle();
		m_lastTime = currentTimeUs;
	}
}

