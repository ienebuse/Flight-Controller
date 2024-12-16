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

static constexpr float dt_T{0.02/3.6};

float Meter::batterLowThreshold = 10, Meter::batteryVoltage = 0, Meter::batteryCapacity = 0;

static constexpr uint8_t ADC_CURRENT_BUFF_SIZE = 10;
static constexpr uint8_t HALF_BUFFER = ADC_CURRENT_BUFF_SIZE/2;

uint32_t __attribute__((section("._ramd3_"), used)) m_AdcCurrentBuff[ADC_CURRENT_BUFF_SIZE];
volatile uint32_t Meter::m_ChannelValues[3];
volatile float Meter::VREF = 2500;
static constexpr float VREF{2.5};

static constexpr float ADC_TO_VOLTAGE {VREF/65536.0f};

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

	HAL_ADC_Start_DMA(&hadc3, m_AdcCurrentBuff, ADC_CURRENT_BUFF_SIZE);
	HAL_TIM_Base_Start(&htim6);
	HAL_ADC_Start_IT (&hadc1);
	HAL_TIM_Base_Start(&htim15);
}

float Meter::getBatteryVoltage() {
	return batteryVoltage;
}

float Meter::getBatteryCapacity() {
	return batteryCapacity;
}

float Meter::getCurrent() {
	float c = getAdcVoltage(ADC_CH_CURRENT) * Configurator::getConfig().settings.CurrentScale;
	return c;
}

float Meter::getAdcVoltage(ADC_CHANNEL channel) {
	return static_cast<float>(m_ChannelValues[channel] * ADC_TO_VOLTAGE);//ADC_TO_VOLTAGE;
}

uint16_t Meter::getAdc(ADC_CHANNEL channel) {
	return static_cast<float>(m_ChannelValues[channel]);//ADC_TO_VOLTAGE;
}

void Meter::update(ADC_HandleTypeDef* hadc, bool cmplt) {
	if(hadc->Instance == ADC3) {
		uint32_t sum = 0;
		uint8_t start, end;

		if(cmplt) {
			start = HALF_BUFFER;
			end = ADC_CURRENT_BUFF_SIZE;
		}
		else {
			start = 0;
			end = HALF_BUFFER;
		}

		for(uint8_t i = start; i < end; i++) {
			sum += m_AdcCurrentBuff[i];
		}
		m_ChannelValues[ADC_CH_CURRENT] += sum;
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

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	Meter::update(hadc, false);
}

void Meter::taskFunc(timetick_us currentTimeUs) {

	switch(Configurator::getConfig().settings.NumCell) {
	case 4:
		batterLowThreshold = 14.0;
		break;
	case 6:
		batterLowThreshold = 21.0;
		break;
	}
	batteryVoltage = getAdcVoltage(ADC_CH_VOLTAGE) * Configurator::getConfig().settings.VoltScale;
//	if((batteryVoltage < batterLowThreshold) && (currentTimeUs - m_lastTime > batterLowThreshold)) {
//		Application::buzzerToggle();
//		m_lastTime = currentTimeUs;
//	}
	if((batteryVoltage < batterLowThreshold)) {
		Buzzer::getInstance()->buzz();
	}

	batteryCapacity = getAdcVoltage(ADC_CH_CURRENT) * Configurator::getConfig().settings.CurrentScale * dt_T;
}

