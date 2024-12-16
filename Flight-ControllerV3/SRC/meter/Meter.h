/*
 * ADC.h
 *
 *  Created on: Mar 18, 2024
 *      Author: ienebuse
 */

#ifndef METER_METER_H_
#define METER_METER_H_

#include <adc.h>
#include <tim.h>
#include <Task.h>

#define TMPSENSOR_V30		0.76
#define TMPSENSOR_AVGSLOPE	2.5
#define V30					30

/**
  * @brief  ADC Class for handling analog-to-digital conversion
  */
class Meter : public Task {
public:
    /**
      * @brief  Constructor for ADC class
      */
    Meter();

    /**
      * @brief  Destructor for ADC class
      */
    virtual ~Meter();

    /**
      * @brief  Enum defining different ADC channels
      */
    enum ADC_CHANNEL {
        ADC_CH_VREF,
        ADC_CH_CURRENT,
        ADC_CH_VOLTAGE,
    };

    /**
      * @brief  Initialize the ADC module
      */
    void init();

    /**
      * @brief  Get the converted ADC value from a specified channel
      * @param  channel The ADC channel whose value is to be returned
      * @retval Returns the analog value (voltage) of the channel
      */
    float getAdcVoltage(ADC_CHANNEL channel);

    uint16_t getAdc(ADC_CHANNEL channel);

    float getBatteryVoltage();

    float getBatteryCapacity();

    float getCurrent();

    /**
      * @brief  Update the ADC channel values with the converted values in the buffer
      * @param  hadc The ADC handle to be updated
      */
    static void update(ADC_HandleTypeDef* hadc, bool cmplt = true);

    static float getVref();

    static bool batteryCritical() {
    	return batteryVoltage < batterLowThreshold;
    }

    virtual void taskFunc(timetick_us currenTimeUs);



private:
//    static constexpr uint8_t ADC_CURRENT_BUFF_SIZE{10};
//    static constexpr uint8_t HALF_BUFFER{ADC_CURRENT_BUFF_SIZE / 2};
//    static uint32_t m_AdcCurrentBuff[ADC_CURRENT_BUFF_SIZE]; 			/**< Buffers for ADC conversion results */
    static volatile uint32_t m_ChannelValues[3]; 			/**< Array to hold ADC channel values */
    static volatile float VREF;
	static float batteryVoltage, batteryCapacity;
	static float batterLowThreshold;
	timetick_us m_lastTime;
};

#endif /* METER_METER_H_ */
