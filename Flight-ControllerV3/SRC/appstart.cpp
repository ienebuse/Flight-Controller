//-------------------------------------------------------------------------//
/**
 *  \file      appstart.cpp
 *  \author    Anton Rothwell
 *
 *  \brief     main entry point to application.  Called from RTOS MainTask
 *
 */

//-------------------------------------------------------------------------//
// Copyright (c) Raymarine UK Limited 2023
//
// Reproduction or transmission in whole or in part (whether by photocopying or
// storing in any medium by electronic means or otherwise) without the written
// permission of Raymarine UK Limited is prohibited.
//
// Confidential
//-------------------------------------------------------------------------//


#include <appstart.h>
#include "Application.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"
//#include "stm32h7xx.h"

static Application app;

/**
 * UART transmit interrupt
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3) {
		Application::sUartTxReady = true;
	}
//	if (huart==app.getIMUSender()->getUart())
//	{
//		app.getIMUSender()->setReady(true);
//	}
//
//	if(huart == app.getNxsSender()->getUart()) {
//		app.getNxsSender()->setReady(true);
//	}
//
//	if(huart == app.getDebugger()->getUart()) {
//		app.getDebugger()->setReady(true);
//	}
}

/**
 * UART receive interrupt
 */

void HAL_UART_ExRxEventCallback(UART_HandleTypeDef *huart,uint16_t size)
{

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static volatile timetick_us lastSbusData = 0,  lastOptflwData = 0;
	timetick_us now = TimeTick::getTimeUs();

	if(huart == app.getSbusRx()->getUartRx()->getUart()) {
		app.getSbusRx()->handleRxInterrupt(now - lastSbusData > 200);
		lastSbusData = now;
	}

	else if(huart == app.getM8NRx()->getUart()) {
		app.getM8NRx()->rxHandler();
	}

//	else if(huart == app.getOptFlwRx()->getUartRx()->getUart()) {
	else if(huart == app.getOptFlwRx()->getUart()) {
		app.getOptFlwRx()->handleRxInterrupt(now - lastOptflwData > 50000);
		lastOptflwData = now;
	}

//	else if(huart == iLogger::getUart()) {
	else if(huart->Instance == USART3) {
		iLogger::handleInterrupt();
	}

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if(huart == app.getSbusRx()->getUartRx()->getUart()) {
		app.getSbusRx()->getUartRx()->startReceiver();
	}

	else if(huart == app.getOptFlwRx()->getUart()) {
		app.getOptFlwRx()->startOptFlw();
	}


	else if(huart == app.getM8NRx()->getUart()) {
		app.getM8NRx()->rxHandler();
	}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
//  static uint8_t state = 0;
//
//  if(htim->Instance == TIM1) {
//	  HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_1);
//  }
//  else if (htim->Instance == TIM3) {
//	state = !state;
//	HAL_GPIO_WritePin(Lens_FREQ_Y_GPIO_Port, Lens_FREQ_Y_Pin, (GPIO_PinState)state);
//	HAL_GPIO_WritePin(Lens_FREQ_YN_GPIO_Port, Lens_FREQ_YN_Pin, (GPIO_PinState)!state);
//	HAL_TIM_OC_Stop_IT(&htim3,TIM_CHANNEL_1);
// }
//
//  /* NOTE : This function should not be modified, when the callback is needed,
//            the HAL_TIM_PWM_PulseFinishedCallback could be implemented in the user file
//   */
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//  if (htim->Instance == TIM5) {
//	  increamentAppTickuS();
//  }
//}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//  if (htim->Instance == TIM5) {
//	  increamentAppTickuS();
//  }
//}


extern "C" void initApplication(HAL_Devices_t *devices)
{
    app.init(devices);
}

extern "C" void runApplication()
{
    app.run();
}

extern "C" void* appInstance()
{
    return (void*)&app;
}

extern "C" void increamentAppTickuS() {
	app.increamentTickUs();
}

