/*
 * devicedef.h
 *
 *  Created on: Oct 31, 2023
 *      Author: ienebuse
 */

#ifndef DEVICEDEF_H_
#define DEVICEDEF_H_

#include <usart.h>
#include <spi.h>

typedef struct
{
//    UART_HandleTypeDef 	*debugUart; 		// Debug
    UART_HandleTypeDef 	*gpsUart;
    SPI_HandleTypeDef 	*imu1SPI;
    SPI_HandleTypeDef 	*imu2SPI;
    SPI_HandleTypeDef	*bbxSPI;
    SPI_HandleTypeDef	*osdSPI;
    TIM_HandleTypeDef 	*appTmr;
    TIM_HandleTypeDef	*motorTmr;
    I2C_HandleTypeDef 	*i2cBus;
    UART_HandleTypeDef 	*sBus;
    UART_HandleTypeDef 	*configUart;
    UART_HandleTypeDef 	*optflwUart;
    UART_HandleTypeDef 	*vtxUart;
} HAL_Devices_t;

extern HAL_Devices_t halDevices;

#endif /* DEVICEDEF_H_ */
