//-------------------------------------------------------------------------//
/**
 *  \file      appstart.h
 *  \author    Anton Rothwell
 *
 *  \brief     main entry point to application.  Called from RTOS MainTask
 *
 */

//-------------------------------------------------------------------------//
// Copyright (c) Raymarine UK Limited 2022
//
// Reproduction or transmission in whole or in part (whether by photocopying or
// storing in any medium by electronic means or otherwise) without the written
// permission of Raymarine UK Limited is prohibited.
//
// Confidential
//-------------------------------------------------------------------------//


#ifndef APPSTART_H_
#define APPSTART_H_

#include "main.h"
#include "devicedef.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes the application.
 *
 * This function initializes the application by setting up the devices.
 *
 * @param devices Pointer to the HAL_Devices_t structure containing device information.
 */
void initApplication(HAL_Devices_t *devices);

/**
 * @brief Runs the application.
 *
 * This function runs the main application logic.
 */
void runApplication();

/**
 * @brief Gets a pointer to the application instance.
 *
 * This function returns a pointer to the application instance.
 *
 * @return Pointer to the application instance.
 */
void *appInstance();

void increamentAppTickuS();

#ifdef __cplusplus
}
#endif

#endif /* APPSTART_H_ */
