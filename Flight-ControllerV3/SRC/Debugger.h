/*
 * Debugger.h
 *
 *  Created on: Dec 30, 2024
 *      Author: Ikenna
 */

#ifndef DEBUGGER_H_
#define DEBUGGER_H_

#include "usbd_cdc_if.h"

class Debugger {
public:
	Debugger();
	virtual ~Debugger();

	static inline void sendDbgLog(char* log) {
		CDC_Transmit_FS((uint8_t*)log, strlen(log));
	}
};

#endif /* DEBUGGER_H_ */
