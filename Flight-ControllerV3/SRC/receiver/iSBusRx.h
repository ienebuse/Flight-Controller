/*
 * iSBusRx.h
 *
 *  Created on: Jul 16, 2024
 *      Author: Ikenna
 */

#ifndef RECEIVER_ISBUSRX_H_
#define RECEIVER_ISBUSRX_H_
#include <stdint.h>

typedef struct SbusData {
  bool lost_frame;
  bool failsafe;
  bool ch17, ch18;
  static constexpr int8_t NUM_CH = 16;
  int16_t ch[NUM_CH];
}SbusData_t;


class iSBusRx {
public:
	iSBusRx();
	virtual ~iSBusRx();

	virtual void handleChannelData(SbusData) = 0;
};

#endif /* RECEIVER_ISBUSRX_H_ */
