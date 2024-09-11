/*
 * util.h
 *
 *  Created on: 10 Sep 2018
 *      Author: ikenna
 */

#ifndef UTIL_H_
#define UTIL_H_

#include "stm32h7xx_hal.h"
#include <string.h>
#include <time.h>

#define true	1
#define false	0


#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })




enum
{
	ON = 1,
	OFF,
};


typedef struct{
	uint32_t tick_start;
}TimerCnt_t;



extern UART_HandleTypeDef huart2;


//extern void SystemClock_Config(void)


extern void printConsole(char*);


int serial_readable(UART_HandleTypeDef* huart);


int serial_writeable(UART_HandleTypeDef* huart);


int serial_getc(UART_HandleTypeDef* huart);


void serial_putc(UART_HandleTypeDef* huart, char c);


int serial_puts(UART_HandleTypeDef* huart,  char* s);


uint8_t check_timeout(TimerCnt_t *timer, uint32_t timeout);


void get_tick_start(TimerCnt_t *timer);


void reset_timeout(TimerCnt_t *timer);


uint32_t get_millis();


void enter_low_power();


void exit_low_power();


#endif /* UTIL_H_ */
