/*
 * util.c
 *
 *  Created on: 10 Sep 2018
 *      Author: ikenna
 */


#include "util.h"
#include<stdio.h>



void printConsole(char *ptr)
{
	while(*ptr != '\0')
	{
		ITM_SendChar((*ptr++));
	}
}


int _write(int file, char *ptr, int len)
{
  /* Implement your write code here, this is used by puts and printf for example */
	while(*ptr != '\0')
	{
		ITM_SendChar((*ptr++));
	}
	return 0;
}




int serial_readable(UART_HandleTypeDef* huart)
{
	/*  To avoid a target blocking case, let's check for
	 *  possible OVERRUN error and discard it
	 */
	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)) {
		__HAL_UART_CLEAR_OREFLAG(huart);
	}
	// Check if data is received
	return (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET) ? 1 : 0;
}


int serial_writeable(UART_HandleTypeDef* huart)
{
	return (__HAL_UART_GET_FLAG(huart, UART_FLAG_TXE) != RESET) ? 1 : 0;
}

int serial_getc(UART_HandleTypeDef* huart)
{
	while (!serial_readable(huart));
	return (int)(huart->Instance->RDR & 0x1FF);
}


void serial_putc(UART_HandleTypeDef* huart, char c)
{
	while (!serial_writeable(huart));
	huart->Instance->TDR = (uint32_t)(c & 0x1FF);
}


int serial_puts(UART_HandleTypeDef* huart,  char* s)
{
	int len = strlen(s);
	int	num = 0;
	int ret;

	while(*s != '\0')
	{
		serial_putc(huart, *s++);
		num++;
	}


	if(num == len)
		ret = 0;
	else
		ret = EOF;

	return ret;

}


uint8_t check_timeout(TimerCnt_t *timer, uint32_t timeout)
{
	return (HAL_GetTick() - timer->tick_start > timeout) ? 1 : 0;
}


void get_tick_start(TimerCnt_t *timer)
{
	timer->tick_start = HAL_GetTick();
}


void reset_timeout(TimerCnt_t *timer)
{
	get_tick_start(timer);
}


uint32_t get_millis()
{
	return HAL_GetTick();
}


void enter_low_power()
{
	HAL_RCC_EnableCSS();
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON,PWR_STOPENTRY_WFI);
}

void exit_low_power()
{
	SystemClock_Config();
}









