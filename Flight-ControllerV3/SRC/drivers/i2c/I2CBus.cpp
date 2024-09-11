/*
 * I2CBus.cpp
 *
 *  Created on: Nov 13, 2023
 *      Author: ienebuse
 */

#include <i2c/I2CBus.h>
#include <TimeTick.h>

I2C_Bus::I2C_Bus() {
	// TODO Auto-generated constructor stub
//	m_BusMutex = osMutexNew(NULL);
	m_busBusy = false;
}

I2C_Bus::~I2C_Bus() {
	// TODO Auto-generated destructor stub
}

void I2C_Bus::init(I2C_HandleTypeDef *hi2c) {
	m_I2c = hi2c;
}

HAL_StatusTypeDef I2C_Bus::is_device_ready(uint16_t DevAddress) {
	return HAL_I2C_IsDeviceReady(m_I2c, DevAddress << 1, 10,100);
}

bool I2C_Bus::isBusy() {
	return m_busBusy;
}


HAL_StatusTypeDef I2C_Bus::mem_read(uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t Size) {
	m_busBusy = true;
	HAL_StatusTypeDef resp = HAL_I2C_Mem_Read(m_I2c,DevAddress << 1, MemAddress, I2C_MEMADD_SIZE_8BIT , pData, Size, 10);
	m_busBusy = false;
	return resp;
}

HAL_StatusTypeDef I2C_Bus::mem_write(uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t Size) {
	m_busBusy = true;
	HAL_StatusTypeDef resp = HAL_I2C_Mem_Write(m_I2c,DevAddress << 1, MemAddress, I2C_MEMADD_SIZE_8BIT , pData, Size, 10);
	m_busBusy = false;
	return resp;
}

HAL_StatusTypeDef I2C_Bus::mem16_read(uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t Size) {
	m_busBusy = true;
	HAL_StatusTypeDef resp = HAL_I2C_Mem_Read(m_I2c,DevAddress << 1, MemAddress, I2C_MEMADD_SIZE_16BIT , pData, Size, 10);
	m_busBusy = false;
	return resp;
}

HAL_StatusTypeDef I2C_Bus::mem16_write(uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t Size) {
	m_busBusy = true;
	HAL_StatusTypeDef resp = HAL_I2C_Mem_Write(m_I2c,DevAddress << 1, MemAddress, I2C_MEMADD_SIZE_16BIT , pData, Size, 10);
	m_busBusy = false;
	return resp;
}

HAL_StatusTypeDef I2C_Bus::writeReg(uint16_t DevAddress, uint8_t* MemAddress, uint16_t Size) {
	m_busBusy = true;
	HAL_StatusTypeDef resp = HAL_I2C_Master_Transmit(m_I2c, DevAddress  << 1, MemAddress, Size, 10);
	m_busBusy = false;
	return resp;
}

HAL_StatusTypeDef I2C_Bus::readReg(uint16_t DevAddress, uint8_t* MemAddress, uint16_t Size) {
	m_busBusy = true;
	HAL_StatusTypeDef resp = HAL_I2C_Master_Receive(m_I2c, DevAddress  << 1, MemAddress, Size, 10);
	m_busBusy = false;
	return resp;
}

