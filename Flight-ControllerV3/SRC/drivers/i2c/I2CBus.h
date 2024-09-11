/*
 * I2CBus.h
 *
 *  Created on: Nov 13, 2023
 *      Author: ienebuse
 */

#ifndef I2C_I2CBUS_H_
#define I2C_I2CBUS_H_

#include "stm32h7xx.h"
//#include "cmsis_os2.h"
//#include <drivers/i2c/I2CBus.h>


typedef uint8_t devAddr_t;



/**
 * @brief Class representing an I2C bus interface.
 */
class I2C_Bus {
public:
    /**
     * @brief Constructor.
     */
    I2C_Bus();

    /**
     * @brief Destructor.
     */
    virtual ~I2C_Bus();

    /**
     * @brief Initializes the I2C bus.
     * @param hi2c Pointer to a I2C_HandleTypeDef structure that contains
     *        the configuration information for the specified I2C.
     */
    void init(I2C_HandleTypeDef *hi2c);

    /**
     * @brief Checks if target device is ready for communication.
     * @param DevAddress Target device address.
     * @retval HAL status.
     */
    HAL_StatusTypeDef is_device_ready(uint16_t DevAddress);

    /**
     * @brief Reads from a specified 8 bit memory address of an I2C device on the I2C bus.
     * @param DevAddress Target device address.
     * @param MemAddress Internal memory address in the device to be read.
     * @param pData Pointer to data buffer to store the read data.
     * @param Size Amount of data to be read.
     * @retval HAL status.
     */
    bool isBusy();

    HAL_StatusTypeDef mem_read(uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t Size);

    /**
     * @brief Writes to a specified 8 bit memory address of an I2C device on the I2C bus.
     * @param DevAddress Target device address.
     * @param MemAddress Internal memory address in the device to be written to.
     * @param pData Pointer to data buffer containing the data to be written.
     * @param Size Amount of data to be written.
     * @retval HAL status.
     */
    HAL_StatusTypeDef mem_write(uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t Size);

    /**
     * @brief Reads from a specified 16 bit memory address of an I2C device on the I2C bus.
     * @param DevAddress Target device address.
     * @param MemAddress Internal memory address in the device to be read.
     * @param pData Pointer to data buffer to store the read data.
     * @param Size Amount of data to be read.
     * @retval HAL status.
     */
    HAL_StatusTypeDef mem16_read(uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t Size);

    /**
     * @brief Writes to a specified 16 bit memory address of an I2C device on the I2C bus.
     * @param DevAddress Target device address.
     * @param MemAddress Internal memory address in the device to be written to.
     * @param pData Pointer to data buffer containing the data to be written.
     * @param Size Amount of data to be written.
     * @retval HAL status.
     */
    HAL_StatusTypeDef mem16_write(uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t Size);

    /**
     * @brief Transmit an internal memory address of an I2C device on the I2C bus.
     * @param DevAddress Target device address.
     * @param MemAddress Internal memory address in the device to be written.
     * @retval HAL status.
     */
    HAL_StatusTypeDef writeReg(uint16_t DevAddress, uint8_t* MemAddress, uint16_t Size = 1);

    /**
	 * @brief Transmit an internal memory address of an I2C device on the I2C bus.
	 * @param DevAddress Target device address.
	 * @param MemAddress Internal memory address in the device to be written.
	 * @retval HAL status.
	 */
	HAL_StatusTypeDef readReg(uint16_t DevAddress, uint8_t* MemAddress, uint16_t Size = 1);

private:
    I2C_HandleTypeDef *m_I2c; 	/**< Pointer to the I2C handle. */
//    osMutexId_t m_BusMutex; 	/**< Mutex used for bus access synchronization. */
    bool m_busBusy{false};
};

#endif /* I2C_I2CBUS_H_ */

