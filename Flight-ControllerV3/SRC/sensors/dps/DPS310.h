/****************************************************************************
 *
 *   Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file DPS310.hpp
 *
 * Driver for the Infineon DPS310 barometer connected via I2C or SPI.
 */

#pragma once

#include <I2CBus.h>
#include <sensors/dps/Infineon_DPS310_Registers.hpp>
#include <TimeTick.h>

#define DPS310_ADDRESS		0x76

typedef struct {
	timetick_us acqTimeUs = 0;
	uint8_t acqState = 0;
	float altitude = 0;
	float temperature = 0;
	bool altAvailable = false;
}AltData;

namespace dps310
{

using Infineon_DPS310::CalibrationCoefficients;
using Infineon_DPS310::Register;

class DPS310
{
public:
	DPS310();
	virtual ~DPS310();

	int init(I2C_Bus* i2cBus) ;
	AltData			getAltitude();

private:

	I2C_Bus* m_i2cBus;
	uint8_t m_devAddr{DPS310_ADDRESS};
	AltData m_altData;

	void			start();
	int			reset();

	uint8_t			RegisterRead(Register reg);
	void			RegisterWrite(Register reg, uint8_t val);
	void			RegisterSetBits(Register reg, uint8_t setbits);
	void			RegisterClearBits(Register reg, uint8_t clearbits);

	int read(uint8_t reg, uint8_t* data, uint16_t len) {
		return m_i2cBus->mem_read(m_devAddr, reg, data, len);
	}

	int write(uint8_t reg, uint8_t* data, uint16_t len) {
		return m_i2cBus->mem_write(m_devAddr, reg, data, len);
	}

	static constexpr uint32_t SAMPLE_RATE{32};

	CalibrationCoefficients	_calibration{};
};

} // namespace dps310
