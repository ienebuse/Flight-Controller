/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include <DPS310.h>
#include "TimeTick.h"
#include "cmath"
#include "typedefs.h"

using namespace Infineon_DPS310;

namespace dps310
{

template<typename T>
static void getTwosComplement(T &raw, uint8_t length)
{
	if (raw & ((T)1 << (length - 1))) {
		raw -= (T)1 << length;
	}
}

DPS310::DPS310()
{
}

DPS310::~DPS310()
{
}

int
DPS310::init(I2C_Bus* i2cBus)
{
	m_i2cBus = i2cBus;
	if (RegisterRead(Register::ID) != Infineon_DPS310::REV_AND_PROD_ID) {
		return -1;
	}

	if (reset() != 0) {
		return -2;
	}

	start();

	return 0;
}

int
DPS310::reset()
{
	// Soft Reset
	RegisterSetBits(Register::RESET, RESET_BIT::SOFT_RST);
	TimeTick::delay_us(40000);
//	usleep(40000);	// 40 milliseconds

	const uint8_t mode_and_status = RegisterRead(Register::MEAS_CFG);

	bool coefficients_ready = mode_and_status & MEAS_CFG_BIT::COEF_RDY;
	bool sensor_ready = mode_and_status & MEAS_CFG_BIT::SENSOR_RDY;

	if (!coefficients_ready) {
//		PX4_ERR("Coefficients are not available");
		return -1;
	}

	if (!sensor_ready) {
//		PX4_ERR("Sensor initialization not complete");
		return -1;
	}

	// 1. Read the pressure calibration coefficients (c00, c10, c20, c30, c01, c11, and c21) from the Calibration Coefficient register.
	//   Note: The coefficients read from the coefficient register are 2's complement numbers.
	uint8_t coef[18] {};

	if (read((uint8_t)Register::COEF, coef, 18)) {
		return -1;
	}

	// first element of coef[18] corresponds to register 0x10

	// 0x11 c0 [3:0] + 0x10 c0 [11:4]
	_calibration.c0 = ((uint32_t)coef[0] << 4) | (((uint32_t)coef[1] >> 4) & 0x0F);
	getTwosComplement(_calibration.c0, 12);

	// 0x11 c1 [11:8] + 0x12 c1 [7:0]
	_calibration.c1 = (((uint32_t)coef[1] & 0x0F) << 8) | (uint32_t)coef[2];
	getTwosComplement(_calibration.c1, 12);

	// 0x13 c00 [19:12] + 0x14 c00 [11:4] + 0x15 c00 [3:0]
	_calibration.c00 = ((uint32_t)coef[3] << 12) | ((uint32_t)coef[4] << 4) | (((uint32_t)coef[5] >> 4) & 0x0F);
	getTwosComplement(_calibration.c00, 20);

	// 0x15 c10 [19:16] + 0x16 c10 [15:8] + 0x17 c10 [7:0]
	_calibration.c10 = (((uint32_t)coef[5] & 0x0F) << 16) | ((uint32_t)coef[6] << 8) | (uint32_t)coef[7];
	getTwosComplement(_calibration.c10, 20);

	// 0x18 c01 [15:8] + 0x19 c01 [7:0]
	_calibration.c01 = ((uint32_t)coef[8] << 8) | (uint32_t)coef[9];
	getTwosComplement(_calibration.c01, 16);

	// 0x1A c11 [15:8] + 0x1B c11 [7:0]
	_calibration.c11 = ((uint32_t)coef[10] << 8) | (uint32_t)coef[11];
	getTwosComplement(_calibration.c11, 16);

	// 0x1C c20 [15:8] + 0x1D c20 [7:0]
	_calibration.c20 = ((uint32_t)coef[12] << 8) | (uint32_t)coef[13];
	getTwosComplement(_calibration.c20, 16);

	// 0x1E c21 [15:8] + 0x1F c21 [7:0]
	_calibration.c21 = ((uint32_t)coef[14] << 8) | (uint32_t)coef[15];
	getTwosComplement(_calibration.c21, 16);

	// 0x20 c30 [15:8] + 0x21 c30 [7:0]
	_calibration.c30 = ((uint32_t)coef[16] << 8) | (uint32_t)coef[17];
	getTwosComplement(_calibration.c30, 16);


	// PRS_CFG: pressure measurement rate (32 Hz) and oversampling (16 time standard)
	RegisterSetBits(Register::PRS_CFG, PRS_CFG_BIT::PM_RATE_32HZ | PRS_CFG_BIT::PM_PRC_16);

	// TMP_CFG: temperature measurement rate (32 Hz) and oversampling (16 times)
	const uint8_t TMP_COEF_SRCE = RegisterRead(Register::COEF_SRCE) & COEF_SRCE_BIT::TMP_COEF_SRCE;
	RegisterSetBits(Register::TMP_CFG, TMP_CFG_BIT::TMP_RATE_32HZ | TMP_CFG_BIT::TMP_PRC_16 | TMP_COEF_SRCE);

	// CFG_REG: set pressure and temperature result bit-shift (required when the oversampling rate is >8 times)
	RegisterSetBits(Register::CFG_REG, CFG_REG_BIT::T_SHIFT | CFG_REG_BIT::P_SHIFT);

	// MEAS_CFG: Continous pressure and temperature measurement
	RegisterSetBits(Register::MEAS_CFG, MEAS_CFG_BIT::MEAS_CTRL_CONT);

	return 0;
}

void
DPS310::start()
{
	// run at twice the sample rate to capture all new data
//	ScheduleOnInterval(1000000 / SAMPLE_RATE / 2);
}

AltData
DPS310::getAltitude()
{
	m_altData.altAvailable = false;
//	perf_begin(_sample_perf);

	// check if pressure ready
	bool pressure_ready = RegisterRead(Register::MEAS_CFG) & MEAS_CFG_BIT::PRS_RDY;

	if (!pressure_ready) {
		return m_altData;
	}


	// 2. Choose scaling factors kT (for temperature) and kP (for pressure) based on the chosen precision rate. The scaling factors are listed in Table 9.
	static constexpr float kT = 253952;	// 16 times (Standard)
	static constexpr float kP = 253952;	// 16 times (Standard)


	// 3. Read the pressure and temperature result from the registers

	// Read PSR_B2, PSR_B1, PSR_B0, TMP_B2, TMP_B1, TMP_B0
	uint8_t buf[6] {};
//	const hrt_abstime timestamp_sample = hrt_absolute_time();

	if (read((uint8_t)Register::PSR_B2, buf, 6) != 0) {
		return m_altData;
	}

	int32_t Praw = (buf[0] << 16) + (buf[1] << 8) + buf[2];
	getTwosComplement(Praw, 24);

	int32_t Traw = (buf[3] << 16) + (buf[4] << 8) + buf[5];
	getTwosComplement(Traw, 24);


	// 4. Calculate scaled measurement results.
	const float Praw_sc = Praw / kP;
	const float Traw_sc = Traw / kT;


	// 5. Calculate compensated measurement results.
	const auto &c00 = _calibration.c00;
	const auto &c01 = _calibration.c01;
	const auto &c10 = _calibration.c10;
	const auto &c11 = _calibration.c11;
	const auto &c20 = _calibration.c20;
	const auto &c21 = _calibration.c21;
	const auto &c30 = _calibration.c30;

	const float Pcomp = c00 + Praw_sc * (c10 + Praw_sc * (c20 + Praw_sc * c30)) + Traw_sc * c01 + Traw_sc * Praw_sc *
			    (c11 + Praw_sc * c21);

	const auto &c0 = _calibration.c0;
	const auto &c1 = _calibration.c1;

	const float Tcomp = c0 * 0.5f + c1 * Traw_sc;

	m_altData.altAvailable = true;
	m_altData.altitude = 44330 * (1.0 - pow(Pcomp / SEA_LEVEL_hPA, 0.1903));
	m_altData.temperature = Tcomp;

	return m_altData;
}

uint8_t
DPS310::RegisterRead(Register reg)
{
	uint8_t buf{};
	read((uint8_t)reg, &buf, 1);

	return buf;
}

void
DPS310::RegisterWrite(Register reg, uint8_t value)
{
	write((uint8_t)reg, &value, 1);
}

void
DPS310::RegisterSetBits(Register reg, uint8_t setbits)
{
	uint8_t val = RegisterRead(reg);

	if (!(val & setbits)) {
		val |= setbits;
		RegisterWrite(reg, val);
	}
}

void
DPS310::RegisterClearBits(Register reg, uint8_t clearbits)
{
	uint8_t val = RegisterRead(reg);

	if (val & clearbits) {
		val &= !clearbits;
		RegisterWrite(reg, val);
	}
}

} // namespace dps310
