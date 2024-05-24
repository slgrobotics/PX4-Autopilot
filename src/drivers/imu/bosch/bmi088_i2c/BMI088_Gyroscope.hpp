/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

#pragma once

#include "BMI088.hpp"

#include "Ema3.hpp"

#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>

#include "Bosch_BMI088_Gyroscope_Registers.hpp"

namespace Bosch::BMI088::Gyroscope
{

class BMI088_Gyroscope : public BMI088
{
public:
	BMI088_Gyroscope(const I2CSPIDriverConfig &config);
	~BMI088_Gyroscope() override;

	void RunImpl() override;
	void print_status() override;

private:
	void exit_and_cleanup() override;

	struct register_config_t {
		Register reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	int probe() override;

	bool Configure();
	void ConfigureGyro();

	bool RegisterCheck(const register_config_t &reg_cfg);

	uint8_t RegisterRead(Register reg);
	void RegisterWrite(Register reg, uint8_t value);
	void RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits);

	bool SelfTest();
	bool NormalRead(const hrt_abstime &timestamp_sample);

	PX4Gyroscope _px4_gyro;

	Ema3f _ema;
	const int GYRO_EMA_PERIOD = 20;

	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME"_gyro: bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME"_gyro: bad transfer")};
	perf_counter_t _drdy_missed_perf{nullptr};

	uint8_t _checked_register{0};
	static constexpr uint8_t size_register_cfg{8};
	register_config_t _register_cfg[size_register_cfg] {
		// Register                         | Set bits, Clear bits
		//{ Register::GYRO_RANGE,             GYRO_RANGE_BIT::gyro_range_250_dps, 0 },
		{ Register::GYRO_RANGE,             GYRO_RANGE_BIT::gyro_range_2000_dps, 0 },
		{ Register::GYRO_BANDWIDTH,         GYRO_BANDWIDTH_BIT::gyro_bw_2000_Hz_532_hz, 0}, // page 28
		//{ Register::GYRO_BANDWIDTH,         GYRO_BANDWIDTH_BIT::gyro_bw_1000_Hz_116_Hz, 0},
		{ Register::GYRO_INT_CTRL,          0, 0 },
		{ Register::INT3_INT4_IO_CONF,      0, 0 },
		{ Register::INT3_INT4_IO_MAP,       0, 0 },
		{ Register::FIFO_WM_ENABLE,         0, 0 },
		{ Register::FIFO_CONFIG_0,          0, 0 }, // fifo_water_mark_level_trigger_retain<6:0>
		{ Register::FIFO_CONFIG_1,          0, 0 },
	};
};

} // namespace Bosch::BMI088::Gyroscope
