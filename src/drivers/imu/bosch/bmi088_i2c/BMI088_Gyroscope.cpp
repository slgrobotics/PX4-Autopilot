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

#include "BMI088_Gyroscope.hpp"

//#include <px4_platform/board_dma_alloc.h>

using namespace time_literals;

namespace Bosch::BMI088::Gyroscope
{

BMI088_Gyroscope::BMI088_Gyroscope(const I2CSPIDriverConfig &config) :
	BMI088(config),
	_px4_gyro(get_device_id(), config.rotation)
{
	PX4_INFO_RAW("BMI088_Gyroscope::BMI088_Gyroscope()\n");

	if (config.drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME"_gyro: DRDY missed");
	}

	_ema.init(20);
}

BMI088_Gyroscope::~BMI088_Gyroscope()
{
	PX4_INFO_RAW("BMI088_Gyroscope::~BMI088_Gyroscope()\n");

	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_drdy_missed_perf);
}

void BMI088_Gyroscope::exit_and_cleanup()
{
	PX4_INFO_RAW("BMI088_Gyroscope::exit_and_cleanup()\n");

	I2CSPIDriverBase::exit_and_cleanup();
}

void BMI088_Gyroscope::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("Sampling interval: %d us (%.1f Hz)", _sampling_interval_us, 1e6 / _sampling_interval_us);

	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_drdy_missed_perf);
}

int BMI088_Gyroscope::probe()
{
	PX4_INFO_RAW("BMI088_Gyroscope::probe()\n");

	const uint8_t chipid = RegisterRead(Register::GYRO_CHIP_ID);

	if (chipid == ID) {
		PX4_INFO("BMI088 Gyro found");

	} else {
		DEVICE_DEBUG("unexpected GYRO_CHIP_ID 0x%02x", chipid);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void BMI088_Gyroscope::RunImpl()
{
	//PX4_INFO_RAW("BMI088_Gyroscope::RunImpl()\n");

	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {

	case STATE::SELFTEST:
		//SelfTest();

		PX4_INFO_RAW("BMI088_Gyroscope::RunImpl() STATE::SELFTEST\n");

		_state = STATE::RESET;
		ScheduleDelayed(1_ms);
		break;

	case STATE::RESET:
		PX4_INFO_RAW("BMI088_Gyroscope::RunImpl() STATE::RESET\n");

		// GYRO_SOFTRESET: Writing a value of 0xB6 to this register resets the sensor.
		// Following a delay of 30 ms, all configuration settings are overwritten with their reset value.
		RegisterWrite(Register::GYRO_SOFTRESET, 0xB6);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(30_ms);
		break;

	case STATE::WAIT_FOR_RESET:
		PX4_INFO_RAW("BMI088_Gyroscope::RunImpl() STATE::WAIT_FOR_RESET\n");

		if ((RegisterRead(Register::GYRO_CHIP_ID) == ID)) {
			// if reset succeeded then configure
			_state = STATE::CONFIGURE;
			ScheduleDelayed(1_ms);

		} else {
			// RESET not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Reset failed, retrying");
				_state = STATE::RESET;
				ScheduleDelayed(100_ms);

			} else {
				PX4_DEBUG("Reset not complete, check again in 10 ms");
				ScheduleDelayed(10_ms);
			}
		}

		break;

	case STATE::CONFIGURE:
		PX4_INFO_RAW("BMI088_Gyroscope::RunImpl() STATE::CONFIGURE\n");

		if (Configure()) {
			// if configure succeeded then start reading data
			_state = STATE::DATA_READ;

			//ScheduleOnInterval(_sampling_interval_us, _sampling_interval_us);
			ScheduleDelayed(10_ms);

		} else {
			// CONFIGURE not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Configure failed, resetting");
				_state = STATE::RESET;

			} else {
				PX4_DEBUG("Configure failed, retrying");
			}

			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::DATA_READ: {
			NormalRead(now);
			//ScheduleDelayed(1_ms);
			ScheduleNow();	// 250 Hz
		}
		break;
	}
}

void BMI088_Gyroscope::ConfigureGyro()
{
	const uint8_t GYRO_RANGE = RegisterRead(Register::GYRO_RANGE) & (Bit3 | Bit2 | Bit1 | Bit0);

	PX4_INFO_RAW("BMI088_Gyroscope::ConfigureGyro()  GYRO_RANGE: %d\n", (int)GYRO_RANGE);

	switch (GYRO_RANGE) {
	case gyro_range_2000_dps:
		_px4_gyro.set_scale(math::radians(1.f / 16.384f));
		_px4_gyro.set_range(math::radians(2000.f));
		break;

	case gyro_range_1000_dps:
		_px4_gyro.set_scale(math::radians(1.f / 32.768f));
		_px4_gyro.set_range(math::radians(1000.f));
		break;

	case gyro_range_500_dps:
		_px4_gyro.set_scale(math::radians(1.f / 65.536f));
		_px4_gyro.set_range(math::radians(500.f));
		break;

	case gyro_range_250_dps:
		_px4_gyro.set_scale(math::radians(1.f / 131.072f));
		_px4_gyro.set_range(math::radians(250.f));
		break;

	case gyro_range_125_dps:
		_px4_gyro.set_scale(math::radians(1.f / 262.144f));
		_px4_gyro.set_range(math::radians(125.f));
		break;
	}
}

bool BMI088_Gyroscope::Configure()
{
	PX4_INFO_RAW("BMI088_Gyroscope::Configure()\n");

	// first set and clear all configured register bits
	for (const auto &reg_cfg : _register_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	// now check that all are configured
	bool success = true;

	for (const auto &reg_cfg : _register_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	ConfigureGyro();

	return success;
}

bool BMI088_Gyroscope::RegisterCheck(const register_config_t &reg_cfg)
{
	PX4_INFO_RAW("BMI088_Gyroscope::RegisterCheck()\n");

	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	return success;
}

uint8_t BMI088_Gyroscope::RegisterRead(Register reg)
{
	//PX4_INFO_RAW("Gyro: RegisterRead()\n");

	uint8_t add = static_cast<uint8_t>(reg);
	uint8_t cmd[2] = {add, 0};
	transfer(&cmd[0], 1, &cmd[1], 1);
	return cmd[1];
}

void BMI088_Gyroscope::RegisterWrite(Register reg, uint8_t value)
{
	//PX4_INFO_RAW("Gyro: RegisterWrite()\n");

	uint8_t add = static_cast<uint8_t>(reg);
	uint8_t cmd[2] = {add, value};
	transfer(cmd, sizeof(cmd), nullptr, 0);
}

void BMI088_Gyroscope::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	PX4_INFO_RAW("BMI088_Gyroscope::RegisterSetAndClearBits()\n");

	const uint8_t orig_val = RegisterRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

bool BMI088_Gyroscope::SelfTest()
{
	PX4_INFO_RAW("BMI088_Gyroscope::SelfTest()\n");

	//Datasheet page 17 self test

	//Set bit0 to enable built in self test
	RegisterWrite(Register::SELF_TEST, 0x01);
	usleep(10000);
	uint8_t res = 0;
	uint8_t test_res = false;

	while (true) {
		res = RegisterRead(Register::SELF_TEST);

		if ((res & 0x02) == 0x02) {
			if ((res & 0x04) == 0x00) {
				PX4_WARN("Gyro Self-test success");
				test_res = true;

			} else {
				PX4_WARN("Gyro Self-test error");
			}

			break;
		}
	}

	RegisterWrite(Register::SELF_TEST, 0x00);
	return test_res;
}

bool BMI088_Gyroscope::NormalRead(const hrt_abstime &timestamp_sample)
{
	//PX4_INFO_RAW("Gyro:NormalRead())\n");

	float x = 0;
	float y = 0;
	float z = 0;
	uint8_t buffer[6] = {0};
	uint8_t cmd[1] = {static_cast<uint8_t>(Register::READ_GYRO)};

	//transfer(&cmd[0], 1, &buffer[0], 6);

	if (transfer(&cmd[0], 1, &buffer[0], 6) != PX4_OK) {
		PX4_WARN("transfer(&data[0], 1, &data[0], n_frames) != PX4_OK");
		perf_count(_bad_transfer_perf);
		return false;
	}


	uint8_t RATE_X_LSB = buffer[0];
	uint8_t RATE_X_MSB = buffer[1];
	uint8_t RATE_Y_LSB = buffer[2];
	uint8_t RATE_Y_MSB = buffer[3];
	uint8_t RATE_Z_LSB = buffer[4];
	uint8_t RATE_Z_MSB = buffer[5];

	const int16_t gyro_x = combine(RATE_X_MSB, RATE_X_LSB);
	const int16_t gyro_y = combine(RATE_Y_MSB, RATE_Y_LSB);
	const int16_t gyro_z = combine(RATE_Z_MSB, RATE_Z_LSB);

	if (gyro_x == INT16_MIN) {
		PX4_WARN("gyro_x == INT16_MIN");
	}

	if (gyro_y == INT16_MIN) {
		PX4_WARN("gyro_y == INT16_MIN");
	}

	if (gyro_z == INT16_MIN) {
		PX4_WARN("gyro_z == INT16_MIN");
	}

	if (gyro_x == INT16_MIN || gyro_y == INT16_MIN || gyro_z == INT16_MIN) {
		PX4_WARN("Gyro:NormalRead(): INT16_MIN frame rejected");
		perf_count(_bad_transfer_perf);
		return false;
	}

	matrix::Vector3f val {(float)gyro_x, (float)gyro_y, (float)gyro_z};

	matrix::Vector3f res = _ema.Compute(val);

	// sensor's frame is +x forward, +y left, +z up
	//  flip y & z to publish right handed with z down (x forward, y right, z down)
	//x = gyro_x;
	//y = -gyro_y;
	//z = -gyro_z;

	x = res(0);
	y = -res(1);
	z = -res(2);

	//PX4_WARN("x: %f | y: %f | z: %f", (double)x, (double)y ,(double)z);
	_px4_gyro.update(timestamp_sample, x, y, z);

	return true;
}

} // namespace Bosch::BMI088::Gyroscope
