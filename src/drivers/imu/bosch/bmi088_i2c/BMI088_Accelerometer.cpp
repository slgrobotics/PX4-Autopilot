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

#include "BMI088_Accelerometer.hpp"

#include <geo/geo.h> // CONSTANTS_ONE_G

using namespace time_literals;

namespace Bosch::BMI088::Accelerometer
{
BMI088_Accelerometer::BMI088_Accelerometer(const I2CSPIDriverConfig &config) :
	BMI088(config),
	_px4_accel(get_device_id(), config.rotation)
{
	if (config.drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME"_accel: DRDY missed");
	}

	//ConfigureSampleRate(1600);
}

BMI088_Accelerometer::~BMI088_Accelerometer()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_drdy_missed_perf);
}

void BMI088_Accelerometer::exit_and_cleanup()
{
	I2CSPIDriverBase::exit_and_cleanup();
}

void BMI088_Accelerometer::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("Sampling interval: %d us (%.1f Hz)", _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us);

	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_drdy_missed_perf);
}

uint8_t BMI088_Accelerometer::RegisterRead(Register reg)
{
	uint8_t add = static_cast<uint8_t>(reg);
	uint8_t cmd[2] = {add, 0};
	transfer(&cmd[0], 1, &cmd[1], 1);
	return cmd[1];
}

uint8_t BMI088_Accelerometer::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t add = static_cast<uint8_t>(reg);
	uint8_t cmd[2] = { add, value};
	return transfer(cmd, sizeof(cmd), nullptr, 0);
}

int BMI088_Accelerometer::probe()
{
	const uint8_t ACC_CHIP_ID = RegisterRead(Register::ACC_CHIP_ID);

	if (ACC_CHIP_ID == ID_088) {
		PX4_INFO("BMI088 Accel found");

	} else if (ACC_CHIP_ID == ID_090L) {
		PX4_INFO("BMI090L Accel found");

	} else {
		PX4_ERR("unexpected ACC_CHIP_ID 0x%02x", ACC_CHIP_ID);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void BMI088_Accelerometer::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::SELFTEST:
		//PX4_WARN("Selftest state");
		//SelfTest();
		_state = STATE::RESET;
		ScheduleDelayed(10_ms);
		break;

	case STATE::RESET:
		// ACC_SOFTRESET: Writing a value of 0xB6 to this register resets the sensor
		RegisterWrite(Register::ACC_SOFTRESET, 0xB6);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;


		ScheduleDelayed(1_ms); // Following a delay of 1 ms, all configuration settings are overwritten with their reset value.

		break;

	case STATE::WAIT_FOR_RESET:
		if ((RegisterRead(Register::ACC_CHIP_ID) == ID_088) || (RegisterRead(Register::ACC_CHIP_ID) == ID_090L)) {
			// ACC_PWR_CONF: Power on sensor
			RegisterWrite(Register::ACC_PWR_CONF, 0);

			// if reset succeeded then configure
			_state = STATE::CONFIGURE;
			ScheduleDelayed(10_ms);

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
		if (Configure()) {
			// if configure succeeded then start reading data
			_state = STATE::DATA_READ;

			ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);

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
		}
		break;
	}
}

void BMI088_Accelerometer::ConfigureAccel()
{
	//PX4_WARN("ConfigureAccel");
	const uint8_t ACC_RANGE = RegisterRead(Register::ACC_RANGE) & (Bit1 | Bit0);

	switch (ACC_RANGE) {
	case acc_range_3g:
		_px4_accel.set_scale(CONSTANTS_ONE_G * (powf(2, ACC_RANGE + 1) * 1.5f) / 32768.f);
		_px4_accel.set_range(3.f * CONSTANTS_ONE_G);
		break;

	case acc_range_6g:
		_px4_accel.set_scale(CONSTANTS_ONE_G * (powf(2, ACC_RANGE + 1) * 1.5f) / 32768.f);
		_px4_accel.set_range(6.f * CONSTANTS_ONE_G);
		break;

	case acc_range_12g:
		_px4_accel.set_scale(CONSTANTS_ONE_G * (powf(2, ACC_RANGE + 1) * 1.5f) / 32768.f);
		_px4_accel.set_range(12.f * CONSTANTS_ONE_G);
		break;

	case acc_range_24g:
		_px4_accel.set_scale(CONSTANTS_ONE_G * (powf(2, ACC_RANGE + 1) * 1.5f) / 32768.f);
		_px4_accel.set_range(24.f * CONSTANTS_ONE_G);
		break;
	}
}

bool BMI088_Accelerometer::Configure()
{

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

	ConfigureAccel();

	return success;
}

bool BMI088_Accelerometer::RegisterCheck(const register_config_t &reg_cfg)
{
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

void BMI088_Accelerometer::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

bool BMI088_Accelerometer::SelfTest()
{
	PX4_WARN("Running self-test with datasheet recomended steps(page 17)");
	// Reset
	PX4_WARN("Reseting the sensor");

	if (RegisterWrite(Register::ACC_SOFTRESET, 0xB6) == PX4_OK) {
		PX4_WARN("Reset success");
	}

	usleep(100000);
	PX4_WARN("Accel on");

	if (RegisterWrite(Register::ACC_PWR_CTRL, 0x04) == PX4_OK) {
		PX4_WARN("Accel on success");
	}

	usleep(100000);
	PX4_WARN("Sensor ErrReg: 0x%02x", CheckSensorErrReg());
	Configure();
	usleep(1000000);
	PX4_WARN("Sensor ErrReg: 0x%02x", CheckSensorErrReg());
	const uint8_t ACC_CHIP_ID = RegisterRead(Register::ACC_CHIP_ID);
	PX4_WARN("ACC_CHIP_ID: 0x%02x", ACC_CHIP_ID);
	usleep(30000);
	PX4_WARN("Sensor ErrReg: 0x%02x", CheckSensorErrReg());

	if (RegisterWrite(Register::ACC_PWR_CONF, 0) == PX4_OK) {
		PX4_WARN("Start sensor success");
		PX4_WARN("ACC_PWR_CONF(0): 0x%02x", RegisterRead(Register::ACC_PWR_CONF));
	}

	usleep(2000000);
	PX4_WARN("Sensor ErrReg: 0x%02x", CheckSensorErrReg());

	if (RegisterWrite(Register::ACC_RANGE, 0x03) == PX4_OK) {
		PX4_WARN("Range set success");
		PX4_WARN("ACC_RANGE(0x03): 0x%02x", RegisterRead(Register::ACC_RANGE));
	}

	usleep(100000);
	PX4_WARN("Sensor ErrReg: 0x%02x", CheckSensorErrReg());

	if (RegisterWrite(Register::ACC_CONF, 0xA7) == PX4_OK) {
		PX4_WARN("Conf set success");
		PX4_WARN("ACC_CONF(0xA7): 0x%02x", RegisterRead(Register::ACC_CONF));
	}

	usleep(100000);
	PX4_WARN("Sensor ErrReg: 0x%02x", CheckSensorErrReg());

	// Positive sel-test polarity
	if (RegisterWrite(Register::ACC_SELF_TEST, 0x0D) == PX4_OK) {
		PX4_WARN("Self-test positive mode set success");
		PX4_WARN("ACC_SELF_TEST(0x0D): 0x%02x", RegisterRead(Register::ACC_SELF_TEST));
	}

	usleep(100000);
	PX4_WARN("Sensor ErrReg: 0x%02x", CheckSensorErrReg());

	/*
	float *accel_mss = ReadAccelDataFIFO();
	PX4_WARN("Positive value");
	PX4_WARN("X %f", (double)accel_mss[0]);
	PX4_WARN("Y %f", (double)accel_mss[1]);
	PX4_WARN("Z %f", (double)accel_mss[2]);

	// Negative sel-test polarity
	if (RegisterWrite(Register::ACC_SELF_TEST, 0x09) == PX4_OK) {
		PX4_WARN("Self-test negative mode set success");
		PX4_WARN("ACC_SELF_TEST(0x09): 0x%02x", RegisterRead(Register::ACC_SELF_TEST));
	}

	usleep(600000);
	PX4_WARN("Sensor ErrReg: 0x%02x", CheckSensorErrReg());
	float *accel_mss2 = ReadAccelDataFIFO();
	PX4_WARN("Negative value");
	PX4_WARN("X %f", (double)accel_mss2[0]);
	PX4_WARN("Y %f", (double)accel_mss2[1]);
	PX4_WARN("Z %f", (double)accel_mss2[2]);

	// Calculate difference between positive and negative sef-test response
	float diff_x = accel_mss[0] - accel_mss2[0];
	float diff_y = accel_mss[1] - accel_mss2[1];
	float diff_z = accel_mss[2] - accel_mss2[2];

	PX4_WARN("Diff value");
	PX4_WARN("diff_x %f", (double)diff_x);
	PX4_WARN("diff_y %f", (double)diff_y);
	PX4_WARN("diff_z %f", (double)diff_z);

	if (diff_x >= 1000) {
		PX4_WARN("X Axis self-test success");
	}

	if (diff_y >= 1000) {
		PX4_WARN("Y Axis self-test success");
	}

	if (diff_z >= 500) {
		PX4_WARN("Z Axis self-test success");
	}
	*/

	// Disable self-test
	RegisterWrite(Register::ACC_SELF_TEST, 0x00);
	usleep(60000);

	PX4_WARN("Sensor ErrReg: 0x%02x", CheckSensorErrReg());
	// Reset
	//PX4_WARN("Reseting the sensor again");
	//RegisterWrite(Register::ACC_SOFTRESET, 0xB6);
	//usleep(100000);
	return true;
}

uint8_t  BMI088_Accelerometer::CheckSensorErrReg()
{
	return RegisterRead(Register::ACC_ERR_REG);
}

bool BMI088_Accelerometer::NormalRead(const hrt_abstime &timestamp_sample)
{
	//PX4_INFO_RAW("BMI088_Accelerometer::NormalRead()\n");

	float x = 0;
	float y = 0;
	float z = 0;
	uint8_t buffer[6] = {0};
	uint8_t cmd[1] = {static_cast<uint8_t>(Register::ACC_READ)};

	if (transfer(&cmd[0], 1, &buffer[0], 6) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	uint8_t RATE_X_LSB = buffer[0];
	uint8_t RATE_X_MSB = buffer[1];

	uint8_t RATE_Y_LSB = buffer[2];
	uint8_t RATE_Y_MSB = buffer[3];

	uint8_t RATE_Z_LSB = buffer[4];
	uint8_t RATE_Z_MSB = buffer[5];

	const int16_t accel_x = combine(RATE_X_MSB, RATE_X_LSB);
	const int16_t accel_y = combine(RATE_Y_MSB, RATE_Y_LSB);
	const int16_t accel_z = combine(RATE_Z_MSB, RATE_Z_LSB);

	if (accel_x == INT16_MIN) {
		PX4_WARN("accel_x == INT16_MIN");
	}

	if (accel_y == INT16_MIN) {
		PX4_WARN("accel_y == INT16_MIN");
	}

	if (accel_z == INT16_MIN) {
		PX4_WARN("accel_z == INT16_MIN");
	}

	if (accel_x == INT16_MIN || accel_y == INT16_MIN || accel_z == INT16_MIN) {
		PX4_WARN("Accel: INT16_MIN frame rejected");
		perf_count(_bad_transfer_perf);
		return false;
	}

	// sensor's frame is +x forward, +y left, +z up
	//  flip y & z to publish right handed with z down (x forward, y right, z down)
	x = accel_x;
	y = -accel_y;
	z = -accel_z;

	//PX4_WARN("x: %f | y: %f | z: %f", (double)x, (double)y ,(double)z);
	_px4_accel.update(timestamp_sample, x, y, z);

	return true;
}

} // namespace Bosch::BMI088::Accelerometer
