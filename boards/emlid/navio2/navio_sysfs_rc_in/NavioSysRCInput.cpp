/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
 *   Copyright (C) 2016 PX4 Development Team. All rights reserved.
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

// - modified by Sergei Grichine Sept 2020

#include "NavioSysRCInput.hpp"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

using namespace time_literals;

#define DRV_RC_DEVTYPE_RCNAVIO2	0x8a

#define TEENSY_BUS				1       // 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)
#define TEENSY_ADDR				0x48	// I2C adress
#define TEENSY_REG				0x00
#define I2C_BUS_FREQUENCY		400000

#define MYDEBUG

namespace navio_sysfs_rc_in
{

NavioSysRCInput::NavioSysRCInput() :
	I2C(DRV_RC_DEVTYPE_RCNAVIO2, MODULE_NAME, TEENSY_BUS, TEENSY_ADDR, I2C_BUS_FREQUENCY),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err"))

{
	PX4_INFO("NavioSysRCInput::NavioSysRCInput()");
	_isRunning = true;
	//memset(data, 0, sizeof(data));

};

NavioSysRCInput::~NavioSysRCInput()
{
	ScheduleClear();

	_isRunning = false;

	PX4_INFO("NavioSysRCInput::~NavioSysRCInput()");

	perf_free(_publish_interval_perf);
}

int NavioSysRCInput::navio_rc_init()
{
	PX4_INFO("NavioSysRCInput::navio_rc_init()");

	int ret;
	ret = I2C::init();

	if (ret != OK) {
		return ret;
	}

	ScheduleNow();

	return ret;
}

int NavioSysRCInput::start()
{
	PX4_INFO("NavioSysRCInput::start()");

	navio_rc_init();

	_should_exit.store(false);

	ScheduleOnInterval(10_ms); // 100 Hz

	return PX4_OK;
}

void NavioSysRCInput::stop()
{
	PX4_INFO("NavioSysRCInput::stop()");

	_should_exit.store(true);
}

void NavioSysRCInput::Run()
{
	if (_should_exit.load()) {
		ScheduleClear();
		return;
	}

	data.rc_lost = false;

	uint8_t _block[18];

	// here we receive data block from Teensy 3.1 via I2C
	// see https://github.com/slgrobotics/Misc/blob/master/Arduino/Sketchbook/RC_PPM_Receiver/RC_PPM_Receiver.ino

	int teensy_data_ok = transfer(nullptr, 0, _block, 18);		// returns 0 (=OK) on success

	if (OK == teensy_data_ok) {
		uint16_t raw_values[9];

		// 8 decoded PPM channels plus a "fake channel" to indicate receiver not receiving from transmitter (setting data.rc_lost below)
		for (int i = 0; i < 9; ++i) {
			int j = i << 1;
			raw_values[i] = ((uint16_t)_block[j]) + (((uint16_t)_block[j + 1]) << 8);
		}

		// check if all channels are within the 800..2200 range:
		for (int i = 0; i < CHANNELS; ++i) {
			if (raw_values[i] < 800 || raw_values[i] > 2200) {
				perf_count(_comms_errors);
				// keep it silent, as it happens a lot. Look at RSSI for a feel.
				//PX4_ERR("NavioSysRCInput::Run() - I2C transfer() CH%d  bad value %d", i+1, raw_values[i]);
				//DEVICE_LOG("RC i2c::transfer I2C transfer() CH%d  bad value %d", i+1, raw_values[i]);
				teensy_data_ok = ERROR; // bad cycle
			}

#ifdef MYDEBUG
			// apply a little noise so that I can see it on pca9685 output:
			int i_myrand = _timestamp_last_signal & 0xF;
			raw_values[i] += i_myrand;
#endif
		}

		// all good, copy to data for publishing:
		if (OK == teensy_data_ok) {
			_timestamp_last_signal = hrt_absolute_time();

			_cnt_good += 1.0f;

			for (int i = 0; i < 9; ++i) {
				uint16_t raw_value = raw_values[i];

				if (i == 3) {
					// CH4 (left stick "rudder") used as ARM switch (right sweep - arm, left sweep - disarm)
					// make sure RC_MAP_ARM_SW is set to 4
					// toggle switch here:
					if (_ch4_switch_state && raw_value < 1200) {
						_ch4_switch_state = false;
						PX4_INFO("CH4_switch_state: DISARM");

					} else if (!_ch4_switch_state && raw_value > 1800) {
						_ch4_switch_state = true;
						PX4_INFO("CH4_switch_state: ARM");
					}

					//raw_value = (_ch4_switch_state ? 1900 : 1100) + (_timestamp_last_signal & 0xf);
					raw_value = _ch4_switch_state ? 2000 : 1000;
				}

				data.values[i] = raw_value;
			}

		} else {
			data.rc_lost = true;
		}

	} else {
		data.rc_lost = true;

		perf_count(_comms_errors);
		PX4_ERR("NavioSysRCInput::Run() - I2C transfer() from Teensy returned %d", teensy_data_ok);
		DEVICE_LOG("RC i2c::transfer returned %d", teensy_data_ok);
	}

	// data.values[0-6] - channel 1..7 values, must be within 800..2200 range
	// data.values[7] - channel 8 value, programmed on the receiver to be 1999 on fail (also 1999 when second left switch down)
	// data.values[8] - milliseconds since Teensy last received valid PPM signal from the receiver

	// R/C channel 8 is also failsafe indicator, as programmed in the receiver.
	// The second left switch (ch8) still works, emulating failsafe in down position.
	// values for switch up: 999 down: 1999
	// the "ch9" value is milliseconds since Teensy last received valid PPM signal from the receiver, normally 1..20

#ifndef MYDEBUG
	data.rc_lost = (OK != teensy_data_ok) || (data.values[7] > 1700 && data.values[7] <= 2200) || (data.values[8] > 100);
#endif

	if (OK != teensy_data_ok && _cnt_good > 1.5f) {
		_cnt_bad += 1.0f;
	}

	_lastRcLost = data.rc_lost;
	_rssi = _cnt_good < 1.5f ? 0.0f : 100.0f * _cnt_good / (_cnt_good + _cnt_bad);

	data.timestamp_last_signal = _timestamp_last_signal;
	data.channel_count = CHANNELS;
	data.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_PPM;
	data.link_quality = -1;
	data.rssi_dbm = NAN;
	data.rssi = (int)_rssi;	// percent. Should be around 99%, less than 2% lost or damaged I2C transfers
	data.rc_failsafe = false;
	data.rc_lost_frame_count = 0;
	data.rc_total_frame_count = 1;

	data.timestamp = hrt_absolute_time();

	_input_rc_pub.publish(data);

	perf_count(_publish_interval_perf);
}

int NavioSysRCInput::print_status()
{
	PX4_INFO("Running: %s",  _isRunning ? "yes" : "no");
	PX4_INFO("Teensy on I2C bus: %d addr: 0x%x  R/C Lost: %s", TEENSY_BUS, TEENSY_ADDR, _lastRcLost ? "true" : "false");
	PX4_INFO("CH4_switch_state: %s", _ch4_switch_state ? "ARM" : "DISARM");
	PX4_INFO("RSSI: %f   good: %d   bad: %d", (double)_rssi, (int)_cnt_good, (int)_cnt_bad);

	print_run_status();		// Scheduler rate and status

	perf_print_counter(_publish_interval_perf);
	perf_print_counter(_comms_errors);

	return 0;
}

}; // namespace navio_sysfs_rc_in
