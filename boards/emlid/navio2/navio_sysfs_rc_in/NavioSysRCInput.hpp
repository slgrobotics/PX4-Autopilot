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

#pragma once

#include <drivers/device/i2c.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/input_rc.h>

namespace navio_sysfs_rc_in
{

class NavioSysRCInput : public device::I2C, public px4::ScheduledWorkItem
{
public:
	NavioSysRCInput();
	~NavioSysRCInput() override;

	/* @return 0 on success, -errno on failure */
	int start();

	/* @return 0 on success, -errno on failure */
	void stop();

	int print_status();

	bool isRunning() { return _isRunning; }

private:
	void Run() override;

	int navio_rc_init();

	px4::atomic_bool _should_exit{false};

	bool _isRunning{false};

	uORB::PublicationMulti<input_rc_s> _input_rc_pub{ORB_ID(input_rc)};

	static constexpr int CHANNELS{8};

	input_rc_s data{};

	// arming switch on left stick "rudder" (right sweep - arm, left sweep - disarm)
	// we need to keep state here, as left stick is spring-loaded
	bool _ch4_switch_state{false};

	uint64_t _timestamp_last_signal{0};

	bool _connected{false};

	bool _lastRcLost{true};

	float _rssi{0};
	float _cnt_good{1.0f};
	float _cnt_bad{1.0f};

	perf_counter_t _publish_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": publish interval")};

	perf_counter_t		_comms_errors;

};

}; // namespace navio_sysfs_rc_in
