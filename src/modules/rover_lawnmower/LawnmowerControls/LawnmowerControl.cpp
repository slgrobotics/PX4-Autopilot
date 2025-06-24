/****************************************************************************
 *
 *   Copyright (c) 2025 Sergei Grichine (slgrobotics). All rights reserved.
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

#include "LawnmowerControl.hpp"


namespace rover_lawnmower
{

LawnmowerControl::LawnmowerControl(ModuleParams *parent) : ModuleParams(parent)
{
	advertisePublishers();

	updateParams();
}

void LawnmowerControl::updateParams()
{
	ModuleParams::updateParams();
}

void LawnmowerControl::updateLawnmowerControl(vehicle_control_mode_s vehicle_control_mode)
{
	_vehicle_control_mode = vehicle_control_mode;

	updateSubscriptions();	// Update uORB subscriptions, poll vehicle attitude, position etc.

	const hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	_dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;

	//PX4_INFO_RAW("---  dt: %f\n", (double)_dt);

	vehicleControl(); // Perform vehicle control, this is where the main logic goes including state machine

	publishAuxActuators(); // Publish auxiliary actuators, like cutter, gas engine throttle, etc.

#ifdef DEBUG_MY_PRINT
	debugPrint();
#endif // DEBUG_MY_PRINT

#ifdef DEBUG_MY_DATA
	publishDebugData();
#endif // DEBUG_MY_DATA

}

}
