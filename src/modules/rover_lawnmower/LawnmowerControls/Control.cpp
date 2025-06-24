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

void LawnmowerControl::vehicleControl()
{

	if (_vehicle_control_mode.flag_control_manual_enabled) {

		if(_pos_ctrl_state != POS_STATE_IDLE) {

			unwindStateMachine();
		}

		if (_vehicle_control_mode.flag_armed) {

			_gas_engine_throttle = _gas_throttle_manual;	// Left knob on R/C transmitter
			_alarm_dev_level = _alarm_dev_level_manual;	// Horn - right knob on R/C transmitter

			// for cutter deck clutch we have direct R/C to servo setting, no need to use _cutter_setpoint_manual
			//_cutter_setpoint = _cutter_setpoint_manual;	// Cutter - right switch on R/C transmitter

			//} else {
			//PX4_WARN("Manual control enabled, but vehicle is not armed");
		}

	} else if (_vehicle_control_mode.flag_control_auto_enabled) {

		if (_vehicle_control_mode.flag_armed) {

			// Note: Gas engine, cutter deck clutch and alarm device are controlled by the state machine.

			workStateMachine();

			//} else {
			//PX4_WARN("Auto control enabled, but vehicle is not armed");
		}

	} else if (_vehicle_control_mode.flag_control_offboard_enabled) {

		PX4_WARN("Offboard control enabled, but not implemented yet");

	} else {

		PX4_WARN("Vehicle control mode not set");
	}
}

} // namespace rover_lawnmower

