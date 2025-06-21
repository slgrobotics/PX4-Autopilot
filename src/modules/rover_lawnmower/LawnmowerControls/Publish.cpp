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

void LawnmowerControl::advertisePublishers()
{
	//_rover_velocity_setpoint_pub.advertise();

#ifdef DEBUG_MY_DATA
	// advertise debug array:
	_dbg_array.id = 1;
	strncpy(_dbg_array.name, "rover_dbg", 10);
	_pub_dbg_array = orb_advertise(ORB_ID(debug_array), &_dbg_array);

#endif // DEBUG_MY_DATA

}

#ifdef DEBUG_MY_DATA

void LawnmowerControl::publishDebugArray()
{
	_dbg_array.timestamp = _timestamp; // hrt_elapsed_time(&_app_started_time);

	orb_publish(ORB_ID(debug_array), _pub_dbg_array, &_dbg_array);
}

#endif // DEBUG_MY_DATA

} // namespace rover_lawnmower

