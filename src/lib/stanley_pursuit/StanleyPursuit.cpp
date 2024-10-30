/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "StanleyPursuit.hpp"
#include <mathlib/mathlib.h>


StanleyPursuit::StanleyPursuit(ModuleParams *parent) : ModuleParams(parent)
{
	_param_handles.xtrack_gain = param_find("ST_XTRACK_GAIN");
	_param_handles.softening_factor = param_find("ST_SOFTENING");
	updateParams();
}

void StanleyPursuit::updateParams()
{
	param_get(_param_handles.xtrack_gain, &_params.xtrack_gain);
	param_get(_param_handles.softening_factor, &_params.softening_factor);

	ModuleParams::updateParams();
}

float StanleyPursuit::calcDesiredHeading(const Vector2f &curr_wp_ned, const Vector2f &prev_wp_ned,
		const Vector2f &curr_pos_ned,
		const float vehicle_speed)
{
	//PX4_INFO_RAW("StanleyPursuit::calcDesiredHeading()\n");

	// Check input validity:
	if (!curr_wp_ned.isAllFinite() || !curr_pos_ned.isAllFinite() || vehicle_speed < -FLT_EPSILON
	    || !PX4_ISFINITE(vehicle_speed) || !prev_wp_ned.isAllFinite()) {
		return NAN;
	}

	const Vector2f P_to_C = curr_wp_ned - prev_wp_ned;
	const Vector2f to_C_norm = (curr_wp_ned - curr_pos_ned).normalized();

	if (P_to_C.length() < 1.0e-6f) {

		// When P = C (overlapping waypoints), return bearing to C:

		if (to_C_norm.length() < 1.0e-6f) {
			// vehicle on top of C and P
			return NAN;
		}

		float heading_to_C = wrap_pi(atan2f(to_C_norm(1), to_C_norm(0)));

		//PX4_INFO_RAW("V: %f   %f\n", (double)curr_pos_ned(0), (double)curr_pos_ned(1));
		//PX4_INFO_RAW("to_C_norm: %f   %f\n", (double)to_C_norm(0), (double)to_C_norm(1));
		PX4_INFO_RAW("heading_to_C V to C: %f   %f\n", (double)heading_to_C, (double)math::degrees(heading_to_C));

		return heading_to_C;
	}

	// Stanley pursuit calculations:

	Vector2f P_to_V = curr_pos_ned - prev_wp_ned;

	Vector2f P_to_C_norm = P_to_C.normalized();

	float crosstrack_error = P_to_C_norm %
				 P_to_V;   // "crosstrack" distance from Vehicle to the desired P--C trajectory, meters.

	float parallel_heading = wrap_pi(atan2f(P_to_C(1), P_to_C(0)));	// angle to North vector (X axis)

	//float bearing_to_C = wrap_pi(atan2f(to_C_norm(1), to_C_norm(0)));

	/* ==========================================

	float diff = wrap_pi(parallel_heading - bearing_to_C);

	if (abs(diff) > M_PI_4_F) {
		PX4_WARN("Stanley: cannot reach target waypoint - bearing %f deg", (double)math::degrees(bearing_to_C));
		return bearing_to_C;
	}

	// ========================================== */

	float xtrack_factor = -atanf(_params.xtrack_gain * crosstrack_error / (_params.softening_factor + math::max(
					     vehicle_speed, 0.0f)));


	//PX4_INFO_RAW("H: %.2f deg   X: %.1f cm  speed: %.3f   XF: %f rad  %f deg  to_C: %f deg\n",
	//	     (double)math::degrees(parallel_heading),
	//	     (double)(crosstrack_error * 100.0f), (double)vehicle_speed, (double)xtrack_factor,
	//	     (double)math::degrees(xtrack_factor), (double)math::degrees(bearing_to_C));

	float desired_heading = wrap_pi(parallel_heading + xtrack_factor);

	//PX4_INFO_RAW("desired_heading: %.2f deg\n", (double)math::degrees(desired_heading));

	return desired_heading;
}
