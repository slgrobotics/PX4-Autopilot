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
#include <px4_platform_common/log.h>

using namespace matrix;
namespace StanleyPursuit
{
float calcTargetBearing(pure_pursuit_status_s &pure_pursuit_status, const float xtrack_gain,
			const float lookahead_max, const float softening_factor, const Vector2f &curr_wp_ned, const Vector2f &prev_wp_ned,
			const Vector2f &curr_pos_ned, const float vehicle_speed)
{
//	return 0.0f; // Placeholder for the actual implementation

	//PX4_INFO_RAW("StanleyPursuit::calcTargetBearing()\n");

	// Check input validity:
	if (!curr_wp_ned.isAllFinite() || !curr_pos_ned.isAllFinite() || !PX4_ISFINITE(vehicle_speed)
	    || !prev_wp_ned.isAllFinite()) {
		return NAN;
	}

	const Vector2f P_to_C = curr_wp_ned - prev_wp_ned;
	const Vector2f curr_pos_to_curr_wp = curr_wp_ned - curr_pos_ned;

	PX4_INFO_RAW("        P: %f   %f\n", (double)prev_wp_ned(0), (double)prev_wp_ned(1));
	PX4_INFO_RAW("        C: %f   %f\n", (double)curr_wp_ned(0), (double)curr_wp_ned(1));
	PX4_INFO_RAW("        V: %f   %f\n", (double)curr_pos_ned(0), (double)curr_pos_ned(1));

	const float bearing_to_curr_waypoint = matrix::wrap_pi(atan2f(curr_pos_to_curr_wp(1), curr_pos_to_curr_wp(0)));

	PX4_INFO_RAW("P_to_C.length() %.2f m   V to C bearing: %f rad = %.1f deg\n", (double)P_to_C.length(),
			(double)bearing_to_curr_waypoint, (double)math::degrees(bearing_to_curr_waypoint));

	if (P_to_C.length() < 1.0e-6f) {

		// When P = C (overlapping waypoints), return bearing to C:

		PX4_INFO_RAW("P_to_C.length() very small.");

		if (curr_pos_to_curr_wp.length() < 1.0e-6f) {
			// vehicle on top of both C and P
			PX4_INFO_RAW("vehicle on top of both C and P.");
			return NAN;
		}

		// Just go to the current waypoint:
		return bearing_to_curr_waypoint;
	}

	// Stanley pursuit calculations:

	Vector2f P_to_V = curr_pos_ned - prev_wp_ned;

	Vector2f P_to_C_norm = P_to_C.normalized();

	const float crosstrack_error = P_to_C_norm %
				 P_to_V;   // "crosstrack" distance from Vehicle to the desired P--C trajectory, meters.

	const float parallel_heading = wrap_pi(atan2f(P_to_C(1), P_to_C(0)));	// angle to North vector (X axis)

	const float diff = wrap_pi(parallel_heading - bearing_to_curr_waypoint);

	float target_bearing = bearing_to_curr_waypoint; // assume fallback case

	PX4_INFO_RAW("crosstrack_error %f m  parallel_heading %f deg  diff %f deg   bearing_to_C %f deg\n",
			(double)crosstrack_error, (double)math::degrees(parallel_heading), (double)math::degrees(diff), (double)math::degrees(bearing_to_curr_waypoint));

	if (abs(diff) > M_PI_4_F) {
		// Fallback: Bearing to current waypoint if the computed path is parallel to P-C line
		PX4_WARN("Stanley: parallel_heading %f deg   bearing_to_C %f deg",
			 (double)math::degrees(parallel_heading), (double)math::degrees(bearing_to_curr_waypoint));
		PX4_WARN("Stanley: cannot reach target waypoint - bearing %f deg   diff %f deg", (double)math::degrees(bearing_to_curr_waypoint), (double)math::degrees(diff));

	} else {

		float xtrack_factor = -atanf(xtrack_gain * crosstrack_error /
					     (softening_factor + math::max(vehicle_speed, 0.0f)));


		PX4_INFO_RAW("PH: %.2f deg   Xtrk: %.1f cm  speed: %.3f   XtFctr: %f rad  %f deg   bearing_to_C: %f deg\n",
			     (double)math::degrees(parallel_heading),
			     (double)(crosstrack_error * 100.0f), (double)vehicle_speed, (double)xtrack_factor,
			     (double)math::degrees(xtrack_factor), (double)math::degrees(bearing_to_curr_waypoint));

		target_bearing = wrap_pi(parallel_heading + xtrack_factor);
	}

	PX4_INFO_RAW("target_bearing: %f rad = %.2f deg\n---------\n", (double)target_bearing, (double)math::degrees(target_bearing));

	pure_pursuit_status.lookahead_distance = NAN;
	pure_pursuit_status.target_bearing = target_bearing;
	pure_pursuit_status.crosstrack_error = crosstrack_error;
	pure_pursuit_status.distance_to_waypoint = curr_pos_to_curr_wp.norm();
	pure_pursuit_status.bearing_to_waypoint = bearing_to_curr_waypoint;
	return target_bearing;
}
} // namespace StanleyPursuit

