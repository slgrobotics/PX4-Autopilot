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

void LawnmowerControl::updateSubscriptions()
{
	// Update uORB subscriptions:
	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		matrix::Quatf vehicle_attitude_quaternion = matrix::Quatf(vehicle_attitude.q);
		_vehicle_yaw = matrix::Eulerf(vehicle_attitude_quaternion).psi();
	}

	if (_global_position_sub.updated()) {
		_global_position_sub.copy(&_global_pos);

		//print_message(ORB_ID(vehicle_global_position), _global_pos);

		/*
		vehicle_global_position
		timestamp: 9524000 (0.004000 seconds ago)
		timestamp_sample: 9524000 (0 us before timestamp)
		lat: 47.397971
		lon: 8.546164
		alt: 0.21325
		alt_ellipsoid: 0.21325
		delta_alt: 0.18699
		delta_terrain: 0.18699
		eph: 0.15780
		epv: 0.19040
		terrain_alt: 0.11394
		lat_lon_valid: True
		alt_valid: True
		lat_lon_reset_counter: 4
		alt_reset_counter: 3
		terrain_reset_counter: 3
		terrain_alt_valid: False
		dead_reckoning: False
		*/

		// see how far EKF-calculated position is from RTK GPS position:
		//updateEkfGpsDeviation();
	}

	if (_vehicle_local_position_sub.updated()) {
		_vehicle_local_position_sub.copy(&_vehicle_local_position);
		_curr_pos_ned = Vector2f(_vehicle_local_position.x, _vehicle_local_position.y);
		_curr_pos = Vector2d(_vehicle_local_position.x, _vehicle_local_position.y);

		if (!_global_local_proj_ref.isInitialized()
		    || (_global_local_proj_ref.getProjectionReferenceTimestamp() != _vehicle_local_position.ref_timestamp)) {

			_global_local_proj_ref.initReference(_vehicle_local_position.ref_lat, _vehicle_local_position.ref_lon,
							     _vehicle_local_position.ref_timestamp);
		}
	}

	if (_pure_pursuit_status_sub.updated()) {
		_pure_pursuit_status_sub.copy(&_pure_pursuit_status);
	}

	if (_position_setpoint_triplet_sub.updated()) {
		_position_setpoint_triplet_sub.copy(&_pos_sp_triplet);

#ifdef DEBUG_MY_PRINT

		if (_vehicle_control_mode.flag_control_auto_enabled
			&& _vehicle_control_mode.flag_armed
			&& _tracing_lev > 4) {

			PX4_INFO("Position setpoint triplet change detected  Valid:  curr: %s   prev: %s  next: %s",
					_pos_sp_triplet.current.valid ? "yes" : "no",
					_pos_sp_triplet.previous.valid ? "yes" : "no",
					_pos_sp_triplet.next.valid ? "yes" : "no"
				);
			//print_message(ORB_ID(position_setpoint_triplet), _pos_sp_triplet);
				/*
				* print produces (three of these):
				*
							current (position_setpoint):
							timestamp: 34820000 (0.004000 seconds ago)
							lat: 33.2303
							lon: -86.2088
							vx: 0.0000
							vy: 0.0000
							vz: 0.0000
							alt: 145.6315
							yaw: nan
							yawspeed: 0.0000
							loiter_radius: 0.3000
							acceptance_radius: 0.5000
							cruising_speed: -1.0000
							cruising_throttle: nan
							valid: True
							type: 2                     <- LOITER, destination waypoint
							velocity_valid: False
							velocity_frame: 0
							alt_valid: False
							yaw_valid: False
							yawspeed_valid: False
							landing_gear: 0
							loiter_direction: 1
							disable_weather_vane: False
				*
				* For a setpoint "type":
				*
						POSITION_SETPOINT_SETPOINT_TYPE_POSITION 0
						POSITION_SETPOINT_SETPOINT_TYPE_VELOCITY 1
						POSITION_SETPOINT_SETPOINT_TYPE_LOITER 2
						POSITION_SETPOINT_SETPOINT_TYPE_TAKEOFF 3
						POSITION_SETPOINT_SETPOINT_TYPE_LAND 4
						POSITION_SETPOINT_SETPOINT_TYPE_IDLE 5
						POSITION_SETPOINT_SETPOINT_TYPE_FOLLOW_TARGET 6
				*/
		}

#endif // DEBUG_MY_PRINT

	}

	if (_global_local_proj_ref.isInitialized() && (_pos_sp_triplet.current.valid || _pos_sp_triplet.previous.valid || _pos_sp_triplet.next.valid)) {

		updateWaypoints();

		updateWaypointDistances();

		//setMaxLegSpeed();
	} else {
		//PX4_WARN("Position setpoint triplet is not valid, no waypoints available");
		_curr_wp = Vector2d(NAN, NAN);
		_prev_wp = Vector2d(NAN, NAN);
		_next_wp = Vector2d(NAN, NAN);
		_curr_wp_ned = Vector2f(NAN, NAN);
		_prev_wp_ned = Vector2f(NAN, NAN);
		_wp_current_dist = NAN;
		_wp_previous_dist = NAN;
		_wp_next_dist = NAN;
	}
}


void LawnmowerControl::updateWaypoints()
{
	// we only come here if _global_local_proj_ref.isInitialized() is true

	// Global waypoint coordinates
	if (_pos_sp_triplet.current.valid && PX4_ISFINITE(_pos_sp_triplet.current.lat)
	    && PX4_ISFINITE(_pos_sp_triplet.current.lon)) {
		_curr_wp = Vector2d(_pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon);

	} else {
		_curr_wp = Vector2d(0, 0);
	}

	if (_pos_sp_triplet.previous.valid && PX4_ISFINITE(_pos_sp_triplet.previous.lat)
	    && PX4_ISFINITE(_pos_sp_triplet.previous.lon)) {
		_prev_wp = Vector2d(_pos_sp_triplet.previous.lat, _pos_sp_triplet.previous.lon);

	} else {
		_prev_wp = _curr_pos; // this is first leg - towards the first waypoint
	}

	if (_pos_sp_triplet.next.valid && PX4_ISFINITE(_pos_sp_triplet.next.lat)
	    && PX4_ISFINITE(_pos_sp_triplet.next.lon)) {
		_next_wp = Vector2d(_pos_sp_triplet.next.lat, _pos_sp_triplet.next.lon);

	//} else {
	//	_next_wp = _home_position;
	}

	// NED waypoint coordinates
	_curr_wp_ned = _global_local_proj_ref.project(_curr_wp(0), _curr_wp(1));
	_prev_wp_ned = _global_local_proj_ref.project(_prev_wp(0), _prev_wp(1));
}

void LawnmowerControl::updateWaypointDistances()
{
	// Calculate distances to waypoints
	if (_pos_sp_triplet.current.valid && PX4_ISFINITE(_pos_sp_triplet.current.lat)
	    && PX4_ISFINITE(_pos_sp_triplet.current.lon)) {
		_wp_current_dist = get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon,
				   _pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon);

	} else {
		_wp_current_dist = NAN;
	}

	// we use previously saved _prev_wp, it helps with the first leg, when _pos_sp_triplet.previous.valid is false:
	if (PX4_ISFINITE(_prev_wp(0)) && PX4_ISFINITE(_prev_wp(1))) {
		_wp_previous_dist = get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon,
				    _prev_wp(0), _prev_wp(1));

	} else {
		_wp_previous_dist = NAN;
	}

	if (_pos_sp_triplet.next.valid && PX4_ISFINITE(_pos_sp_triplet.next.lat)
	    && PX4_ISFINITE(_pos_sp_triplet.next.lon)) {
		_wp_next_dist = get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon,
				_pos_sp_triplet.next.lat, _pos_sp_triplet.next.lon);

	} else {
		_wp_next_dist = NAN;
	}
}

} // namespace rover_lawnmower

