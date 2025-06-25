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

		_crosstrack_error = _pure_pursuit_status.crosstrack_error;
		float target_bearing = _pure_pursuit_status.target_bearing;
		float bearing_to_waypoint = _pure_pursuit_status.bearing_to_waypoint;
		_bearing_error = wrap_pi(target_bearing - bearing_to_waypoint);
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

	if (_global_local_proj_ref.isInitialized() && (_pos_sp_triplet.current.valid || _pos_sp_triplet.previous.valid
			|| _pos_sp_triplet.next.valid)) {

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

	if (_manual_control_setpoint_sub.updated() && _vehicle_control_mode.flag_control_manual_enabled) {

		// we come here only in Manual mode
		_manual_control_setpoint_sub.copy(&_manual_control_setpoint);

		/*
		struct manual_control_setpoint_s {
			uint64_t timestamp;
			uint64_t timestamp_sample;
			float roll;
			float pitch;
			float yaw;
			float throttle;
			float flaps;
			float aux1;
			float aux2;
			float aux3;
			float aux4;
			float aux5;
			float aux6;
			bool valid;
			uint8_t data_source;
			bool sticks_moving;
			uint8_t _padding0[1]; // required for logger
		*/

#ifdef DEBUG_MY_PRINT

		if (_tracing_lev > 4) {
			//PX4_INFO("Manual setpoint change detected:  Y (roll): %f     Z (throttle): %f", (double)_manual_control_setpoint.roll, (double)_manual_control_setpoint.throttle);
			//print_message(ORB_ID(manual_control_setpoint), _manual_control_setpoint);
		}

#endif // DEBUG_MY_PRINT

		// Set heading torque from the manual roll input channel. See RD_MAN_YAW_SCALE - for smoother manual control.
		_torque_control_manual = _manual_control_setpoint.roll;
		// * _param_rd_man_yaw_scale.get();	// Nominally yaw: _manual_control_setpoint.roll;  - this is right stick, horizontal movement, R/C CH1 "ailerons"
		// Set thrust from the manual throttle channel
		_thrust_control_manual =
			_manual_control_setpoint.throttle;	// this is right stick, vertical movement, R/C CH2 "elevator"

		// Here we ignore left stick movement - x (vertical) and r (horizontal) - those are mapped to Flight Mode and Arming functions. Could be useful in the future.

		// When channels are not under R/C Passthrough (like "param set PCA9685_FUNC3 406") - we collect R/C controls positions here, to pass them to servos in Manual mode:

		_cutter_setpoint_manual =
			_manual_control_setpoint.flaps;	// R/C CH7 "leftmost top switch" - param set RC_MAP_FLAPS 7 - cutter clutch
		_gas_throttle_manual =
			_manual_control_setpoint.aux1;	// R/C CH5 "left round knob"     - param set RC_MAP_AUX1 5  - gas engine throttle
		_alarm_dev_level_manual =
			_manual_control_setpoint.aux2;	// R/C CH6 "right round knob"    - param set RC_MAP_AUX2 6  - spare servo
	}

	if (_sensor_gps_sub.updated()) {
		// coming here at 10Hz
		_sensor_gps_sub.copy(&_sensor_gps_data);
#ifdef DEBUG_MY_PRINT

		if (_tracing_lev > 4) {
			//PX4_INFO("GPS data came");
			//print_message(ORB_ID(sensor_gps), _sensor_gps_data);
			/*
			 * print produces (comments from sensor_gps.msg):
			INFO  [rover_pos_control] GPS data came
			 sensor_gps
				timestamp: 90395226584 (0.012087 seconds ago)
				time_utc_usec: 1650659597200344
				device_id: 11141125 (Type: 0xAA, SERIAL:0 (0x00))
				lat: 33230009				# Latitude in 1E-7 degrees
				lon: -862038035				# Longitude in 1E-7 degrees
				alt: 157245					# Altitude in 1E-3 meters above MSL, (millimetres)
				alt_ellipsoid: 116743		# Altitude in 1E-3 meters bove Ellipsoid, (millimetres)
				s_variance_m_s: 0.0810		# GPS speed accuracy estimate, (metres/sec)
				c_variance_rad: 3.1416		# GPS course accuracy estimate, (radians)
				eph: 0.0140					# GPS horizontal position accuracy (metres)
				epv: 0.0100					# GPS vertical position accuracy (metres)
				hdop: 0.4900				# Horizontal dilution of precision
				vdop: 1.0400				# Vertical dilution of precision
				noise_per_ms: 71			# GPS noise per millisecond
				jamming_indicator: 11		# indicates jamming is occurring
				vel_m_s: 0.0200				# GPS ground speed, (metres/sec)
				vel_n_m_s: 0.0200			# GPS North velocity, (metres/sec)
				vel_e_m_s: 0.0020			# GPS East velocity, (metres/sec)
				vel_d_m_s: 0.0130			# GPS Down velocity, (metres/sec)
				cog_rad: 3.8606				# Course over ground (NOT heading, but direction of movement), -PI..PI, (radians) - !!! ACTUALLY, 0...2PI is delivered here !!!
				timestamp_time_relative: 0	# timestamp + timestamp_time_relative = Time of the UTC timestamp since system start, (microseconds)
				heading: nan				# heading angle of XYZ body frame rel to NED. Set to NaN if not available and updated (used for dual antenna GPS), (rad, [-PI, PI])
				heading_offset: 0.0000		# heading offset of dual antenna array in body frame. Set to NaN if not applicable. (rad, [-PI, PI])
				heading_accuracy: 0.0000	# heading accuracy (rad, [0, 2PI])
				automatic_gain_control: 0   # Automatic gain control monitor
				fix_type: 6					# 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 6: Real-Time Kinematic, fixed, 8: Extrapolated.
				jamming_state: 1			# indicates whether jamming has been detected or suspected by the receivers. O: Unknown, 1: OK, 2: Warning, 3: Critical
				vel_ned_valid: True			# True if NED velocity is valid
				satellites_used: 31			# Number of satellites used
			 */
		}

#endif // DEBUG_MY_PRINT
	}


#if defined(DEBUG_MY_PRINT) || defined(DEBUG_MY_DATA)

	if (_actuator_outputs_sub.updated()) {
		_actuator_outputs_sub.copy(&_actuator_outputs);

#ifdef DEBUG_MY_PRINT

		if (_tracing_lev > 4) {
			//PX4_INFO("Actuator_Outputs change detected");
			//print_message(ORB_ID(actuator_outputs), _actuator_outputs);
			/*
			 * print produces:
			INFO  [rover_pos_control] Actuator_Outputs change detected
			 actuator_outputs
				timestamp: 105617561109 (0.002838 seconds ago)
				noutputs: 8
				output: [1500.0000, 1500.0000, 800.0000, 1000.0000, 800.0000, 1500.0000, 1500.0000, 1500.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000]

			 */
		}

#endif // DEBUG_MY_PRINT

		_wheel_left_servo_position  = _actuator_outputs.output[0];		// PCA9685 Channel 1
		_wheel_right_servo_position  = _actuator_outputs.output[1];		// PCA9685 Channel 2

		_cutter_servo_position = _actuator_outputs.output[2];			// PCA9685 Channel 3 - cutter
		_gas_throttle_servo_position = _actuator_outputs.output[3];		// PCA9685 Channel 4
		_alarm_servo_position = _actuator_outputs.output[5];			// PCA9685 Channel 6 - alarm

	}

#endif // defined(DEBUG_MY_PRINT) || defined(DEBUG_MY_DATA)


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

