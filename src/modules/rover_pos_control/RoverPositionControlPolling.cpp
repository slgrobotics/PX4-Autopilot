/****************************************************************************
 *
 *   Copyright (c) 2017-2021 PX4 Development Team. All rights reserved.
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

/**
 *
 * This module is a modification of the fixed wing module and it is designed for ground rovers.
 * It has been developed starting from the fw module, simplified and improved with dedicated items.
 *
 * All the acknowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Marco Zorzi <mzorzi@student.ethz.ch>
 *
 * @author Modified for heavy lawnmower by Sergei Grichine <slg@quakemap.com>
 *
 */


#include "RoverPositionControl.hpp"

using namespace matrix;

void RoverPositionControl::poll_everything()
{
	// =============== Collect all pose and state info needed for control: ===================================

	/* update parameters from storage, if changed */
	if (_parameter_update_sub.updated()) {

		// clear update
		parameter_update_s parameter_update;
		_parameter_update_sub.copy(&parameter_update);

		// update parameters from storage
		ModuleParams::updateParams();

		// update our PIDs etc:
		updateParams();
	}

	_vehicle_angular_velocity_sub.update(&_angular_velocity);

	_z_yaw_rate = _angular_velocity.xyz[2];	// for logging/tracing

	_vehicle_acceleration_sub.update();

	/* check vehicle control mode for changes */
	vehicle_control_mode_poll();
	attitude_setpoint_poll();			// autonomous inputs
	//rates_setpoint_poll();				// only mavlink would publish this, no need to process here yet

	vehicle_attitude_poll();

	// see what's happening upstairs:
	//mission_poll();						// never published?
	mission_result_poll();				// most informative (tells when mission finished)
	//navigator_mission_item_poll();		// very informative, but not useful
	vehicle_status_poll();				// published at 1Hz, nav_state 0...3 usually

	actuator_outputs_poll();			// where gas throttle is

	_wheel_left_servo_position  = _actuator_outputs.output[0];		// PCA9685 Channel 1
	_wheel_right_servo_position  = _actuator_outputs.output[1];		// PCA9685 Channel 2

	_cutter_servo_position = _actuator_outputs.output[2];			// PCA9685 Channel 3 - cutter
	_gas_throttle_servo_position = _actuator_outputs.output[3];		// PCA9685 Channel 4
	_alarm_servo_position = _actuator_outputs.output[5];			// PCA9685 Channel 6 - alarm

	// store raw GPS and magnetometer data, if available:
	gps_poll();
	magnetometer_poll();

#ifdef DEBUG_MY_PRINT

	float gps_fix_warn_period = _control_mode.flag_armed ? 2.0f : 20.0f; // seconds

	// See GND_GPS_MINFIX
	if (_sensor_gps_data.fix_type < _gps_minfix
	    && hrt_elapsed_time(&_debug_gps_warn_last_called) > gps_fix_warn_period * 1000_ms) {
		PX4_WARN("Bad GPS fix: %d (must be %d)  jamming: state: %d indicator: %d   noise_per_ms: %d",
			 _sensor_gps_data.fix_type, _gps_minfix, _sensor_gps_data.jamming_state, _sensor_gps_data.jamming_indicator,
			 _sensor_gps_data.noise_per_ms);

		_debug_gps_warn_last_called = _now;
	}

#endif // DEBUG_MY_PRINT

	// See GND_HD_MEAS_MODE, GND_SP_MEAS_MODE and GND_EKF_OVERRIDE:

	_gps_current_heading = _sensor_gps_data.heading;	// only valid for dual antenna RTK, otherwise NAN

	if (PX4_ISFINITE(_gps_current_heading)) {
		// We have dual antenna heading and precise RTK positioning:
		_gps_current_heading = wrap_pi(_gps_current_heading);
		_gps_ground_speed_abs = _sensor_gps_data.vel_ned_valid ? _sensor_gps_data.vel_m_s : NAN;

	} else {
		// When dual antenna heading is missing, try using GPS course-over-ground (cog_rad):
		_gps_ground_speed_abs = NAN;

		if (_sensor_gps_data.fix_type >= _gps_minfix && _sensor_gps_data.vel_ned_valid) {
			//matrix::Vector2f t_vel(_sensor_gps_data.vel_n_m_s, _sensor_gps_data.vel_e_m_s);
			//_gps_ground_speed_abs = t_vel.length();

			//_gps_ground_speed_abs = std::sqrt(sq(_sensor_gps_data.vel_n_m_s) + sq(_sensor_gps_data.vel_e_m_s));

			_gps_ground_speed_abs = _sensor_gps_data.vel_m_s;

			if (abs(_gps_ground_speed_abs) >= 0.05f) {
				// only compute "velocity heading" when moving fast enough. Wrap it to -PI...PI range:

				//_gps_current_heading = wrap_pi(atan2f(_sensor_gps_data.vel_e_m_s, _sensor_gps_data.vel_n_m_s));
				_gps_current_heading = wrap_pi(_sensor_gps_data.cog_rad);
			}
		}
	}

	// store position change:
	if (_local_pos_sub.update(&_local_pos)) {

		_ekf_data_good = _local_pos.xy_valid && _local_pos.v_xy_valid && _local_pos.heading_good_for_control;
		_ekf_flags = {_local_pos.xy_valid, _local_pos.v_xy_valid, _local_pos.heading_good_for_control};
	}

	if (_global_pos_sub.update(&_global_pos)) {
		// see how far EKF-calculated position is from RTK GPS position:
		updateEkfGpsDeviation();
	}

	if (_ekf_override_by_gps && _sensor_gps_data.fix_type == 6) {	// GND_EKF_OVERRIDE

		// do some brute force substitutions of EKF estimates with precise RTK GPS data.

		_global_pos.lat = _sensor_gps_data.latitude_deg;
		_global_pos.lon = _sensor_gps_data.longitude_deg;
		_global_pos.alt = _sensor_gps_data.altitude_msl_m;

		float gps_local_x = get_distance_to_next_waypoint(
					    _global_pos.lat, _global_pos.lon,
					    _local_pos.ref_lat, _global_pos.lon);

		float gps_local_y = get_distance_to_next_waypoint(
					    _global_pos.lat, _local_pos.ref_lon,
					    _global_pos.lat, _global_pos.lon);

		//PX4_WARN("Local Pos: x: %.3f / %.3f   y: %.3f / %.3f",
		//	 (double)gps_local_x, (double)_local_pos.x, (double)gps_local_y, (double)_local_pos.y);

		_local_pos.x = gps_local_x;
		_local_pos.y = gps_local_y;
		_local_pos.xy_valid = true;

		if (PX4_ISFINITE(_gps_current_heading)) {
			_local_pos.heading = _sensor_gps_data.heading;
			_local_pos.heading_good_for_control = true;
		}

		if (_sensor_gps_data.vel_ned_valid) {
			PX4_INFO_RAW("V subst:   X: %.3f -> %.3f   Y: %.3f -> %.3f\n", (double)_local_pos.vx,
				     (double)_sensor_gps_data.vel_e_m_s, (double)_local_pos.vy, (double)_sensor_gps_data.vel_n_m_s);
			_local_pos.vx = _sensor_gps_data.vel_e_m_s;
			_local_pos.vy = _sensor_gps_data.vel_n_m_s;
		}
	}

	position_setpoint_triplet_poll();	// autonomous inputs - goal waypoint

	if (!_global_local_proj_ref.isInitialized()
	    || (_global_local_proj_ref.getProjectionReferenceTimestamp() != _local_pos.ref_timestamp)) {

		_global_local_proj_ref.initReference(_local_pos.ref_lat, _local_pos.ref_lon,
						     _local_pos.ref_timestamp);
	}

	/*
	 *  This is part of control_velocity() code, removed for now
	 *
	// Convert Local setpoints to global setpoints
	if (_control_mode.flag_control_offboard_enabled) {
		// this is for direct MAVLINK control:

		_trajectory_setpoint_sub.update(&_trajectory_setpoint);

		const Vector3f desired_velocity{_trajectory_setpoint.velocity};

		// local -> global
		_global_local_proj_ref.reproject(
			_trajectory_setpoint.position[0], _trajectory_setpoint.position[1],
			_pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon);

		_pos_sp_triplet.current.valid = true;
	}
	*/
}

void RoverPositionControl::updateParams()
{
	PX4_INFO("Parameters update detected");

	_tracing_lev = _param_tracing_lev.get();	// GND_TRACING_LEV
	_gps_minfix = _param_gps_minfix.get();		// GND_GPS_MINFIX - Minimal acceptable GPS fix for heading calculations

	_speed_prefer_gps = _param_speed_measurement_mode.get() == 1;		// GND_SP_MEAS_MODE
	_heading_prefer_gps = _param_heading_measurement_mode.get() == 1;	// GND_HD_MEAS_MODE
	_ekf_override_by_gps = _param_ekf_override_by_gps.get() == 1;		// GND_EKF_OVERRIDE

	_ekf_heading_correction = math::radians(_param_heading_err_decl.get());	// GND_HEADING_DECL

	_gnd_control.set_l1_damping(_param_l1_damping.get());	// GND_L1_DAMPING
	_gnd_control.set_l1_period(_param_l1_period.get());	// GND_L1_PERIOD

	// To support Differential Drive module logic, whole body max speed and turn rate:
	_rdd_max_speed = _param_rdd_max_wheel_speed.get() * _param_rdd_wheel_radius.get();	// RDD_WHEEL_SPEED, RDD_WHEEL_RADIUS
	_rdd_max_angular_velocity = _rdd_max_speed / (_param_rdd_wheel_base.get() / 2.f);	// RDD_WHEEL_BASE

	_differential_drive_kinematics.setWheelBase(_param_rdd_wheel_base.get());	// RDD_WHEEL_BASE
	_differential_drive_kinematics.setMaxSpeed(_rdd_max_speed);
	_differential_drive_kinematics.setMaxAngularVelocity(_rdd_max_angular_velocity);

	// to provide for Line Following when in modified L1 mode:
	pid_init(&_line_following_ctrl, PID_MODE_DERIVATIV_CALC, 0.01f);
	pid_set_parameters(&_line_following_ctrl,
			   _param_line_following_p.get(),	// GND_LF_P
			   _param_line_following_i.get(),
			   _param_line_following_d.get(),
			   _param_line_following_imax.get(),
			   _param_line_following_max.get());

	pid_reset_integral(&_line_following_ctrl);

	// PID_MODE_DERIVATIV_CALC calculates discrete derivative from previous error, val_dot in pid_calculate() will be ignored

	// to stabilize speed at desired level, given by GND_SPEED_TRIM (m/s):
	pid_init(&_speed_ctrl, PID_MODE_DERIVATIV_CALC, 0.01f);
	pid_set_parameters(&_speed_ctrl,
			   _param_speed_p.get(),	// GND_SPEED_P
			   _param_speed_i.get(),
			   _param_speed_d.get(),
			   _param_speed_imax.get(),
			   _param_speed_max.get());

	pid_reset_integral(&_speed_ctrl);

	// Velocity setpoint smoothing parameters:
	_forwards_velocity_smoothing.setMaxJerk(_param_rdd_max_jerk.get());	// RDD_MAX_JERK
	_forwards_velocity_smoothing.setMaxAccel(_param_rdd_max_accel.get());	// RDD_MAX_ACCEL
	_forwards_velocity_smoothing.setMaxVel(_rdd_max_speed);

	// Turn rate control parameters (z-axis only):
	_rate_control.setPidGains(matrix::Vector3f(0.0f, 0.0f, _param_rate_p.get()),	// GND_RATE_P
				  matrix::Vector3f(0.0f, 0.0f, _param_rate_i.get()),
				  matrix::Vector3f(0.0f, 0.0f, _param_rate_d.get()));
	_rate_control.setFeedForwardGain(
		matrix::Vector3f(0.0f, 0.0f, 0.0f));	// will be set to GND_RATE_FF when in L1_GOTO_WAYPOINT
	_rate_control.setIntegratorLimit(matrix::Vector3f(0.0f, 0.0f, _param_rate_imax.get()));

	_rate_control.resetIntegral();

	// Set up measurements smoothing:
	int ema_period = _param_measurements_ema_period.get();	// GND_EMA_M_PERIOD
	_velocity_measured_ema.init(ema_period);

	// Set up velocity setpoint smoothing:
	ema_period = _param_speed_sp_ema_period.get();	// GND_SPEED_SP_EMA
	_velocity_setpoint_ema.init(ema_period);

	// Set up outputs smoothing:
	ema_period = _param_outputs_ema_period.get();	// GND_EMA_O_PERIOD
	_mission_torque_ema.init(ema_period);
	_mission_thrust_ema.init(ema_period);
}

void
RoverPositionControl::vehicle_control_mode_poll()
{
	if (_control_mode_sub.updated()) {
		_control_mode_sub.copy(&_control_mode);
#ifdef DEBUG_MY_PRINT

		if (_tracing_lev > 4) {
//			PX4_INFO("Control mode change detected");
//			print_message(ORB_ID(vehicle_control_mode), _control_mode);
			/*
			 * print produces:
			 vehicle_control_mode
				timestamp: 196044000 (0.000000 seconds ago)
				flag_armed: True
				flag_multicopter_position_control_enabled: False
				flag_control_manual_enabled: False
				flag_control_auto_enabled: True
				flag_control_offboard_enabled: False
				flag_control_rates_enabled: True
				flag_control_attitude_enabled: True
				flag_control_acceleration_enabled: False
				flag_control_velocity_enabled: True
				flag_control_position_enabled: True
				flag_control_altitude_enabled: True
				flag_control_climb_rate_enabled: True
				flag_control_termination_enabled: False
			 */
		}

#endif // DEBUG_MY_PRINT
	}
}

void
RoverPositionControl::manual_control_setpoint_poll()
{
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

	if (_control_mode.flag_control_manual_enabled && _manual_control_setpoint_sub.copy(&_manual_control_setpoint)) {
		// we come here only in Manual mode
#ifdef DEBUG_MY_PRINT
		if (_tracing_lev > 4) {
			//PX4_INFO("Manual setpoint change detected:  Y (roll): %f     Z (throttle): %f", (double)_manual_control_setpoint.roll, (double)_manual_control_setpoint.throttle);
			//print_message(ORB_ID(manual_control_setpoint), _manual_control_setpoint);
		}

#endif // DEBUG_MY_PRINT

		// Set heading torque from the manual roll input channel. See GND_MAN_YAW_SC - for smoother manual control.
		_torque_control_manual = _manual_control_setpoint.roll *
					 _param_manual_yaw_scaler.get();		// Nominally yaw: _manual_control_setpoint.roll;  - this is right stick, horizontal movement, R/C CH1 "ailerons"
		// Set thrust from the manual throttle channel
		_thrust_control_manual =
			_manual_control_setpoint.throttle;	// this is right stick, vertical movement, R/C CH2 "elevator"

		// Here we ignore left stick movement - x (vertical) and r (horizontal) - those are mapped to Flight Mode and Arming functions. Could be useful in the future.

		// When channels are not under R/C Passthrough (like "param set PCA9685_FUNC3 406") - we collect R/C controls positions here, to pass them to servos in Manual mode:

		_cutter_setpoint_manual =
			_manual_control_setpoint.flaps;	// R/C CH7 "leftmost top switch" - param set RC_MAP_FLAPS 7 - cutter clutch
		_gas_throttle_manual =
			_manual_control_setpoint.aux1;		// R/C CH5 "left round knob"     - param set RC_MAP_AUX1 5  - gas engine throttle
		_alarm_dev_level_manual =
			_manual_control_setpoint.aux2;	// R/C CH6 "right round knob"    - param set RC_MAP_AUX2 6  - spare servo

		_manual_setpoint_last_called = _now = hrt_absolute_time();
	}
}

void
RoverPositionControl::position_setpoint_triplet_poll()
{
	if (_pos_sp_triplet_sub.updated()) {
		_pos_sp_triplet_sub.copy(&_pos_sp_triplet);

		if (_pos_sp_triplet.current.valid || _pos_sp_triplet.previous.valid || _pos_sp_triplet.next.valid) {
#ifdef DEBUG_MY_PRINT

			if (_tracing_lev > 4) {
				PX4_INFO("Position setpoint triplet change detected  Valid:  curr: %s   prev: %s  next: %s",
					 _pos_sp_triplet.current.valid ? "yes" : "no",
					 _pos_sp_triplet.previous.valid ? "yes" : "no",
					 _pos_sp_triplet.next.valid ? "yes" : "no"
					);
				print_message(ORB_ID(position_setpoint_triplet), _pos_sp_triplet);
				/*
				 * print produces (three of these):
				 *
							  current (position_setpoint):
								timestamp: 34820000 (0.004000 seconds ago)
								lat: 33.2203
								lon: -86.3088
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

				debugPrintAll();
			}

#endif // DEBUG_MY_PRINT

			//setStateMachineState(POS_STATE_IDLE);
		}
	}
}

void
RoverPositionControl::gps_poll()
{
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
				lat: 332200009				# Latitude in 1E-7 degrees
				lon: -863088035				# Longitude in 1E-7 degrees
				alt: 147045					# Altitude in 1E-3 meters above MSL, (millimetres)
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
}

void
RoverPositionControl::magnetometer_poll()
{
	if (_magnetometer_sub.update(&_magnetometer)) {
		bool reset = false;

		// check if magnetometer has changed
		if (_magnetometer.device_id != _device_id_mag) {
			if (_device_id_mag != 0) {
				PX4_WARN("mag sensor ID changed %" PRIu32 " -> %" PRIu32, _device_id_mag, _magnetometer.device_id);
			}

			reset = true;

		} else if (_magnetometer.calibration_count != _mag_calibration_count) {
			// existing calibration has changed, reset saved mag bias
			PX4_DEBUG("mag %" PRIu32 " calibration updated, resetting bias", _device_id_mag);
			reset = true;
		}

		if (reset) {
			_device_id_mag = _magnetometer.device_id;
			_mag_calibration_count = _magnetometer.calibration_count;
		}

		//  magnetometer_ga - Magnetic field in the FRD body frame XYZ-axis in Gauss

		_mag_reading3d = Vector3f{_magnetometer.magnetometer_ga};

		const matrix::Vector2f mr2d(_mag_reading3d);

		_mag_reading2d = mr2d;

		_mag_current_heading = wrap_pi(-atan2f(_mag_reading2d(1), _mag_reading2d(0)));
	}
}

void
RoverPositionControl::attitude_setpoint_poll()
{
	if (_att_sp_sub.updated()) {
		// we come here in autonomous flight mode
		//PX4_INFO("Attitude setpoint change detected");
		_att_sp_sub.copy(&_att_sp);
	}
}

void
RoverPositionControl::vehicle_attitude_poll()
{
	if (_att_sub.updated()) {
		// we come here driven by sensors
		//PX4_INFO("Attitude change detected");
		_att_sub.copy(&_vehicle_att);
	}
}

void
RoverPositionControl::rates_setpoint_poll()
{
	// only mavlink would publish this, no need to process here yet
	if (_vehicle_rates_setpoint_sub.updated()) {
		_vehicle_rates_setpoint_sub.copy(&_rates_setpoint);
//#ifdef DEBUG_MY_PRINT
		//if(_tracing_lev > 4)
		{
			PX4_INFO("Rates Setpoint change detected");
			print_message(ORB_ID(vehicle_rates_setpoint), _rates_setpoint);
			/*
			 * print produces:
			 */
		}
//#endif // DEBUG_MY_PRINT
	}
}

void
RoverPositionControl::mission_poll()
{
	if (_mission_sub.updated()) {
		_mission_sub.copy(&_mission);
#ifdef DEBUG_MY_PRINT

		if (_tracing_lev > 4) {
			PX4_INFO("Mission change detected");
			print_message(ORB_ID(mission), _mission);
			/*
			 * print produces:
			 */
		}

#endif // DEBUG_MY_PRINT
	}
}

void
RoverPositionControl::mission_result_poll()
{
	if (_mission_result_sub.updated()) {
		_mission_result_sub.copy(&_mission_result);
#ifdef DEBUG_MY_PRINT

		if (_tracing_lev > 4) {
			PX4_INFO("Mission Result change detected");
			print_message(ORB_ID(mission_result), _mission_result);
			/*
			 * print produces:
			INFO  [rover_pos_control] Mission Result change detected
			 mission_result
				timestamp: 6388000 (0.004000 seconds ago)
				instance_count: 1
				seq_reached: -1
				seq_current: 0
				seq_total: 4
				item_changed_index: 0
				item_do_jump_remaining: 0
				valid: True
				warning: False
				finished: False
				failure: False
				stay_in_failsafe: False
				flight_termination: False
				item_do_jump_changed: False
				execution_mode: 0

			...

			INFO  [rover_pos_control] Vehicle Status change detected
			...

			INFO  [commander] Armed by external command
			INFO  [navigator] Executing Mission
			INFO  [logger] Start file log (type: full)
			INFO  [logger] [logger] ./log/2022-01-18/15_22_52.ulg
			INFO  [logger] Opened full log file: ./log/2022-01-18/15_22_52.ulg
			INFO  [rover_pos_control] Mission Result change detected
			 mission_result
				timestamp: 20184000 (0.004000 seconds ago)
				instance_count: 1
				seq_reached: -1
				seq_current: 3
				seq_total: 4
				item_changed_index: 0
				item_do_jump_remaining: 0
				valid: True
				warning: False
				finished: False
				failure: False
				stay_in_failsafe: False
				flight_termination: False
				item_do_jump_changed: False
				execution_mode: 0
			INFO  [rover_pos_control] Vehicle Status change detected
			...

			INFO  [navigator] Mission finished, loitering
			INFO  [rover_pos_control] Mission Result change detected
			 mission_result
				timestamp: 34256000 (0.004000 seconds ago)
				instance_count: 1
				seq_reached: 3
				seq_current: 3
				seq_total: 4
				item_changed_index: 0
				item_do_jump_remaining: 0
				valid: True
				warning: False
				finished: True
				failure: False
				stay_in_failsafe: False
				flight_termination: False
				item_do_jump_changed: False
				execution_mode: 0


			 */

			debugPrintAll();
		}

#endif // DEBUG_MY_PRINT
	}
}

void
RoverPositionControl::navigator_mission_item_poll()
{
	if (_navigator_mission_item_sub.updated()) {
		_navigator_mission_item_sub.copy(&_navigator_mission_item);
#ifdef DEBUG_MY_PRINT

		if (_tracing_lev > 4) {
			PX4_INFO("Navigator Mission Item change detected");
			print_message(ORB_ID(navigator_mission_item), _navigator_mission_item);
			/*
			 * print produces:
			 *
			INFO  [rover_pos_control] Navigator Mission Item change detected
			 navigator_mission_item
				timestamp: 43820000 (0.004000 seconds ago)
				instance_count: 1
				latitude: 33.2203
				longitude: -86.3088
				time_inside: 0.0000
				acceptance_radius: 0.5000
				loiter_radius: 0.3000
				yaw: nan
				altitude: 145.4980
				sequence_current: 4
				nav_cmd: 17
				frame: 1
				origin: 0
				loiter_exit_xtrack: False
				force_heading: False
				altitude_is_relative: False
				autocontinue: False
				vtol_back_transition: False

			nav_cmd:

					enum NAV_CMD {
						NAV_CMD_IDLE = 0,
						NAV_CMD_WAYPOINT = 16,
						NAV_CMD_LOITER_UNLIMITED = 17,
						NAV_CMD_LOITER_TIME_LIMIT = 19,
						NAV_CMD_RETURN_TO_LAUNCH = 20,
						NAV_CMD_LAND = 21,
						NAV_CMD_TAKEOFF = 22,
						NAV_CMD_LOITER_TO_ALT = 31,
						NAV_CMD_DO_FOLLOW_REPOSITION = 33,
						NAV_CMD_VTOL_TAKEOFF = 84,
						NAV_CMD_VTOL_LAND = 85,
						NAV_CMD_DELAY = 93,
						NAV_CMD_DO_JUMP = 177,
						NAV_CMD_DO_CHANGE_SPEED = 178,
						NAV_CMD_DO_SET_HOME = 179,
						NAV_CMD_DO_SET_SERVO = 183,
						NAV_CMD_DO_LAND_START = 189,
						NAV_CMD_DO_SET_ROI_LOCATION = 195,
						NAV_CMD_DO_SET_ROI_WPNEXT_OFFSET = 196,
						NAV_CMD_DO_SET_ROI_NONE = 197,
						NAV_CMD_DO_CONTROL_VIDEO = 200,
						NAV_CMD_DO_SET_ROI = 201,
						NAV_CMD_DO_DIGICAM_CONTROL = 203,
						NAV_CMD_DO_MOUNT_CONFIGURE = 204,
						NAV_CMD_DO_MOUNT_CONTROL = 205,
						NAV_CMD_DO_SET_CAM_TRIGG_INTERVAL = 214,
						NAV_CMD_DO_SET_CAM_TRIGG_DIST = 206,
						NAV_CMD_OBLIQUE_SURVEY = 260,
						NAV_CMD_SET_CAMERA_MODE = 530,
						NAV_CMD_SET_CAMERA_ZOOM = 531,
						NAV_CMD_SET_CAMERA_FOCUS = 532,
						NAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW = 1000,
						NAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE = 1001,
						NAV_CMD_IMAGE_START_CAPTURE = 2000,
						NAV_CMD_IMAGE_STOP_CAPTURE = 2001,
						NAV_CMD_DO_TRIGGER_CONTROL = 2003,
						NAV_CMD_VIDEO_START_CAPTURE = 2500,
						NAV_CMD_VIDEO_STOP_CAPTURE = 2501,
						NAV_CMD_DO_VTOL_TRANSITION = 3000,
						NAV_CMD_FENCE_RETURN_POINT = 5000,
						NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION = 5001,
						NAV_CMD_FENCE_POLYGON_VERTEX_EXCLUSION = 5002,
						NAV_CMD_FENCE_CIRCLE_INCLUSION = 5003,
						NAV_CMD_FENCE_CIRCLE_EXCLUSION = 5004,
						NAV_CMD_CONDITION_GATE = 4501,
						NAV_CMD_INVALID = UINT16_MAX / * ensure that casting a large number results in a specific error * /
					};

			 */
		}

#endif // DEBUG_MY_PRINT
	}
}

void
RoverPositionControl::vehicle_status_poll()
{
	// published at 1Hz

	if (_vehicle_status_sub.updated()) {
		_vehicle_status_sub.copy(&_vehicle_status);
#ifdef DEBUG_MY_PRINT

		if (_tracing_lev > 4) {
			//PX4_INFO("Vehicle Status change detected");
			//print_message(ORB_ID(vehicle_status), _vehicle_status);
			/*
			 *
				NAVIGATION_STATE_MANUAL = 0;
				NAVIGATION_STATE_ALTCTL = 1;
				NAVIGATION_STATE_POSCTL = 2;
				NAVIGATION_STATE_AUTO_MISSION = 3;
				NAVIGATION_STATE_AUTO_LOITER = 4;
				NAVIGATION_STATE_AUTO_RTL = 5;
				NAVIGATION_STATE_UNUSED3 = 8;
				NAVIGATION_STATE_UNUSED = 9;
				NAVIGATION_STATE_ACRO = 10;
				NAVIGATION_STATE_UNUSED1 = 11;
				NAVIGATION_STATE_DESCEND = 12;
				NAVIGATION_STATE_TERMINATION = 13;
				NAVIGATION_STATE_OFFBOARD = 14;
				NAVIGATION_STATE_STAB = 15;
				NAVIGATION_STATE_UNUSED2 = 16;
				NAVIGATION_STATE_AUTO_TAKEOFF = 17;
				NAVIGATION_STATE_AUTO_LAND = 18;
				NAVIGATION_STATE_AUTO_FOLLOW_TARGET = 19;
				NAVIGATION_STATE_AUTO_PRECLAND = 20;
				NAVIGATION_STATE_ORBIT = 21;
				NAVIGATION_STATE_AUTO_VTOL_TAKEOFF = 22;
				NAVIGATION_STATE_MAX = 23;
			 *
			 * print produces:
			 *
			 */
		}

#endif // DEBUG_MY_PRINT
	}
}

void
RoverPositionControl::actuator_outputs_poll()
{
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
	}
}
