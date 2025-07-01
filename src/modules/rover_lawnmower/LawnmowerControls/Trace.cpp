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

#ifdef DEBUG_MY_PRINT

void LawnmowerControl::debugPrint()
{
	_tracing_lev = _param_lm_tracing_lev.get();

	if (_tracing_lev < 1) {
		return; // No tracing
	}

	//PX4_INFO_RAW("_tracing_lev: %d\n", _tracing_lev);


	if (hrt_elapsed_time(&_debug_print_last_called) > 1_s) {

		/*
		if (!_ekf_data_good) {
			PX4_WARN("Bad EKF: xy_valid: %s  v_xy_valid: %s  hdg_good_for_control: %s",
				 _ekf_flags(0) ? "true" : "false",
				 _ekf_flags(1) ? "true" : "false",
				 _ekf_flags(2) ? "true" : "false"
				);
		}
		*/

		if (_vehicle_control_mode.flag_control_manual_enabled) {

			if (_vehicle_control_mode.flag_armed) {
				debugPrintManual();

				//} else {
				//	PX4_WARN("Manual control enabled, but vehicle is not armed");
			}

		} else if (_vehicle_control_mode.flag_control_auto_enabled) {

			if (_pos_ctrl_state != POS_STATE_NONE || !_printed_none_trace) {

				debugPrintAuto();

				debugPrintCrosstrackStats();
			}

		} else if (_vehicle_control_mode.flag_control_offboard_enabled) {

			PX4_WARN("Offboard control enabled, but not implemented yet");

		} else {

			PX4_WARN("Vehicle control mode not set");
		}

		_printed_none_trace = _pos_ctrl_state == POS_STATE_NONE;

		_debug_print_last_called = _timestamp;
	}
}

void LawnmowerControl::debugPrintAuto()
{
	if (_tracing_lev > 0) {

		PX4_INFO_RAW("=== %s ============  dt: %.3f ms\n",
			     control_state_name(_pos_ctrl_state), (double)(_dt * 1000.0f));

		PX4_INFO_RAW("--- EKF off: %.1f cm %.0f deg   --- YAW: EKF: %.1f GPS: %.1f  cog: %.1f deg\n",
			     (double)(_location_metrics.ekfGpsDeviation(0) * 100.0f),
			     (double)math::degrees(_location_metrics.ekfGpsDeviation(1)),
			     (double)math::degrees(_vehicle_yaw), (double)math::degrees(_sensor_gps_data.heading),
			     (double)math::degrees(_sensor_gps_data.cog_rad));

		PX4_INFO_RAW("--- WP: curr_dist: %.2f   prev_dist: %.2f   next_dist: %.2f\n",
			     (double)_wp_current_dist, (double)_wp_previous_dist, (double)_wp_next_dist);

		PX4_INFO_RAW("--- Speed: setpoint: %.2f    actual: %.2f    gps_vel: %.2f\n",
			     (double)_rover_speed_setpoint, (double)_location_metrics.ekf_x_vel, (double)_location_metrics.gps_vel_m_s);

		PX4_INFO_RAW("--- AN: bearing_to_curr_wp: %.1f deg   yaw_error: %.2f deg   abbe_error: %.2f m\n",
			     (double)math::degrees(_bearing_to_curr_wp), (double)math::degrees(_yaw_error), (double)_abbe_error);

		PX4_INFO_RAW("--- PP: crosstrack error: %.1f cm   bearing error: %.3f degrees   distance_to_waypoint: %.2f m\n",
			     (double)(_pure_pursuit_status.crosstrack_error * 100.0f),
			     (double)math::degrees(_bearing_error),
			     (double)(_pure_pursuit_status.distance_to_waypoint));

		PX4_INFO_RAW("--- Setpoints: engine: %.4f    cutter: %.4f    alarm: %.4f\n",
			     (double)_ice_throttle_setpoint, (double)_cutter_setpoint, (double)_alarm_dev_level);

//#ifdef __x86_64
#if !defined(CONFIG_ARCH_BOARD_PX4_SITL)
		PX4_INFO_RAW("--- Servos: whls L: %d R: %d   gas thrtle: %d   blades: %d   alarm: %d\n",
			     (int)_wheel_left_servo_position, (int)_wheel_right_servo_position,
			     (int)_ice_throttle_servo_position, (int)_cutter_servo_position, (int)_alarm_servo_position);
#endif // CONFIG_ARCH_BOARD_PX4_SITL
	}
}

void LawnmowerControl::debugPrintCrosstrackStats()
{
	if (PX4_ISFINITE(_crosstrack_error_avg)) {

		PX4_INFO_RAW("+++ Last leg crosstrack error:  avg: %.1f cm  max: %.1f cm\n",
			     (double)(_crosstrack_error_avg * 100.0f),
			     (double)(_crosstrack_error_max * 100.0f));
	}

	if (PX4_ISFINITE(_crosstrack_error_mission_avg)) {

		PX4_INFO_RAW("+++ Mission crosstrack error:   avg: %.1f cm  max: %.1f cm   seconds outside: %i\n",
			     (double)(_crosstrack_error_mission_avg * 100.0f),
			     (double)(_crosstrack_error_mission_max * 100.0f),
			     _cte_seconds_outside);
	}
}

void LawnmowerControl::debugPrintArriveDepart()
{
	if (hrt_elapsed_time(&_debug_print_ad_last_called) > 100_ms) {

		if (_tracing_lev > 4) {

			debugPrintAuto();
		}

		_debug_print_ad_last_called = _timestamp;
	}
}

void LawnmowerControl::debugPrintManual()
{
	PX4_INFO_RAW("=== Manual CONTROL: %s ============  dt: %.3f ms EKF off: %.1f cm %.0f deg  yaw: EKF: %.1f GPS: %.1f/%.1f deg\n",
		     control_state_name(_pos_ctrl_state), (double)(_dt * 1000.0f),
		     (double)(_location_metrics.ekfGpsDeviation(0) * 100.0f),
		     (double)math::degrees(_location_metrics.ekfGpsDeviation(1)),
		     (double)math::degrees(_vehicle_yaw), (double)math::degrees(_sensor_gps_data.heading),
		     (double)math::degrees(_sensor_gps_data.cog_rad));

	if (_tracing_lev > 0) {

		PX4_INFO_RAW("R/C: roll: %.2f  pitch: %.2f  yaw: %.2f  throttle: %.2f   flaps: %.2f  aux1: %.2f  aux2: %.2f\n",
			     (double)_manual_control_setpoint.roll,
			     (double)_manual_control_setpoint.pitch,
			     (double)_manual_control_setpoint.yaw,
			     (double)_manual_control_setpoint.throttle,
			     (double)_manual_control_setpoint.flaps,
			     (double)_manual_control_setpoint.aux1,
			     (double)_manual_control_setpoint.aux2);

		PX4_INFO_RAW("--- setpoints: yaw: %4f    thrust: %4f    engine: %.4f    cutter: %.4f    alarm: %.4f\n",
			     (double)_torque_control_manual, (double)_thrust_control_manual,
			     (double)_gas_throttle_manual,
			     (double)_cutter_setpoint_manual, (double)_alarm_dev_level_manual);

		PX4_INFO_RAW("--- servos: whls L: %d R: %d   gas thrtle: %d   blades: %d   alarm: %d\n",
			     (int)_wheel_left_servo_position, (int)_wheel_right_servo_position,
			     (int)_ice_throttle_servo_position, (int)_cutter_servo_position, (int)_alarm_servo_position);
	}
}

const char *LawnmowerControl::control_state_name(const POS_CTRLSTATES state)
{
	switch (state) {
	case POS_STATE_NONE:				// undefined/invalid state, no need controlling anything
		return "POS_STATE_NONE";

	case POS_STATE_IDLE:				// idle state, no need controlling anything
		return "POS_STATE_IDLE";

	case STRAIGHT_RUN:				// target waypoint is far away, we can use Pursuit and cruise speed
		return "STRAIGHT_RUN";

	case WP_ARRIVING:				// target waypoint is close, we need to slow down and head straight to it till stop
		return "WP_ARRIVING";

	case WP_ARRIVED:				// reached waypoint, completely stopped. Make sure mission knows about it
		return "WP_ARRIVED";

	case WP_TURNING:				// we need to turn in place to the next waypoint
		return "WP_TURNING";

	case WP_DEPARTING:				// we turned to next waypoint and must start accelerating
		return "WP_DEPARTING";

	case POS_STATE_STOPPING:			// we hit a waypoint and need to stop
		return "POS_STATE_STOPPING";

	case POS_STATE_MISSION_START:			// turn on what we need for the mission (lights, gas engine throttle, blades)
		return "POS_STATE_MISSION_START";

	case POS_STATE_MISSION_END:			// turn off what we needed for the mission at the end or error
		return "POS_STATE_MISSION_END";

	default:
		return "???control_state_name()???";
	}
}

#endif // DEBUG_MY_PRINT

#ifdef DEBUG_MY_DATA

// see .../PX4-Autopilot/src/examples/px4_mavlink_debug/px4_mavlink_debug.cpp

void LawnmowerControl::publishDebugData()
{
	//PX4_INFO_RAW("publishDebugData()\n");


	/*
		* Default MAVLINK forwarding rate is 1 HZ.
		* To change that (to 100Hz) see .../PX4-Autopilot/src/modules/mavlink/mavlink_main.cpp:1528,1589
		* We just need DEBUG_FLOAT_ARRAY at 100Hz in several places there:
		*
		configure_stream_local("DEBUG", 1.0f);
		configure_stream_local("DEBUG_FLOAT_ARRAY", 100.0f);    <--
		configure_stream_local("DEBUG_VECT", 1.0f);
		configure_stream_local("NAMED_VALUE_FLOAT", 1.0f);
		configure_stream_local("LINK_NODE_STATUS", 1.0f);
		*
		* Make sure you have parameters set:
		*
		# Logging to SD card:
		param set-default SDLOG_MODE 0
		#param set-default SDLOG_PROFILE 255   <-- no need to allow all. 123 is sufficient.
		param set-default SDLOG_PROFILE 123

		# Bitmask SDLOG_PROFILE:
		# 0: Default set (general log analysis)
		# 1: Estimator replay (EKF2)
		# 2: Thermal calibration
		# 3: System identification
		# 4: High rate
		# 5: Debug
		# 6: Sensor comparison
		# 7: Computer Vision and Avoidance
		# 8: Raw FIFO high-rate IMU (Gyro)
		# 9: Raw FIFO high-rate IMU (Accel)
		# 10: Mavlink tunnel message logging

		# 123 = 0b01111011
		# 131 = 0b10000011

		*
		*/

	_debug_data_last_called = _timestamp;

	// send one array:

	/*
	// test debug_array with generated curves:
	for (size_t i = 0; i < debug_array_s::ARRAY_SIZE; i++) {
		_dbg_array.data[i] = i * 0.01 * sin(ms);
	*/

	// Clear the array
	memset(_dbg_array.data, 0, sizeof(_dbg_array.data));

	int i = 1;	// data[0] is reserved for total number of parameters, max 58

	_dbg_array.data[i++] = (float)_pos_ctrl_state;
	_dbg_array.data[i++] = _dt;	// seconds, time since last update

	_dbg_array.data[i++] = _location_metrics.gps_data_valid;	// 0 or 1, GPS data valid
	_dbg_array.data[i++] =
		_location_metrics.fix_type;		// 0: no GPS, 1: no fix, 2: 2D fix, 3: 3D fix, 4: 3D DGPS, 5: RTK float, 6: RTK fixed
	_dbg_array.data[i++] = _location_metrics.gps_vel_m_s;		// meters per second, GPS velocity magnitude
	_dbg_array.data[i++] = math::degrees(_location_metrics.gps_cog_rad);	// degrees, GPS course over ground -PI.. PI
	_dbg_array.data[i++] = math::degrees(_location_metrics.gps_yaw); 	// degrees, RTK dual antenna GPS "heading" -PI.. PI

	_dbg_array.data[i++] = _location_metrics.ekf_data_good ? 1.0f : 0.0f;	// 1.0 if EKF data is good, 0.0 if not
	_dbg_array.data[i++] = _location_metrics.ekf_ground_speed_abs;	// meters per second, EKF ground speed magnitude
	_dbg_array.data[i++] = _location_metrics.ekf_x_vel;		// meters per second, EKF velocity along X axis
	_dbg_array.data[i++] = math::degrees(_location_metrics.ekf_current_yaw);	// degrees, EKF current yaw angle -PI.. PI

	_dbg_array.data[i++] = _location_metrics.ekfGpsDeviation(0);	// meters, EKF GPS deviation from the last GPS reading
	_dbg_array.data[i++] = math::degrees(_location_metrics.ekfGpsDeviation(
			1));	// degrees, bearing from EKF to the last GPS reading

	_dbg_array.data[i++] = _wp_current_dist;	// meters, distance to the current waypoint
	_dbg_array.data[i++] = _wp_previous_dist;	// meters, distance to the previous waypoint
	_dbg_array.data[i++] = _wp_next_dist;		// meters, distance to the next waypoint

	_dbg_array.data[i++] = _cutter_setpoint;	// -1..1, cutter setpoint, published to PCA9685 channel 3
	_dbg_array.data[i++] = _ice_throttle_setpoint;	// 0..1, gas engine throttle setpoint, published to PCA9685 channel 4
	_dbg_array.data[i++] = _alarm_dev_level;	// 0..1, alarm device level setpoint, published to PCA9685 channel 6

#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
	// No servos to monitor in emulation, keep placeholders:
	_dbg_array.data[i++] = NAN;
	_dbg_array.data[i++] = NAN;
	_dbg_array.data[i++] = NAN;
	_dbg_array.data[i++] = NAN;
	_dbg_array.data[i++] = NAN;
# else
	// Actuators actual positions, as polled from actuator_outputs:
	_dbg_array.data[i++] = _wheel_left_servo_position;	// 0..1, left wheel servo position, polled from PCA9685 channel 1
	_dbg_array.data[i++] = _wheel_right_servo_position;	// 0..1, right wheel servo position, polled from PCA9685 channel 2
	_dbg_array.data[i++] = _cutter_servo_position;		// 0..1, cutter servo position, polled from PCA9685 channel 3
	_dbg_array.data[i++] =
		_ice_throttle_servo_position;	// 0..1, gas engine throttle servo position, polled from PCA9685 channel 4
	_dbg_array.data[i++] = _alarm_servo_position;		// 0..1, alarm servo position, polled from PCA9685 channel 6
#endif // defined(CONFIG_ARCH_BOARD_PX4_SITL)

	_dbg_array.data[i++] = _crosstrack_error_avg;	// meters, average cross track error for the straight run ("leg")
	_dbg_array.data[i++] = _crosstrack_error_max;	// meters, maximum cross track error for the straight run
	_dbg_array.data[i++] = _crosstrack_error_mission_avg;	// meters, average cross track error for the whole mission
	_dbg_array.data[i++] = _crosstrack_error_mission_max;	// meters, maximum cross track error for the whole mission
	_dbg_array.data[i++] = _cte_seconds_outside;	// time in seconds the cross track error was outside the mission limits


	// calculated by Pursuit controller, and are present as members of _gnd_control:
	_dbg_array.data[i++] = math::degrees(_vehicle_yaw);
	_dbg_array.data[i++] = _crosstrack_error;		// meters
	_dbg_array.data[i++] = math::degrees(_bearing_error);	// degrees, bearing error from the target waypoint, 0..360 degrees
	_dbg_array.data[i++] = _pure_pursuit_status.distance_to_waypoint;	// meters, distance to the target waypoint
	_dbg_array.data[i++] =
		_pure_pursuit_status.lookahead_distance;	// meters, lookahead distance of the pure pursuit controller

	_dbg_array.data[i++] = math::degrees(
				       _bearing_to_curr_wp); // radians, bearing to the current waypoint, calculated by updateBearings()
	_dbg_array.data[i++] = math::degrees(
				       _yaw_error);	// radians, yaw error to the current waypoint, positive - right turn, negative - left turn expected
	_dbg_array.data[i++] = _abbe_error;		//  meters, heading error at the target point

	// TODO: more data here, polled or calculated by LawnmowerControl

	_dbg_array.data[0] = i;	// must be less than 58, per size of the data[]

	publishDebugArray();
}

#endif // DEBUG_MY_DATA


} // namespace rover_lawnmower

