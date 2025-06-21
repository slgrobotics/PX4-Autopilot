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

			} else {
				PX4_WARN("Manual control enabled, but vehicle is not armed");
			}

		} else if (_vehicle_control_mode.flag_control_auto_enabled) {
			/*
			if (_pos_ctrl_state != POS_STATE_IDLE || !_printed_idle_trace) {

			if (PX4_ISFINITE(_ekfGpsDeviation) && _ekfGpsDeviation > 0.2f) {
				PX4_WARN("EKF2 deviation: %.1f cm", (double)(_ekfGpsDeviation * 100.0f));
			}

			if (PX4_ISFINITE(_crosstrack_error) && _crosstrack_error > 0.2f) {
				PX4_WARN("crosstrack error: %.1f cm", (double)(_crosstrack_error * 100.0f));
			}
			*/

			debugPrintAuto();

		} else if (_vehicle_control_mode.flag_control_offboard_enabled) {

			PX4_WARN("Offboard control enabled, but not implemented yet");

		} else {

			PX4_WARN("Vehicle control mode not set");
		}

		//_printed_idle_trace = _pos_ctrl_state == POS_STATE_IDLE;

		_debug_print_last_called = _timestamp;
	}
}

void LawnmowerControl::debugPrintAuto()
{
	PX4_INFO_RAW("=== AUTO CONTROL\n");

	PX4_INFO_RAW("_vehicle_yaw: %f\n", (double)math::degrees(_vehicle_yaw));

	/*
	if (_tracing_lev > 0) {
		PX4_INFO_RAW("=== %s ===================    dt: %.3f ms EKF off: %.1f cm\n", control_state_name(_pos_ctrl_state),
			     (double)(_dt * 1000.0f), (double)(_ekfGpsDeviation * 100.0f));
		//print_run_status();   // scheduler calling rate

		if (PX4_ISFINITE(_crosstrack_error_avg)) {
			PX4_INFO_RAW("---    last leg crosstrack error:  avg: %.1f cm  max: %.1f cm\n",
				     (double)(_crosstrack_error_avg * 100.0f),
				     (double)(_crosstrack_error_max * 100.0f));
			PX4_INFO_RAW("---    mission crosstrack error:   avg: %.1f cm  max: %.1f cm  outside: %i\n",
				     (double)(_crosstrack_error_mission_avg * 100.0f),
				     (double)(_crosstrack_error_mission_max * 100.0f),
				     _cte_count_outside);
		}

		if (_tracing_lev < 4) {
			PX4_INFO_RAW("---    trgt_berng: %.2f  curr_hdg: %.2f  hdg_error: %.2f\n",
				     (double) math::degrees(_target_bearing), (double) math::degrees(_current_heading),
				     (double) math::degrees(_heading_error));

			if (_pos_ctrl_state == L1_GOTO_WAYPOINT) {
				PX4_INFO_RAW("---    XTrack err: %.1f cm\n",
					     (double)(_crosstrack_error * 100.0f));
			}
		}
	}

	if (_tracing_lev > 3) {
		PX4_INFO_RAW("---    dist_trgt: %.2f   leg: %.2f   trgt_berng: %.2f  nav_berng: %.2f\n",
			     (double) _dist_target, (double)_leg_distance, (double) math::degrees(_target_bearing),
			     (double) math::degrees(_nav_bearing));

		if (_pos_ctrl_state == L1_GOTO_WAYPOINT) {
			PX4_INFO_RAW("---    XTrack err: %.1f cm\n",
				     (double)(_crosstrack_error * 100.0f));
		}

		PX4_INFO_RAW("---    hdg_er: %.4f   abbe: %.2f m   gas: %.2f tool: %.2f alrm: %.1f\n",
			     (double)math::degrees(_heading_error), (double)_abbe_error,
			     (double)_gas_engine_throttle, (double)_cutter_setpoint, (double)_alarm_dev_level);
	}

	if (_tracing_lev > 1) {
		PX4_INFO_RAW("---    servos: whls L: %d R: %d   gas thrtle: %d   blades: %d   alarm: %d\n",
			     (int)_wheel_left_servo_position, (int)_wheel_right_servo_position,
			     (int)_gas_throttle_servo_position, (int)_cutter_servo_position, (int)_alarm_servo_position);
	}

	if (_tracing_lev > 2) {
		PX4_INFO_RAW("---    spd abs: %.2f  x_vel: %.4f  yaw_rate: %.1f\n",
			     (double)_ground_speed_abs, (double)_x_vel,
			     (double)math::degrees(_z_yaw_rate));

		PX4_INFO_RAW("---    wp_curr_dist: %.2f   wp_prev_dist: %.2f   wp_next_dist: %.2f\n",
			     (double)_wp_current_dist, (double)_wp_previous_dist, (double)_wp_next_dist);
	}

	if (_tracing_lev > 1) {
		PX4_INFO_RAW("---    misn_vel_sp: %.4f   misn_thrust_eff: %.4f   nav_state: %d\n",
			     (double)_mission_velocity_setpoint, (double)_mission_thrust_effort,
			     (int)_nav_state);

		PX4_INFO_RAW("---    misn_turn_sp: %.4f  misn_torq_effrt: %.4f  cnt_run: %d  cnt_calc: %d\n",
			     (double)_mission_yaw_rate_setpoint, (double)_mission_torque_effort, _cnt_run, _cnt_calc);
	}

	_cnt_run = 0;
	_cnt_calc = 0;
	*/
}

void LawnmowerControl::debugPrintManual()
{
	PX4_INFO_RAW("=== MANUAL CONTROL\n");
	/*
	if (_tracing_lev > 0) {
	PX4_INFO_RAW("=== MANUAL CONTROL %s ===== %s =====    dt: %.3f ms EKF off: %.1f cm\n",
		     _manual_using_pids ? "using PIDs" : "direct",
		     control_state_name(_pos_ctrl_state), (double)(_dt * 1000.0f), (double)(_ekfGpsDeviation * 100.0f));

	PX4_INFO_RAW("---Heading:  current: %.2f  gps: %.2f  ekf: %.2f  by vel: %.2f  mag: %.2f\n",
		     (double)math::degrees(_current_heading), (double)math::degrees(_gps_current_heading),
		     (double)math::degrees(_ekf_current_heading),
		     (double)math::degrees(wrap_pi(atan2f(_sensor_gps_data.vel_e_m_s, _sensor_gps_data.vel_n_m_s))),
		     (double)math::degrees(_mag_current_heading));

	PX4_INFO_RAW("---Speed: ground: %.2f   gps: %.2f   ekf: %.2f  x_vel: %.4f\n",
		     (double)_ground_speed_abs, (double)_gps_ground_speed_abs, (double)_ekf_ground_speed_abs, (double)_x_vel);

	if (_manual_using_pids) {
		PX4_INFO_RAW("---     misn_vel_sp: %.2f  misn_thrust_eff: %.2f  ground_speed_abs: %.4f\n",
			     (double)_mission_velocity_setpoint, (double)_mission_thrust_effort, (double)_ground_speed_abs);

		PX4_INFO_RAW("---     mission_torq_effort: %.2f   yaw_rate: %.1f\n",
			     (double)_mission_torque_effort, (double)math::degrees(_z_yaw_rate));
	}

	PX4_INFO_RAW("---act_controls: yaw: %4f    thrust: %4f    engine: %.4f    tool: %.4f    alarm: %.4f\n",
		     (double)_torque_control, (double)_thrust_control, (double)_gas_engine_throttle, (double)_cutter_setpoint,
		     (double)_alarm_dev_level);

	PX4_INFO_RAW("---servos: whls L: %d R: %d   gas thrtle: %d   blades: %d   2d tool: %d\n",
		     (int)_wheel_left_servo_position, (int)_wheel_right_servo_position,
		     (int)_gas_throttle_servo_position, (int)_cutter_servo_position, (int)_alarm_servo_position);
	}
	*/
}

#endif // DEBUG_MY_PRINT

#ifdef DEBUG_MY_DATA

// see .../PX4-Autopilot/src/examples/px4_mavlink_debug/px4_mavlink_debug.cpp

void LawnmowerControl::debugPublishData()
{
	//PX4_INFO_RAW("debugPublishData()\n");


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

	// calculated by Pursuit controller, and are present as members of _gnd_control:
	_dbg_array.data[i++] = math::degrees(_vehicle_yaw);

	// TODO: more data here, polled or calculated by LawnmowerControl

	_dbg_array.data[0] = i;	// must be less than 58, per size of the data[]

	publishDebugArray();
}

#endif // DEBUG_MY_DATA


} // namespace rover_lawnmower

