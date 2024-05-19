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

const char *RoverPositionControl::control_state_name(const POS_CTRLSTATES state)
{
	switch (state) {
	case POS_STATE_NONE:				// undefined/invalid state, no need controlling anything
		return "POS_STATE_NONE";

	case POS_STATE_IDLE:				// idle state, no need controlling anything
		return "POS_STATE_IDLE";

	case L1_GOTO_WAYPOINT:				// target waypoint is far away, we can use L1 and cruise speed
		return _param_line_following_p.get() > FLT_EPSILON ? "L1_GOTO_WAYPOINT (PID)" : "L1_GOTO_WAYPOINT (L1)";

	case WP_ARRIVING:					// target waypoint is close, we need to slow down and head straight to it till stop
		return "WP_ARRIVING";

	case WP_ARRIVED:					// reached waypoint, completely stopped. Make sure mission knows about it
		return "WP_ARRIVED";

	case WP_TURNING:					// we need to turn in place to the next waypoint
		return "WP_TURNING";

	case WP_DEPARTING:					// we turned to next waypoint and must start accelerating
		return "WP_DEPARTING";

	case POS_STATE_STOPPING:			// we hit a waypoint and need to stop
		return "POS_STATE_STOPPING";

	case POS_STATE_MISSION_START:		// turn on what we need for the mission (lights, gas engine throttle, blades)
		return "POS_STATE_MISSION_START";

	case POS_STATE_MISSION_END:			// turn off what we needed for the mission at the end or error
		return "POS_STATE_MISSION_END";

	default:
		return "???control_state_name()???";
	}
}

const char *RoverPositionControl::waypoint_type_name(const uint8_t wptype)
{
	/*
	SETPOINT_TYPE_POSITION=0	# position setpoint
	SETPOINT_TYPE_VELOCITY=1	# velocity setpoint
	SETPOINT_TYPE_LOITER=2		# loiter setpoint
	SETPOINT_TYPE_TAKEOFF=3		# takeoff setpoint
	SETPOINT_TYPE_LAND=4		# land setpoint, altitude must be ignored, descend until landing
	SETPOINT_TYPE_IDLE=5		# do nothing, switch off motors or keep at idle speed (MC)
	*/

	switch (wptype)	{
	case position_setpoint_s::SETPOINT_TYPE_POSITION:
		return "POSITION";

	case position_setpoint_s::SETPOINT_TYPE_VELOCITY:
		return "VELOCITY";

	case position_setpoint_s::SETPOINT_TYPE_LOITER:
		return "LOITER";

	case position_setpoint_s::SETPOINT_TYPE_TAKEOFF:
		return "TAKEOFF";

	case position_setpoint_s::SETPOINT_TYPE_LAND:
		return "LAND";

	case position_setpoint_s::SETPOINT_TYPE_IDLE:
		return "IDLE";

	default:
		return "???";
	}
}

#ifdef DEBUG_MY_PRINT
void RoverPositionControl::debugPrint()
{

	if (hrt_elapsed_time(&_debug_print_last_called) > 500_ms) {

		if (!_ekf_data_good) {
			PX4_WARN("Bad EKF: xy_valid: %s  v_xy_valid: %s  hdg_good_for_control: %s",
				 _ekf_flags(0) ? "true" : "false",
				 _ekf_flags(1) ? "true" : "false",
				 _ekf_flags(2) ? "true" : "false"
				);
		}

		if (_control_mode.flag_control_manual_enabled) {
			if (_control_mode.flag_armed) {
				debugPrintManual();
			}

		} else if (_pos_ctrl_state != POS_STATE_IDLE || !_printed_idle_trace) {
			debugPrintAll();
		}

		_printed_idle_trace = _pos_ctrl_state == POS_STATE_IDLE;

		_debug_print_last_called = _now;
	}

}

// Note: all strings appear on TTY truncated to 127 chars, due to ORB log message buffer definition. See px4_log_modulename() and struct log_message_s.

void RoverPositionControl::debugPrintArriveDepart()
{
	if (hrt_elapsed_time(&_debug_print1_last_called) > 100_ms) {
		if (_tracing_lev > 4) {
			/*
			PX4_INFO_RAW("------- %s  wp_current_dist: %.2f   wp_previous_dist: %.2f   mission_velocity_setpoint: %.2f   mission_thrust: %.2f\n",
					 control_state_name(_pos_ctrl_state),
					 (double)_wp_current_dist, (double)_wp_previous_dist, (double)_mission_velocity_setpoint,
					 (double)_mission_thrust_effort);

			PX4_INFO_RAW("--- hdg_err: %.4f / %.4f   dt: %.3f ms\n",
					 (double)math::degrees(_heading_error), (double)math::degrees(_heading_error_vel),
					 (double)(_dt * 1000.0f));
			*/

			PX4_INFO_RAW("---- %s hdg_err: %.2f / %.2f abbe: %.2f m  turn_sp: %.4f  miss_torq_efrt: %.4f\n",
				     control_state_name(_pos_ctrl_state),
				     (double)math::degrees(_heading_error), (double)math::degrees(_heading_error_vel),
				     (double)_abbe_error, (double)_mission_turning_setpoint,
				     (double)_mission_torque_effort);

			PX4_INFO_RAW("---     misn_vel_sp: %.2f  misn_thrst_eff: %.2f  grd_spd_abs: %.4f  yaw_rate: %.1f\n",
				     (double)_mission_velocity_setpoint, (double)_mission_thrust_effort, (double)_ground_speed_abs,
				     (double)math::degrees(_z_yaw_rate));

			PX4_INFO_RAW("---     dist_trgt: %.2f  dist_prev: %.2f  cls_enough: %.2f  x_vel: %.4f\n",
				     (double)_dist_target, (double)_wp_previous_dist, (double)_wp_close_enough_rad, (double)_x_vel);

			//PX4_INFO_RAW("--- turn_sp: %.4f  miss_torq_efrt: %.4f   cnt_run: %d  cnt_calc: %d\n",
			//		 (double)_mission_turning_setpoint, (double)_mission_torque_effort, _cnt_run, _cnt_calc);
		}

		_debug_print1_last_called = _now;
	}
}

void RoverPositionControl::debugPrintAll()
{
	if (_tracing_lev > 0) {
		PX4_INFO_RAW("=== %s ===================    dt: %.3f ms EKF off: %.1f cm\n", control_state_name(_pos_ctrl_state),
			     (double)(_dt * 1000.0f), (double)(_ekfGpsDeviation * 100.0f));
		//print_run_status();   // scheduler calling rate

		if (_tracing_lev < 4) {
			PX4_INFO_RAW("---    trgt_berng: %.2f  curr_hdg: %.2f  hdg_error: %.2f / %.2f\n",
				     (double) math::degrees(_target_bearing), (double) math::degrees(_current_heading),
				     (double) math::degrees(_heading_error), (double) math::degrees(_heading_error_vel));

			if (_pos_ctrl_state == L1_GOTO_WAYPOINT) {
				PX4_INFO_RAW("---    XTrack err: %.1f cm    L1 acc_demand: %.4f\n",
					     (double)(_crosstrack_error * 100.0f),
					     (double)_nav_lateral_acceleration_demand);

			}
		}

	}

	if (_tracing_lev > 3) {
		PX4_INFO_RAW("---    dist_trgt: %.2f   leg: %.2f   trgt_berng: %.2f  nav_berng: %.2f\n",
			     (double) _dist_target, (double)_leg_distance, (double) math::degrees(_target_bearing),
			     (double) math::degrees(_nav_bearing));

		if (_pos_ctrl_state == L1_GOTO_WAYPOINT) {
			PX4_INFO_RAW("---    XTrack err: %.1f cm    L1 acc_demand: %.4f\n",
				     (double)(_crosstrack_error * 100.0f),
				     (double)_nav_lateral_acceleration_demand);

		}

		PX4_INFO_RAW("---    hdg_er: %.4f / %.4f  abbe: %.2f m   gas: %.2f tool: %.2f alrm: %.1f\n",
			     (double)math::degrees(_heading_error), (double)math::degrees(_heading_error_vel), (double)_abbe_error,
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
			     (int)_vehicle_status.nav_state);

		PX4_INFO_RAW("---    misn_turn_sp: %.4f  misn_torq_effrt: %.4f  cnt_run: %d  cnt_calc: %d\n",
			     (double)_mission_turning_setpoint, (double)_mission_torque_effort, _cnt_run, _cnt_calc);
	}

	_cnt_run = 0;
	_cnt_calc = 0;
}

void RoverPositionControl::debugPrintManual()
{
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
}
#endif // DEBUG_MY_PRINT


#ifdef DEBUG_MY_DATA

// see .../PX4-Autopilot/src/examples/px4_mavlink_debug/px4_mavlink_debug.cpp

void RoverPositionControl::debugPublishAll()
{
	//if(_control_mode_current == UGV_POSCTRL_MODE_AUTO)	// only publish when in Mission -- this doesn't work, log file grows anyway
	{
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

		_debug_data_last_called = _now = hrt_absolute_time();

		//double ms = (_now - _app_started_time) / 1e+5d;

		/*
		 * these are several options on sending debug data:
		 *
		// send one named value:
		_dbg_key.value = sin(ms);
		_dbg_key.timestamp = _now;
		orb_publish(ORB_ID(debug_key_value), _pub_dbg_key, &_dbg_key);

		// send one indexed value:
		_dbg_ind.value = sin(ms);
		_dbg_ind.timestamp = _now;
		orb_publish(ORB_ID(debug_value), _pub_dbg_ind, &_dbg_ind);

		// send one vector:
		_dbg_vect.x = PX4_ISFINITE(_heading_error)      ? _heading_error : 2.0f;			// radians, -1.57...+1.57 range
		_dbg_vect.y = PX4_ISFINITE(_rates_setpoint_yaw) ? _rates_setpoint_yaw : 2.0f;
		_dbg_vect.z = PX4_ISFINITE(_mission_torque_effort) ? _mission_torque_effort : 2.0f;		// -1...+1 range
		_dbg_vect.timestamp = _now;
		orb_publish(ORB_ID(debug_vect), _pub_dbg_vect, &_dbg_vect);

		// test debug_vect with generated curves:
		//_dbg_vect.x = ((int)ms) & 0x7; // sin(ms);
		//_dbg_vect.y = (_cnt_run / 8) & 0x7; //cos(ms);
		//_dbg_vect.z = since_sec * 800.0f;
		//_dbg_vect.timestamp = _now;
		//orb_publish(ORB_ID(debug_vect), _pub_dbg_vect, &_dbg_vect);

		// test debug_vect with generated curves:
		//_dbg_vect.x = ((int)ms) & 0x7; // sin(ms);
		//_dbg_vect.y = cos(ms);
		//_dbg_vect.z = -0.5f;
		//_dbg_vect.timestamp = _now;
		//orb_publish(ORB_ID(debug_vect), _pub_dbg_vect, &_dbg_vect);
		*/

		// send one array:

		/*
		// test debug_array with generated curves:
		for (size_t i = 0; i < debug_array_s::ARRAY_SIZE; i++) {
			_dbg_array.data[i] = i * 0.01 * sin(ms);
		*/

		int i = 1;	// data[0] is reserved for total number of parameters, max 58

		// calculated by L1 controller, and are present as members of _gnd_control:
		_dbg_array.data[i++] = math::degrees(_target_bearing);
		_dbg_array.data[i++] = math::degrees(_nav_bearing);
		_dbg_array.data[i++] = math::degrees(_current_heading);
		_dbg_array.data[i++] = _crosstrack_error;	// meters
		_dbg_array.data[i++] = _mission_torque_effort;
		_dbg_array.data[i++] = _abbe_error; 		// meters

		// calculated values that become published actuator inputs:
		_dbg_array.data[i++] = math::degrees(_current_heading);
		_dbg_array.data[i++] = math::degrees(_heading_error);
		_dbg_array.data[i++] = _mission_torque_effort;	// result of RateControl
		_dbg_array.data[i++] = _torque_control;		// what is sent to actuators, smoothed and trimmed
		_dbg_array.data[i++] = (float)_pos_ctrl_state;

		// When GND_HEADING_P is set to > SIGMA, PID heading error control is in effect:
		_dbg_array.data[i++] = _current_heading;
		_dbg_array.data[i++] = _mission_turning_setpoint; // what State Machine thinks of turning - based on heading error
		_dbg_array.data[i++] = _rates_setpoint_yaw;	  // YAW rate setpoint
		_dbg_array.data[i++] = _mission_torque_effort;	  // result of RateControl
		_dbg_array.data[i++] = _z_yaw_rate;

		// computed or manual torque and thrust (thrust) here, before finally published in _act_controls:
		_dbg_array.data[i++] = _mission_thrust_effort;
		_dbg_array.data[i++] = _thrust_control;

		// some values that we calculate locally to decide on throttling thrust near waypoints:
		_dbg_array.data[i++] = _ground_speed_abs;
		_dbg_array.data[i++] = _x_vel;
		_dbg_array.data[i++] = _ground_speed_ns;

		// When GND_SP_CTRL_MODE = 1, PID speed control is in effect:
		_dbg_array.data[i++] = _mission_velocity_setpoint;
		_dbg_array.data[i++] = _x_vel;
		_dbg_array.data[i++] = _x_vel_ema;
		_dbg_array.data[i++] = _ekfGpsDeviation;	// meters
		_dbg_array.data[i++] = _thrust_control;

		// distances:
		_dbg_array.data[i++] = _dist_target;
		_dbg_array.data[i++] = _wp_current_dist;
		_dbg_array.data[i++] = _wp_previous_dist;
		_dbg_array.data[i++] = _wp_next_dist;

		// other important values:
		_dbg_array.data[i++] = _nav_lateral_acceleration_demand;
		_dbg_array.data[i++] = _mission_turning_setpoint;
		_dbg_array.data[i++] = _wp_close_enough_rad;
		_dbg_array.data[i++] = _acceptance_radius;

		// gps data - heading compared to EKF2:
		_dbg_array.data[i++] = math::degrees(_ekf_current_heading);
		_dbg_array.data[i++] = math::degrees(_mag_current_heading);
		_dbg_array.data[i++] = math::degrees(_gps_current_heading);
		_dbg_array.data[i++] = math::degrees(wrap_pi(_sensor_gps_data.cog_rad));
		//_dbg_array.data[i++] = _sensor_gps_data.heading;			// only works with dual RTK GPS setup
		//_dbg_array.data[i++] = _sensor_gps_data.heading_offset;
		//_dbg_array.data[i++] = _sensor_gps_data.heading_accuracy;

		// gps data - speed compared to EKF2:
		_dbg_array.data[i++] = _gps_ground_speed_abs;
		_dbg_array.data[i++] = _ekf_ground_speed_abs;
		_dbg_array.data[i++] = _sensor_gps_data.vel_m_s;
		_dbg_array.data[i++] = _sensor_gps_data.vel_n_m_s;
		_dbg_array.data[i++] = _sensor_gps_data.vel_e_m_s;
		_dbg_array.data[i++] = _sensor_gps_data.vel_ned_valid ? 1.0f : 0.0f;

		// gps data - performance:
		_dbg_array.data[i++] = (float)_sensor_gps_data.fix_type;
		_dbg_array.data[i++] = (float)_sensor_gps_data.satellites_used;
		_dbg_array.data[i++] = (float)_sensor_gps_data.noise_per_ms;
		_dbg_array.data[i++] = (float)_sensor_gps_data.jamming_indicator;
		_dbg_array.data[i++] = (float)_sensor_gps_data.jamming_state;

		// servo positions after mixers:
		_dbg_array.data[i++] = 	_gas_throttle_servo_position;
		_dbg_array.data[i++] = 	_cutter_servo_position;
		_dbg_array.data[i++] = 	_alarm_servo_position;
		_dbg_array.data[i++] = 	_wheel_left_servo_position;
		_dbg_array.data[i++] = 	_wheel_right_servo_position;

		_dbg_array.data[0] = i;	// must be less than 58, per size of the data[]

		_dbg_array.timestamp = hrt_absolute_time(); // hrt_elapsed_time(&_app_started_time);

		orb_publish(ORB_ID(debug_array), _pub_dbg_array, &_dbg_array);

		//warnx("...sending debug data...");
	}
}
#endif // DEBUG_MY_DATA

