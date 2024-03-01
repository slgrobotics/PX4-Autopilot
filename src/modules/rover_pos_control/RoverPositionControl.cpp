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

//#define PRINT_GPS_WALK

#include "RoverPositionControl.hpp"

using namespace matrix;

/**
 * L1 control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int rover_pos_control_main(int argc, char *argv[]);

RoverPositionControl::RoverPositionControl() :
	ModuleParams(nullptr),
	//WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
	// performance counters:
	//_loop_perf(perf_alloc(PC_ELAPSED,  MODULE_NAME": cycle")) // TODO : do we even need these perf counters
{
	// update parameters from storage:
	ModuleParams::updateParams();

	// update our PIDs etc:
	updateParams();
}

RoverPositionControl::~RoverPositionControl()
{
	ScheduleClear();
	//perf_free(_loop_perf);
}

bool
RoverPositionControl::init()
{
	// Warning: uncommenting the following causes Run() to be called at 100Hz:
	//if (!_vehicle_angular_velocity_sub.registerCallback()) {
	//	PX4_ERR("vehicle angular velocity callback registration failed!");
	//	return false;
	//}

	_app_started_time = _now = hrt_absolute_time();

	_gas_engine_throttle = _param_gas_throttle_idle.get();
	_cutter_setpoint = ACTUATOR_OFF;
	_alarm_dev_level = ACTUATOR_OFF;

	// schedule regular updates
	ScheduleOnInterval(kINTERVAL, kINTERVAL);

#ifdef DEBUG_MY_DATA
	/*
	// advertise named debug value:
	strncpy(_dbg_key.key, "rover_1", 10);
	_dbg_key.value = 0.0f;
	_pub_dbg_key = orb_advertise(ORB_ID(debug_key_value), &_dbg_key);

	// advertise indexed debug value:
	_dbg_ind.ind = 42;
	_dbg_ind.value = 0.5f;
	_pub_dbg_ind = orb_advertise(ORB_ID(debug_value), &_dbg_ind);

	// advertise debug vector:
	strncpy(_dbg_vect.name, "rover_vec", 10);
	_pub_dbg_vect = orb_advertise(ORB_ID(debug_vect), &_dbg_vect);
	*/

	// advertise debug array:
	_dbg_array.id = 1;
	strncpy(_dbg_array.name, "rover_dbg", 10);
	_pub_dbg_array = orb_advertise(ORB_ID(debug_array), &_dbg_array);

#endif // DEBUG_MY_DATA

	return true;
}

#ifdef PRINT_GPS_WALK
static hrt_abstime _gps_print_last_called;
static float _gps_print_interval_sec = 5.0f;
static double _last_lat {0};
static double _last_lon {0};
static double _last_alt {0};
static double _last_lat_e {0};
static double _last_lon_e {0};
static double _last_alt_e {0};
#endif // PRINT_GPS_WALK

void
RoverPositionControl::update_orientation()
{
	//_dt = 0.004f; // Using non zero value to a avoid division by zero, assume 250 Hz cycle (4 ms)
	_dt = 0.01f;

	if (_update_orientation_last_called > 0) {
		int64_t elapsed = hrt_elapsed_time(&_update_orientation_last_called);
		_dt = (float)elapsed * 1e-6f;		// seconds, usually 0.004 in sitl

		if (_dt > 0.9f) {
			_dt = 0.01f;        // this was a first call after a state change
		}
	}

	_update_orientation_last_called = _now = hrt_absolute_time();

	matrix::Vector3f gs(_local_pos.vx, _local_pos.vy,  _local_pos.vz);
	_ground_speed = gs;

	double f_lat = _sensor_gps_data.latitude_deg;	// must be float64 for RTK precision
	double f_lon = _sensor_gps_data.longitude_deg;
#ifdef PRINT_GPS_WALK
	double f_alt = _sensor_gps_data.altitude_msl_m;
	double f_alte = _sensor_gps_data.altitude_ellipsoid_m;
	bool printed = false;
#endif // PRINT_GPS_WALK

	if (_sensor_gps_data.fix_type == 6) { // RTK fix
		matrix::Vector2d cp(f_lat, f_lon);

#ifdef PRINT_GPS_WALK

		if (hrt_elapsed_time(&_gps_print_last_called) > _gps_print_interval_sec * 1000_ms) {
			PX4_INFO_RAW("GPS6: Lat: %.10f Lon: %.10f Alt: %.6f : %.6f H: %.2f deg\n", f_lat, f_lon, f_alt, f_alte,
				     (double)math::degrees(_sensor_gps_data.heading));
			printed = true;
		}

#endif // PRINT_GPS_WALK

		_current_position = cp;

	} else {
		// No RTK fix - we can only rely on EKF2 estimated position:

		matrix::Vector2d cp(_global_pos.lat, _global_pos.lon);

#ifdef PRINT_GPS_WALK

		if (hrt_elapsed_time(&_gps_print_last_called) > _gps_print_interval_sec * 1000_ms) {
			PX4_INFO_RAW("GPS%d: Lat: %.10f Lon: %.10f Alt: %.6f : %.6f\n", _sensor_gps_data.fix_type, f_lat, f_lon, f_alt, f_alte);
			printed = true;
		}

#endif // PRINT_GPS_WALK

		_current_position = cp;
	}

#ifdef PRINT_GPS_WALK

	if (printed) {
		PX4_INFO_RAW("EKF2: Lat: %.10f Lon: %.10f Alt: %.6f : %.6f H: %.2f deg\n", _global_pos.lat, _global_pos.lon,
			     (double)_global_pos.alt, (double)_global_pos.alt_ellipsoid, (double)_ekf_current_heading);
		PX4_INFO_RAW("=== LLH Walk: %.4f / %.4f  %.4f / %.4f  %.4f / %.4f  meters ===\n", (f_lat - _last_lat) * 1e5,
			     (_global_pos.lat - _last_lat_e) * 1e5, (f_lon - _last_lon) * 1e5, (_global_pos.lon - _last_lon_e) * 1e5,
			     f_alt - _last_alt, (double)_global_pos.alt - _last_alt_e);
		_last_lat = f_lat;
		_last_lon = f_lon;
		_last_alt = f_alt;
		_last_lat_e = _global_pos.lat;
		_last_lon_e = _global_pos.lon;
		_last_alt_e = _global_pos.alt;
		_gps_print_last_called = _now;
	}

#endif // PRINT_GPS_WALK

	if (_local_pos.v_xy_valid) {
		// Velocity in body frame
		const Dcmf R_to_body(Quatf(_vehicle_att.q).inversed());
		const Vector3f vel = R_to_body * Vector3f(_local_pos.vx, _local_pos.vy, _local_pos.vz);
		_x_vel = vel(0);
	}

	const matrix::Vector2f gs2d(_ground_speed);
	_ground_speed_2d = gs2d;
	_ground_speed_ns = _ground_speed_2d.norm_squared();		// L1 desired_r is dependent on this
	_ekf_ground_speed_abs = _ground_speed_2d.length();

	_ground_speed_abs = _ekf_ground_speed_abs;

	_ekf_current_heading = Eulerf(Quatf(_vehicle_att.q)).psi();

	_current_heading = wrap_pi(_ekf_current_heading + _ekf_heading_correction);	// radians to absolute North, -PI...PI

#if !defined(CONFIG_ARCH_BOARD_PX4_SITL)
	// based on preference and availability, reassign speed and/or measurements to RTK GPS:
	_ground_speed_abs = _speed_prefer_gps
			    && PX4_ISFINITE(_gps_ground_speed_abs) ? _gps_ground_speed_abs : _ekf_ground_speed_abs;
	_current_heading = _heading_prefer_gps
			   && PX4_ISFINITE(_gps_current_heading) ? _gps_current_heading : _ekf_current_heading;
#endif

	_x_vel_ema = _velocity_measured_ema.Compute(_x_vel);	// smooth the jitter for the PID's input
}

void
RoverPositionControl::control_position(const matrix::Vector2d &current_position)
{
#ifdef DEBUG_MY_PRINT
	_cnt_calc++;
#endif // DEBUG_MY_PRINT

	// _pos_sp_triplet.current is always valid here
	_wp_current_dist = get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon,
			   _pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon);

	_dist_target = _wp_current_dist;

	// we can have a LOITER waypoint arriving (1) on mission end and (2) when "Go to this point" is clicked on the map at any time.
	// _vehicle_status.nav_state will be MAIN_STATE_AUTO_MISSION = 3 at the mission, and MAIN_STATE_AUTO_LOITER = 4 at the go-to

	if (_pos_sp_triplet.current.type != position_setpoint_s::SETPOINT_TYPE_LOITER
	    || _vehicle_status.nav_state != vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) {
		// ============= Prepare variables related to distances between waypoints and vehicle motion: ===================================

		/* get circle mode */
		//bool was_circle_mode = _gnd_control.circle_mode();

		/* current waypoint (the one we are currently heading for) */
		const matrix::Vector2d t_wp(_pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon);
		_curr_wp = t_wp;

		/* previous waypoint */
		_prev_wp = _curr_wp;

		if (_pos_sp_triplet.previous.valid) {
			_prev_wp(0) = _pos_sp_triplet.previous.lat;
			_prev_wp(1) = _pos_sp_triplet.previous.lon;
			_wp_previous_dist = get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon,
					    _pos_sp_triplet.previous.lat, _pos_sp_triplet.previous.lon);

		} else {
			_wp_previous_dist = NAN;
		}

		if (_pos_sp_triplet.next.valid) {
			_wp_next_dist = get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon,
					_pos_sp_triplet.next.lat, _pos_sp_triplet.next.lon);

		} else {
			_wp_next_dist = NAN;
		}

		//PX4_INFO("Setpoint type %s", waypoint_type_name(_pos_sp_triplet.current.type));
		//PX4_INFO(" State machine state %d", (int) _pos_ctrl_state);
		//PX4_INFO(" Setpoint Lat %f, Lon %f", (double) curr_wp(0), (double)curr_wp(1));
		//PX4_INFO(" Distance to target %f", (double) _dist_target);

		_wp_close_enough_rad = _acceptance_radius;

	} else {

		// if mission isn't valid or is finished (loitering at last waypoint) - we stay put

		// upon arrival to last waypoint, state will be POS_STATE_ARRIVING

		switch (_pos_ctrl_state) {
		case POS_STATE_NONE:
		case POS_STATE_IDLE:

			_dist_target = _wp_current_dist = NAN;

			break;

		case POS_STATE_STOPPING:
		case WP_ARRIVED:
		case POS_STATE_MISSION_END:

			break;

		default:
			setStateMachineState(POS_STATE_STOPPING);
			break;
		}
	}

	// ============= Work the State machine:  ====================================

	workStateMachine();	// computes _mission_turning_setpoint and _mission_velocity_setpoint

	// ============= Pass speed/thrust/torque to actuators: =======================

	adjustThrustAndTorque();	// have PIDs work on control_effort and _mission_velocity_setpoint for smooth control, computes _mission_thrust_effort and _mission_torque_effort

	setActControls();		// sends _mission_thrust_effort and _mission_torque_effort to actuators

#ifdef DEBUG_MY_PRINT
	debugPrint();
#endif // DEBUG_MY_PRINT
}

bool RoverPositionControl::checkNewWaypointArrival()
{
	// Note _prev_curr_wp is different to the local prev_wp which is related to a mission waypoint.
	//	  this is strictly for detecting arrival of new current wp, a new target, between the state machine cycles:
	const float dist_between_waypoints = get_distance_to_next_waypoint((double)_prev_curr_wp(0), (double)_prev_curr_wp(1),
					     (double)_curr_wp(0), (double)_curr_wp(1));

	if (dist_between_waypoints > SIGMA) {
//#ifdef DEBUG_MY_PRINT
		PX4_WARN(" A new waypoint has arrived at distance: %.2f", (double)dist_between_waypoints);
//#endif // DEBUG_MY_PRINT

		if (_pos_sp_triplet.current.valid && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER) {
			PX4_WARN("Current setpoint type: %s", waypoint_type_name(_pos_sp_triplet.current.type));
		}

		_prev_curr_wp = _curr_wp;	// prevents coming here again

		return true;
	}

	//PX4_INFO(" Distance between prev and curr waypoints %.2f", (double)dist_between_waypoints);
	return false;
}

float
RoverPositionControl::control_yaw_rate(const vehicle_angular_velocity_s &rates,
				       const vehicle_rates_setpoint_s &rates_sp)
{
	// code below is a combination of:
	//                                  https://github.com/PX4/PX4-Autopilot/pull/20082
	//                              and src/modules/mc_rate_control/MulticopterRateControl.cpp:185+

	float dt = (_control_yaw_rate_last_called > 0) ? hrt_elapsed_time(&_control_yaw_rate_last_called) * 1e-6f : 0.01f;
	_control_yaw_rate_last_called = hrt_absolute_time();

	// reset integral if disarmed
	if (!_control_mode.flag_armed) {
		_rate_control.resetIntegral();
	}

#ifdef PUBLISH_THRUST_TORQUE
	// update saturation status from control allocation feedback (only if CA is configured)
	control_allocator_status_s control_allocator_status;

	if (_control_allocator_status_sub.update(&control_allocator_status)) {
		Vector<bool, 3> saturation_positive;
		Vector<bool, 3> saturation_negative;

		if (!control_allocator_status.torque_setpoint_achieved) {
			for (int i = 0; i < 3; i++) {
				if (control_allocator_status.unallocated_torque[i] > FLT_EPSILON) {
					saturation_positive(i) = true;
					PX4_WARN("saturation_positive %d", i);

				} else if (control_allocator_status.unallocated_torque[i] < -FLT_EPSILON) {
					saturation_negative(i) = true;
					PX4_WARN("saturation_negative %d", i);
				}
			}
		}

		// TODO: send the unallocated value directly for better anti-windup
		_rate_control.setSaturationStatus(saturation_positive, saturation_negative);
	}

#endif // PUBLISH_THRUST_TORQUE

	const matrix::Vector3f vehicle_rates(rates.xyz[0], rates.xyz[1], rates.xyz[2]);		// measured angular rates
	const matrix::Vector3f rates_setpoint(rates_sp.roll, rates_sp.pitch, rates_sp.yaw);	// desired angular rates

	// when stopped, lock PID integrator:
	bool is_stopped = bool(_ground_speed_abs <
			       _param_rate_i_minspeed.get()); // if true, integral is not allowed to accumulate (I-component of PID disabled)

	const matrix::Vector3f angular_acceleration{rates.xyz_derivative};	// measured angular accelerations

	// Now call the magic, assuming that it computes body torque action:
	const matrix::Vector3f torque = _rate_control.update(vehicle_rates, rates_setpoint, angular_acceleration, dt,
					is_stopped);

	// publish rate controller status
	rate_ctrl_status_s rate_ctrl_status{};
	_rate_control.getRateControlStatus(rate_ctrl_status);
	rate_ctrl_status.timestamp = hrt_absolute_time();
	_controller_status_pub.publish(rate_ctrl_status);

	// only interested in yaw (z) axis:
	float steering_input = math::constrain(torque(2), -1.0f, 1.0f);

	//PX4_WARN("mission_torque_effort: %.3f   steering_input: %.3f", (double)_mission_torque_effort, (double)steering_input);

	return steering_input;
}

void
RoverPositionControl::Run()
{
	// we run at kINTERVAL (100 Hz on RPi, 250 Hz in SITL), as scheduled

#ifdef DEBUG_MY_PRINT
	_cnt_run++;
#endif // DEBUG_MY_PRINT

	_now = hrt_absolute_time();

	_rates_setpoint_yaw = NAN;	// for logging, will be set in control_position()
	_z_yaw_rate = NAN;			//              will be set in poll_everything()

	poll_everything();

	// update the reset counters in any case
	// TODO: use it in logging/tracing?
	_pos_reset_counter = _global_pos.lat_lon_reset_counter;

	update_orientation();

	// =============== Now get to actually controlling the vehicle: ============================

	_thrust_control = _torque_control = NAN;
	_mission_thrust_effort = _mission_torque_effort = NAN;

	_control_mode_current = UGV_POSCTRL_MODE_OTHER;		// assume Manual mode

	if (!_control_mode.flag_control_manual_enabled && _control_mode.flag_control_position_enabled) {

		if ((_control_mode.flag_control_auto_enabled ||
		     _control_mode.flag_control_offboard_enabled) && _pos_sp_triplet.current.valid) {
			/* AUTONOMOUS FLIGHT */

			_control_mode_current = UGV_POSCTRL_MODE_AUTO;

			// _acceptance_radius - if large enough, it will be used for mission advancement to next WP

			//TODO: check if acceptance radius makes sense here
			// _acceptance_radius = _gnd_control.switch_distance(100.0f);
			_acceptance_radius = _param_wp_precision.get(); // GND_WP_PRECISN, set it to 0.1..5.0 meters

#ifdef QQQ

			if (_pos_ctrl_state == POS_STATE_NONE || _pos_ctrl_state == POS_STATE_IDLE) {
				/*if(_mission_result.valid && !_mission_result.finished) {
						PX4_INFO("Switched to Auto control mode, mission valid");
						setStateMachineState(POS_STATE_MISSION_START);
				} else { */
				if (_mission_result.valid) {
					if (!_mission_result.finished) {
						PX4_INFO("Switched to Auto control mode, mission valid");
						setStateMachineState(POS_STATE_MISSION_START);
					}

					// else mission valid and finished - keep state in IDLE or NONE

				} else if (_mission_result.finished) {
					// mission result not valid, strange...
					PX4_WARN("Cannot switch to Auto control mode. Mission valid: %s   finished: %s",
						 _mission_result.valid ? "yes" : "no", _mission_result.finished ? "yes" : "no");
				}
			}

#endif

			// This is where all the magic happens:
			control_position(_current_position);

			// publish controller status, mostly for tracing and tuning:
			publishControllerStatus();
		}

	} else if (_control_mode.flag_control_manual_enabled) {
		// Direct R/C control - maybe with PIDs:
		control_position_manual();
	}

	//
	// note: we come here at kINTERVAL (100/500 Hz?), as scheduled
	//

	// =============== Set actuators to torque and thrust values we just computed: ============================

	/* Only publish if any of the proper modes are enabled */
	if (_control_mode.flag_control_position_enabled ||
	    _control_mode.flag_control_manual_enabled) {

		hrt_abstime timestamp_sample = _angular_velocity.timestamp_sample;

#ifdef PUBLISH_THRUST_TORQUE
		// Body Torque and Thrust for mixing by Control Allocator:
		publishTorqueSetpoint(timestamp_sample);
		publishThrustSetpoint(timestamp_sample);

		// "Servo" channels - Gas Engine Throttle, Cutter, Strobe, Horn, Alarm:
		publishAuxActuators(timestamp_sample);
#else
		// Directly publish to actuators, calling DifferentialDriveKinematics:

		float linear_velocity_x = _thrust_control * _param_rdd_thrust_scaler.get();	// RDD_THRUST_SC
		float yaw_rate = _torque_control * _param_rdd_torque_scaler.get();		// RDD_TORQUE_SC

		// get the wheel speeds from the inverse kinematics class (DifferentialDriveKinematics)
		_wheel_speeds = _differential_drive_kinematics.computeInverseKinematics(linear_velocity_x, yaw_rate);

		/*
			PX4_INFO_RAW("THR %f ->  %f     TRQ %f -> %f  :   %f   %f\n",
				(double)_mission_thrust_effort, (double)_thrust_control, (double)_mission_torque_effort,
				(double)_torque_control, (double)_wheel_speeds(0), (double)_wheel_speeds(1));
		*/

		// Check if max_angular_wheel_speed is zero
		//const bool setpoint_timeout = (_differential_drive_setpoint.timestamp + 100_ms) < now;
		//const bool valid_max_speed = _param_rdd_speed_scale.get() > FLT_EPSILON;

		if (!_control_mode.flag_armed) { // || setpoint_timeout || !valid_max_speed) {
			_wheel_speeds = {0.0f, 0.0f}; // stop
		}

		_wheel_speeds = matrix::constrain(_wheel_speeds, -1.f, 1.f);

		// publish data to actuator_motors (output module):
		// (does not work, as Control Allocator doesn't have suitable Effectiveness class)
		//publishWheelMotors(timestamp_sample);

		// "Servo" channels - Gas Engine Throttle, Cutter, Strobe, Horn, Alarm:
		// (also publishes wheel speeds directly as Servo 1 and 2)
		publishAuxActuators(timestamp_sample);
#endif // PUBLISH_THRUST_TORQUE

	}

#ifdef DEBUG_MY_DATA
	debugPublishAll();
#endif // DEBUG_MY_DATA

}

void RoverPositionControl::control_position_manual()
{
	// we are in Manual mode

	if (_pos_ctrl_state != POS_STATE_NONE && _pos_ctrl_state != POS_STATE_IDLE) {
		PX4_INFO("Switched to Manual control mode");
		setStateMachineState(POS_STATE_MISSION_END);
		workStateMachine();
	}

	manual_control_setpoint_poll();		// R/C inputs, fills _torque_control_manual, _thrust_control_manual

	if (_control_mode.flag_armed) {
#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
		_manual_using_pids = _param_manual_use_pid.get() >
				     0; // you can set GND_MAN_USE_PID=1 when running sim on a PC to debug PID response
		_manual_drive_straight = _param_manual_drive_straight.get() >
					 0; // you can set GND_MAN_STRAIGHT=1 when running sim on a PC
#else
		_manual_using_pids = _param_manual_use_pid.get() > 0
				     && _cutter_setpoint_manual > 0.5f; // set GND_MAN_USE_PID=1 and leftmost switch on R/C transmitter: up: -1, down: +1
		_manual_drive_straight = _param_manual_drive_straight.get() > 0
					 && _cutter_setpoint_manual > 0.5f; // set GND_MAN_STRAIGHT=1 and leftmost switch on R/C transmitter: up: -1, down: +1
#endif // CONFIG_ARCH_BOARD_PX4_SITL

		//PX4_INFO("cutter: %f    yaw: %f    thrust: %f ", (double)_cutter_setpoint_manual, (double)_torque_control_manual, (double)_thrust_control_manual);

		if (_manual_using_pids) {
			_mission_turning_setpoint = _torque_control_manual /
						    _param_manual_yaw_scaler.get(); // GND_MAN_YAW_SC +- 1.0 controlled by right stick horizontal movement
			_mission_velocity_setpoint = _thrust_control_manual *
						     _param_gndspeed_trim.get(); // GND_SPEED_TRIM +- max (a.k.a. trim) speed controlled by right stick vertical movement

			adjustThrustAndTorque(); // will call PIDs via computeTorqueEffort() and computeThrust(), result in _mission_torque_effort, _mission_thrust_effort;
			setActControls();
			_cutter_setpoint = -1.0f;		// Cutter off

		} else {
			_torque_control = _torque_control_manual;
			_thrust_control = _thrust_control_manual;
			_cutter_setpoint = _cutter_setpoint_manual;		// Cutter clutch - leftmost switch on R/C transmitter
		}

		if (_manual_drive_straight) {
			_torque_control = 0.0f;	// zero torque/yaw component
		}

		_gas_engine_throttle = _gas_throttle_manual;	// Left knob on R/C transmitter
		_alarm_dev_level = _alarm_dev_level_manual;		// Horn - right knob on R/C transmitter
	}

#ifdef DEBUG_MY_PRINT
	debugPrint();
#endif // DEBUG_MY_PRINT
}

int RoverPositionControl::task_spawn(int argc, char *argv[])
{
	RoverPositionControl *instance = new RoverPositionControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int RoverPositionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RoverPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Controls the position of a ground rover using an L1 controller.

Publishes `vehicle_thrust_setpoint (only in x) and vehicle_torque_setpoint (only yaw)` messages at IMU_GYRO_RATEMAX.

(Old: Publishes `actuator_controls_0` messages at IMU_GYRO_RATEMAX.)

### Implementation
Currently, this implementation supports only a few modes:

 * Full manual: Thrust and torque controls are passed directly through to the actuators
 * Auto mission: The rover runs missions
 * Loiter: The rover will navigate to within the loiter radius, then stop the motors

### Examples
CLI usage example:
$ rover_pos_control start
$ rover_pos_control status
$ rover_pos_control stop

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rover_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start")
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int RoverPositionControl::print_status()
{
	PX4_INFO("Rover Position Control - heavy vehicle");

	print_run_status();		// Scheduler rate and status

	return 0;
}

int rover_pos_control_main(int argc, char *argv[])
{
	return RoverPositionControl::main(argc, argv);
}
