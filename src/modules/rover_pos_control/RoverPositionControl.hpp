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
 * All the ackowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Marco Zorzi <mzorzi@student.ethz.ch>
 */

#pragma once

#include <float.h>

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/l1/ECL_L1_Pos_Controller.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <lib/pid/pid.h>
#include <lib/rate_control/rate_control.hpp>
#include <lib/motion_planning/VelocitySmoothing.hpp>
#include <lib/pure_pursuit/PurePursuit.hpp>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/position_controller_status.h>
#ifdef PUBLISH_THRUST_TORQUE
#include <uORB/topics/control_allocator_status.h>
#endif // PUBLISH_THRUST_TORQUE
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/navigator_mission_item.h>
#include <uORB/topics/vehicle_status.h>

#ifdef DEBUG_MY_DATA
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/debug_value.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/debug_array.h>
#endif // DEBUG_MY_DATA

using namespace matrix;

using matrix::Dcmf;

using namespace time_literals;

#define SIGMA 0.000001f

template<typename Type>
class Ema
{

public:
	inline Ema() {};
	inline ~Ema() {};

	inline void init(int period)
	{
		emaPeriod = period;
		multiplier = 2.0 / (1.0 + emaPeriod);
		valuePrev = NAN;
	};

	const inline Type Compute(Type val)
	{
		const Type valEma = emaPeriod <= 1 || !PX4_ISFINITE(valuePrev) ? val : ((val - valuePrev) * multiplier + valuePrev);
		valuePrev = valEma;
		return valEma;
	};

	const inline Type ValuePrev() { return valuePrev; };

	inline void Reset()	{ valuePrev = NAN; };

private:
	// variables :
	Type valuePrev{NAN};
	int emaPeriod{0};
	Type multiplier;
};

using Emaf = Ema<float>;

// For binary actuators, connected through PCA9685 "servo relays":
#define ACTUATOR_ON   (1.0f)
#define ACTUATOR_OFF (-1.0f)

class RoverPositionControl final : public ModuleBase<RoverPositionControl>, public ModuleParams,
	public px4::ScheduledWorkItem //, public px4::WorkItem
{
public:
	RoverPositionControl();
	~RoverPositionControl();
	RoverPositionControl(const RoverPositionControl &) = delete;
	RoverPositionControl operator=(const RoverPositionControl &other) = delete;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	int print_status();

	bool init();

protected:
	void updateParams() override;

private:
	void Run() override;

// see what your GCC predefines:    echo | gcc -dM -E -
//                                  echo | /usr/bin/arm-linux-gnueabihf-g++ -dM -E -

//#ifdef __x86_64
#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
	static const hrt_abstime	kINTERVAL {2_ms};	/**< Sim with Gazebo requires 2_ms = 500Hz, and runs on a PC with x86 CPU at 250Hz with attitute update cycle */
#else
	static const hrt_abstime	kINTERVAL {5_ms};	/**< 5_ms = 200Hz base rate for arm boards like Raspberry Pi with cortex-a53 CPU, runs as scheduled */
#endif // CONFIG_ARCH_BOARD_PX4_SITL

	const float MIN_PID_INTERVAL {0.001f}; // seconds - only matters for PID_MODE_DERIVATIV_CALC or PID_MODE_DERIVATIV_CALC_NO_SP

	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	uORB::Subscription _control_mode_sub{ORB_ID(vehicle_control_mode)}; /**< control mode subscription */
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _sensor_gps_sub{ORB_ID(sensor_gps)};
	uORB::Subscription _magnetometer_sub{ORB_ID(vehicle_magnetometer)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)}; /**< notification of manual control updates */
	uORB::Subscription _position_setpoint_triplet_sub{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _att_sp_sub{ORB_ID(vehicle_attitude_setpoint)};
	//uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};

	uORB::Subscription _home_position_sub{ORB_ID(home_position)};
	uORB::Subscription _mission_sub{ORB_ID(mission)}; /**< mission subscription */
	uORB::Subscription _mission_result_sub{ORB_ID(mission_result)}; /**< mission result subscription */
	uORB::Subscription _navigator_mission_item_sub{ORB_ID(navigator_mission_item)}; /**< navigator_mission_item subscription */
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)}; /**< vehicle status subscription */
	uORB::Subscription _actuator_outputs_sub{ORB_ID(actuator_outputs)}; /**< actuator outputs subscription */
#ifdef PUBLISH_THRUST_TORQUE
	uORB::Subscription _control_allocator_status_sub {ORB_ID(control_allocator_status)};	/**< for controller saturation status */
#endif // PUBLISH_THRUST_TORQUE

	uORB::Publication<vehicle_attitude_setpoint_s>	_attitude_sp_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<position_controller_status_s>	_pos_ctrl_status_pub{ORB_ID(position_controller_status)};  /**< navigation capabilities publication */
	uORB::PublicationMulti<actuator_motors_s> 	_actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<actuator_servos_s>		_actuator_servos_pub{ORB_ID(actuator_servos)};
	uORB::PublicationMulti<rate_ctrl_status_s>	_controller_status_pub{ORB_ID(rate_ctrl_status)};

	manual_control_setpoint_s		_manual_control_setpoint{};	/**< r/c channel data */
	position_setpoint_triplet_s		_pos_sp_triplet{};		/**< triplet of mission items */
	vehicle_attitude_setpoint_s		_att_sp{};			/**< attitude setpoint > */
	vehicle_control_mode_s			_control_mode{};		/**< control mode */
	vehicle_global_position_s		_global_pos{};			/**< global vehicle position */
	sensor_gps_s				_sensor_gps_data{};		/**< raw gps data */
	vehicle_magnetometer_s			_magnetometer{};		/**< raw magnetometer data */
	vehicle_local_position_s		_local_pos{};			/**< global vehicle position */
	vehicle_attitude_s			_vehicle_att{};
	//trajectory_setpoint_s			_trajectory_setpoint{};		/**< speed or outboard control */
	vehicle_angular_velocity_s		_angular_velocity{};		/**< store gyro changes */
	vehicle_rates_setpoint_s		_rates_setpoint{};

	mission_s				_mission{};			/**< mission */
	mission_result_s			_mission_result{};		/**< mission result */
	navigator_mission_item_s		_navigator_mission_item{};	/**< navigator_mission_item */
	vehicle_status_s			_vehicle_status{};		/**< commander state */
	actuator_outputs_s			_actuator_outputs{};		/**< actuator outputs */

	uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s>	_vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};

	uORB::SubscriptionData<vehicle_acceleration_s>		_vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	hrt_abstime _manual_setpoint_last_called{0};
	hrt_abstime _turn_goal_last_reached{0};
	hrt_abstime _control_yaw_rate_last_called{0};

	MapProjection _global_ned_proj_ref{};

	// estimator reset counters
	uint8_t _pos_reset_counter{0};		// captures the number of times the estimator has reset the horizontal position

	// PID controller for the speed:
	PID_t _speed_ctrl{};

	// velocity smoothing for forward motion:
	VelocitySmoothing _forwards_velocity_smoothing;

	// PID controller for the Line Following:
	PID_t _line_following_ctrl{};
	float _max_yaw_rate{0.f};

	// The PID controller for the heading (calculates yaw rate):
	PID_t _pid_heading;
	// The PID controller for yaw rate (calculates wheels speed difference)
	PID_t _pid_yaw_rate;

	// Line following controller:
	ECL_L1_Pos_Controller _gnd_control{};

	// Yaw rate controller:
	RateControl _rate_control{};

	PurePursuit _pure_pursuit{this}; // Pure pursuit library

	enum POS_CTRLSTATES : int {
		POS_STATE_NONE,			// undefined/invalid state, no need controlling anything
		POS_STATE_IDLE,			// idle state, no need controlling anything
		L1_GOTO_WAYPOINT,		// target waypoint is far away, we can use L1 and cruise speed
		WP_ARRIVING,			// target waypoint is close, we need to slow down and head straight to it till stop
		WP_ARRIVED,			// reached waypoint, completely stopped. Make sure mission knows about it
		WP_TURNING,			// we need to turn in place to the next waypoint
		WP_DEPARTING,			// we turned to next waypoint and must start accelerating
		POS_STATE_STOPPING,		// we hit a waypoint and need to stop
		POS_STATE_MISSION_START,	// turn on what we need for the mission (lights, gas engine throttle, blades)
		POS_STATE_MISSION_END		// turn off what we needed for the mission at the end or error
	} _pos_ctrl_state {POS_STATE_NONE};	// Position control state machine

	const char *control_state_name(const POS_CTRLSTATES state);

	const char *waypoint_type_name(const uint8_t wptype);

	// -----------------------------------------------------------------------------------------------------------------------------
	// added to support heavy differential drive rover:

#ifdef PUBLISH_THRUST_TORQUE
	void publishTorqueSetpoint(const hrt_abstime &timestamp_sample, float torque);
	void publishThrustSetpoint(const hrt_abstime &timestamp_sample, float thrust);
	void publishAuxActuators(const hrt_abstime &timestamp_sample);
#else
	//void publishWheelMotors(const hrt_abstime &timestamp_sample);
	void publishAuxActuators(const hrt_abstime &timestamp_sample);

	Vector2f _wheel_speeds{0.0f, 0.0f};
#endif // PUBLISH_THRUST_TORQUE

	void publishControllerStatus();

	bool updateBearings();
	void updateWaypoints();
	void updateWaypointDistances();
	void updateEkfGpsDeviation();
	bool checkNewWaypointArrival();
	float adjustMissionVelocitySetpoint();
	float computeTurningSetpoint();
	float computeTorqueEffort();
	float computeThrustEffort();
	void computeWheelSpeeds();
	void computeCrosstrackError();
	void adjustThrustAndTorque();
	void resetTorqueControls();
	void resetThrustControls();
	void resetVelocitySmoothing();
	void setDefaultMissionSpeed();
	bool detectOvershot();
	void adjustAcuatorSetpoints();
	void setActControls();

	void workStateMachine();
	void setStateMachineState(const POS_CTRLSTATES desiredState);

	// good for small numbers 0..1 - aggressive around 0, slower to the sides:
	inline float sqrt_signed(float val)	{ return PX4_ISFINITE(val) ? sqrt(abs(val)) * sign(val) : 0.0f; };

	hrt_abstime _app_started_time{0};

	/* previous waypoint, NOT the Mission triplet previous wp.
	     This one is strictly for detecting arrival of a new current wp in checkNewWaypointArrival()
	     a new target, between the state machine cycles */
	Vector2d _prev_curr_wp{0, 0};

	// Waypoints - from position_setpoint_triplet_s
	Vector2d _curr_wp{0, 0};
	Vector2f _curr_wp_ned{0, 0};
	Vector2d _prev_wp{0, 0};
	Vector2f _prev_wp_ned{0, 0};
	Vector2d _next_wp{0, 0};

	Vector2d _home_position{0, 0};

	Vector2d _curr_pos{0, 0};
	Vector2f _curr_pos_ned{0, 0};	// local projection - updated when polling

	Vector3f _ground_speed{0, 0, 0};
	Vector2f _ground_speed_2d{0, 0};

	// these are calculated by L1 controller, and are present as members of _gnd_control:
	float _target_bearing{0.0f};		// the direction between the rover position and next (current) waypoint
	float _nav_bearing{0.0f};		// bearing from current position to L1 point
	float _crosstrack_error{0.0f};		// meters, how far we are from the A-B line (A = previous, visited waypoint, B = current waypoint, target)
	float _ekfGpsDeviation{0.0f};		// meters, how far is EKF2 calculated position from GPS reading
	bool _ekf_data_good{false};		// combination of: _local_pos.xy_valid && v_xy_valid && heading_good_for_control
	Vector3<bool> _ekf_flags;

	// Mission metrics:
	float _crosstrack_error_avg{NAN};	// average (compound) absolute crosstrack error diring the line following leg
	float _crosstrack_error_max{NAN};	// max absolute crosstrack error diring the line following leg

	float _crosstrack_error_mission_avg{NAN};	// average (compound) absolute crosstrack error for the mission
	float _crosstrack_error_mission_max{NAN};	// max absolute crosstrack error for the mission

	// to compute _crosstrack_error_avg / _max:
	float _cte_accum{NAN};
	int _cte_count{0};
	hrt_abstime _cte_lf_started{0};
	hrt_abstime _cte_lf_tick{0};

	float _cte_accum_mission{NAN};
	int _cte_count_mission{0};
	int _cte_count_outside{0};

	inline void cte_begin_mission()
	{
		_cte_accum_mission = 0.0f; _cte_count_mission = 0; _cte_count_outside = 0;
		_crosstrack_error_mission_avg = NAN; _crosstrack_error_mission_max = 0.0f;
	}

	inline void cte_end_mission()
	{
		_crosstrack_error_mission_avg = _cte_accum_mission / _cte_count_mission;
	};

	inline void cte_begin()
	{
		_cte_accum = 0.0f; _cte_count = 0;
		_cte_lf_started = _cte_lf_tick = _timestamp;
		_crosstrack_error_avg = NAN; _crosstrack_error_max = 0.0f;
	};

	inline void cte_compute()
	{
		if (PX4_ISFINITE(_crosstrack_error) && hrt_elapsed_time(&_cte_lf_started) > 5 * 1_s) {
			float cte_abs = abs(_crosstrack_error);

			_crosstrack_error_max = math::max(_crosstrack_error_max, cte_abs);
			_cte_accum += cte_abs;
			++_cte_count;

			_crosstrack_error_mission_max = math::max(_crosstrack_error_mission_max, cte_abs);
			_cte_accum_mission += cte_abs;
			++_cte_count_mission;

			if (hrt_elapsed_time(&_cte_lf_tick) > 1_s) {

				// every second we see if we are outside +-20 cm corridor, and count those seconds:

				_cte_lf_tick = _timestamp;

				if (cte_abs > 0.2f) {
					++_cte_count_outside;
					PX4_WARN("+++++++++++++  outside: %i +++++++++++++", _cte_count_outside);
				}
			}
		}
	};

	inline void cte_end()
	{
		_crosstrack_error_avg = _cte_accum / _cte_count;
		_crosstrack_error_mission_avg = _cte_accum_mission / _cte_count_mission; // keep it current for tracing
	};

	// Compute desired yaw rate for the vehicle. See src/modules/rover_differential/RoverDifferential.cpp
	struct differential_setpoint {
		float desired_speed{0.f};
		float desired_yaw_rate{0.f};
	};

	differential_setpoint _rd_guidance{0.0f, 0.0f};

	void computeRdGuidance();

	inline void resetRdGuidance() { _rd_guidance.desired_speed = _rd_guidance.desired_yaw_rate = 0.0f; };

	// calculated values that become published actuator inputs:
	float _mission_torque_effort{0.0f};	// Rate control output (yaw a.k.a. heading)
	Emaf  _mission_torque_ema;		// to remove spikes in _mission_torque_effort before publishing as _torque_control
	float _mission_thrust_effort{0.0f};	// PID output (speed control)
	Emaf  _mission_thrust_ema;		// to remove spikes in _mission_thrust_effort before publishing as _thrust_control

	// store computed or manual torque (yaw) and thrust (throttle) here, before finally publishing them in _act_controls:
	float _torque_control {0.0f};
	float _thrust_control {0.0f};

	// from R/C inputs:
	float _torque_control_manual {0.0f};
	float _thrust_control_manual {0.0f};
	float _gas_throttle_manual {0.0f};
	float _cutter_setpoint_manual {-1.0f};
	float _alarm_dev_level_manual {0.0f};
	bool _manual_using_pids{false};
	bool _manual_drive_straight{false};

	// Magnetometer sensor - variables and readings:
	uint32_t _device_id_mag{0};
	uint8_t _mag_calibration_count{0};
	Vector3f _mag_reading3d{};
	Vector2f _mag_reading2d{0.0f, 0.0f};
	float _mag_current_heading{0.0f};	// radians to absolute North, -PI...PI

	// RTK GPS calculated values:
	float _gps_current_heading{0.0f};	// radians to absolute North, -PI...PI
	float _gps_ground_speed_abs{0.0f};	// meters per second

	// EKF2 calculated values and correction:
	float _ekf_current_heading{0.0f};	// radians to absolute North, -PI...PI
	float _ekf_heading_correction{0.0f};	// GND_HEADING_DECL, radians
	float _ekf_ground_speed_abs{0.0f};	// meters per second

	// EKF2 or RTK GPS measurement priority:
	bool _speed_prefer_gps{false};
	bool _heading_prefer_gps{false};
	bool _ekf_override_by_gps{true};

	// some values that we calculate locally to decide on throttling thrust near waypoints:
	float _current_heading{0.0f};		// radians to absolute North, selected between EKF and GPS values above
	float _current_heading_vel{NAN};	// radians to absolute North, derived from _local_pos.vx/vy
	float _heading_error{0.0f};		// radians
	float _heading_error_vel{0.0f};		// radians - from _current_heading_vel above
	float _abbe_error{0.0f};		// meters, heading error at the target point

	// 1.0 at gas throttle 0 (idle), GND_GTL_YAWF_MIN at 1(max gas):
	inline float yaw_responsiveness_factor() { return 1.0f - (1.0f - _param_gas_throttle_yaw_factor_min.get()) * _gas_engine_throttle; };

	float _ground_speed_abs{0.0f};		// meters per second, selected between EKF and GPS values above
	float _ground_speed_ns{0.0f};		// just storing ground_speed_2d.norm_squared() here for L1 desired_r calculation

	// main calculated setpoints:
	float _mission_velocity_setpoint{0.0f}; // target velocity for PID speed control
	float _mission_turning_setpoint{0.0f};  // calculated Yaw effort, derived from L1 acceleration demand or heading error, -1..+1

	// PID speed control related:
	float _x_vel{0.0f};			// measured current velocity for PID speed control from EKF2
	float _x_vel_ema{0.0f};			// meters per second, smoothed _x_vel for PID
	Emaf  _velocity_measured_ema;		// to remove spikes in _x_vel
	Emaf  _velocity_setpoint_ema;		// to remove spikes in _mission_velocity_setpoint

	// Yaw Rate Control input and output for logging:
	float _z_yaw_rate{0.0f};		// measured current yaw rate from EKF2

	// Waypoint navigation variables:
	float _dist_target{NAN};		// meters
	float _leg_distance{NAN};		// meters, distance between previous and current waypoints
	bool _is_short_leg{false};
	float _wp_current_dist{NAN};  		// meters, initialize to very large
	float _wp_previous_dist{NAN};		// meters
	float _wp_next_dist{NAN};		// meters
	float _accel_dist{100.0f};		// meters
	float _decel_dist{100.0f};		// meters
	bool _isSharpTurn{true};		// when turn angle exceeds a threshold, we consider turn sharp
	float _nav_lateral_acceleration_demand{0.0f};	// from _gnd_control, just stored here for tracing
	float _wp_close_enough_rad{0.0f};	// when we consider waypoint reached
	float _acceptance_radius{0.0f};		// if large enough, it will be used for mission advancement to next WP

	// Tools actuators:
	float _gas_engine_throttle{0.0f};	// 0...1 - using INDEX_SPOILERS channel
	float _gas_throttle_servo_position{0.0f};	// gas throttle servo position, 800...2200us - after mixers
	float _cutter_setpoint{0.0f};		// -1...1 - tool like lawnmower blades etc. Using INDEX_FLAPS channel
	float _cutter_servo_position{0.0f};	// cutter servo position, 800...2200us - after mixers
	float _alarm_dev_level{-1.0f};		// horn or other alarm device - using INDEX_AIRBRAKES channel
	float _alarm_servo_position{0.0f};	// second tool servo position, 800...2200us - after mixers

	// For logging:
	float _wheel_left_servo_position{0.0f};	// left wheel servo position, 800...2200us - after mixers
	float _wheel_right_servo_position{0.0f};	// right wheel servo position

	bool _stateHasChanged{false};	// only good inside the loop

	int _tracing_lev{0};			// tracing level, 5-high, 0-none  GND_TRACING_LEV
	int _gps_minfix{0};			// minimal GPS fix when heading may be taken from GPS course-over-ground (cog)

	// timing and tracing helpers:
	hrt_abstime _timestamp{0};
	float _dt{0.0f};			// seconds between calling control loop, usually passed to PIDs

#ifdef DEBUG_MY_PRINT
	hrt_abstime _debug_print_last_called {0};
	hrt_abstime _debug_print1_last_called{0};
	hrt_abstime _debug_print2_last_called{0};
	hrt_abstime _debug_gps_warn_last_called{0};
	bool _printed_idle_trace{false};
	int 	_cnt_run{0};
	int 	_cnt_calc{0};

	void debugPrint();
	void debugPrintAll();
	void debugPrintArriveDepart();
	void debugPrintManual();
#endif // DEBUG_MY_PRINT

#ifdef DEBUG_MY_DATA

	/*
	struct debug_key_value_s _dbg_key;
	orb_advert_t _pub_dbg_key;

	struct debug_value_s _dbg_ind;
	orb_advert_t _pub_dbg_ind;

	struct debug_vect_s _dbg_vect;
	orb_advert_t _pub_dbg_vect;
	*/

	struct debug_array_s _dbg_array;
	orb_advert_t _pub_dbg_array;

	hrt_abstime _debug_data_last_called{0};

	void debugPublishAll();
#endif // DEBUG_MY_DATA

	// end heavy rover additions
	// -----------------------------------------------------------------------------------------------------------------------------

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RD_HEADING_P>) _param_rd_p_gain_heading,
		(ParamFloat<px4::params::RD_HEADING_I>) _param_rd_i_gain_heading,
		(ParamFloat<px4::params::RD_SPEED_P>) _param_rd_p_gain_speed,
		(ParamFloat<px4::params::RD_SPEED_I>) _param_rd_i_gain_speed,
		(ParamFloat<px4::params::RD_MAX_SPEED>) _param_rd_max_speed,
		(ParamFloat<px4::params::NAV_ACC_RAD>) _param_nav_acc_rad,
		(ParamFloat<px4::params::RD_MAX_JERK>) _param_rd_max_jerk,
		(ParamFloat<px4::params::RD_MAX_ACCEL>) _param_rd_max_accel,
		(ParamFloat<px4::params::RD_MISS_SPD_DEF>) _param_rd_miss_spd_def,
		(ParamFloat<px4::params::RD_MAX_YAW_RATE>) _param_rd_max_yaw_rate,
		(ParamFloat<px4::params::RD_TRANS_TRN_DRV>) _param_rd_trans_trn_drv,
		(ParamFloat<px4::params::RD_TRANS_DRV_TRN>) _param_rd_trans_drv_trn,

		// R/C yaw scaler to control right stick horizontal movement effect, avoiding too sensitive feel:
		(ParamFloat<px4::params::RD_MAN_YAW_SCALE>) _param_rd_man_yaw_scale,
		(ParamFloat<px4::params::RD_YAW_RATE_P>) _param_rd_p_gain_yaw_rate,
		(ParamFloat<px4::params::RD_YAW_RATE_I>) _param_rd_i_gain_yaw_rate,
		(ParamInt<px4::params::CA_R_REV>) _param_r_rev,

		(ParamFloat<px4::params::GND_L1_PERIOD>) _param_l1_period,
		(ParamFloat<px4::params::GND_L1_DAMPING>) _param_l1_damping,
		(ParamFloat<px4::params::GND_L1_SCALER>) _param_l1_scaler,

		// PID controller for Line Following (L1 derived):
		(ParamFloat<px4::params::GND_LF_P>) _param_line_following_p,
		(ParamFloat<px4::params::GND_LF_I>) _param_line_following_i,
		(ParamFloat<px4::params::GND_LF_D>) _param_line_following_d,
		(ParamFloat<px4::params::GND_LF_IMAX>) _param_line_following_imax,
		(ParamFloat<px4::params::GND_LF_MAX>) _param_line_following_max,
		(ParamFloat<px4::params::GND_LF_PID_SC>) _param_line_following_pid_output_scaler,

		(ParamFloat<px4::params::GND_LF_WIDTH>) _param_line_following_width,
		(ParamFloat<px4::params::GND_LF_HDG_SC>) _param_line_following_heading_error_scaler,

		// Whether to use Yaw Rate Controller while line following:
		(ParamInt<px4::params::GND_LF_USE_RATE>) _param_lf_use_rates_controller,

		// PID-controlled speed setpoint, m/s:
		(ParamFloat<px4::params::GND_SPEED_SP_EMA>) _param_speed_sp_ema_period,

		(ParamInt<px4::params::GND_SP_MEAS_MODE>) _param_speed_measurement_mode,
		(ParamInt<px4::params::GND_HD_MEAS_MODE>) _param_heading_measurement_mode,
		(ParamInt<px4::params::GND_EKF_OVERRIDE>) _param_ekf_override_by_gps,

		(ParamFloat<px4::params::GND_SPEED_P>) _param_speed_p,
		(ParamFloat<px4::params::GND_SPEED_I>) _param_speed_i,
		(ParamFloat<px4::params::GND_SPEED_D>) _param_speed_d,
		(ParamFloat<px4::params::GND_SPEED_IMAX>) _param_speed_imax,
		(ParamFloat<px4::params::GND_SPEED_MAX>) _param_speed_max,

		(ParamFloat<px4::params::GND_THR_MIN>) _param_thrust_min,
		(ParamFloat<px4::params::GND_THR_MAX>) _param_thrust_max,

		// added to support heavy differential drive rover (lawnmower):

		// Acceleration (wp departure) and deceleration (wp arrival) tuning:
		(ParamFloat<px4::params::GND_ACCEL_DIST>) _param_accel_dist,
		(ParamFloat<px4::params::GND_DECEL_DIST>) _param_decel_dist,
		(ParamFloat<px4::params::GND_SPD_ARR_MIN>) _param_speed_decel_min_factor,
		(ParamFloat<px4::params::GND_SPD_DEP_MIN>) _param_speed_accel_min_factor,

		(ParamFloat<px4::params::GND_WP_PRECISN>) _param_wp_precision,
		(ParamFloat<px4::params::GND_STOP_PRECISN>) _param_stop_precision,

		(ParamFloat<px4::params::GND_TURN_SPEED>) _param_turn_speed,
		(ParamFloat<px4::params::GND_TURN_PRECISN>) _param_turn_precision,
		(ParamFloat<px4::params::GND_TURN_WAIT>) _param_turn_wait,
		(ParamFloat<px4::params::GND_TURN_RATE>) _param_turn_rate_sp,

		(ParamFloat<px4::params::GND_HEADING_DECL>) _param_heading_err_decl,

		(ParamFloat<px4::params::GND_RATE_P>) _param_rate_p,
		(ParamFloat<px4::params::GND_RATE_I>) _param_rate_i,
		(ParamFloat<px4::params::GND_RATE_D>) _param_rate_d,
		(ParamFloat<px4::params::GND_RATE_FF>) _param_rate_ff,
		(ParamFloat<px4::params::GND_RATE_IMAX>) _param_rate_imax,
		(ParamFloat<px4::params::GND_RATE_IMINSPD>) _param_rate_i_minspeed,
		(ParamFloat<px4::params::GND_RATE_AD_TRIM>) _param_rate_depart_arrive_trim,
		(ParamFloat<px4::params::GND_RATE_AD_SC>) _param_heading_ad_rate_scaler,

		(ParamFloat<px4::params::GND_THRUST_SC>) _param_gnd_thrust_scaler,
		(ParamFloat<px4::params::GND_TORQUE_SC>) _param_gnd_torque_scaler,

		(ParamInt<px4::params::GND_TRACING_LEV>) _param_tracing_lev,
		(ParamInt<px4::params::GND_GPS_MINFIX>) _param_gps_minfix,
		(ParamInt<px4::params::GND_EMA_M_PERIOD>) _param_measurements_ema_period,
		(ParamInt<px4::params::GND_EMA_O_PERIOD>) _param_outputs_ema_period,

		// For debugging Rate Conrol PID in manual mode:
		(ParamInt<px4::params::GND_MAN_USE_PID>) _param_manual_use_pid,
		(ParamInt<px4::params::GND_MAN_STRAIGHT>) _param_manual_drive_straight,

		// Gas engine throttle in different states:
		(ParamFloat<px4::params::GND_GTL_IDLE>) _param_gas_throttle_idle,
		(ParamFloat<px4::params::GND_GTL_DEPART>) _param_gas_throttle_departing,
		(ParamFloat<px4::params::GND_GTL_TURN>) _param_gas_throttle_turning,
		(ParamFloat<px4::params::GND_GTL_ARRIVE>) _param_gas_throttle_arriving,
		(ParamFloat<px4::params::GND_GTL_STRAIGHT>) _param_gas_throttle_straight,
		(ParamFloat<px4::params::GND_GTL_YAWF_MIN>) _param_gas_throttle_yaw_factor_min
	)

	void		poll_everything();
	void		position_setpoint_triplet_poll();
	void		attitude_setpoint_poll();
	void		rates_setpoint_poll();
	void		vehicle_control_mode_poll();
	void 		vehicle_attitude_poll();
	void		manual_control_setpoint_poll();
	void		gps_poll();
	void		magnetometer_poll();
	void		mission_poll();
	void		mission_result_poll();
	void		navigator_mission_item_poll();
	void		vehicle_status_poll();
	void		actuator_outputs_poll();

	/**
	 * Control position.
	 */
	void		update_orientation();
	void		control_position_manual();
	void		control_position(const Vector2d &global_pos);
	float		control_yaw_rate();

};
