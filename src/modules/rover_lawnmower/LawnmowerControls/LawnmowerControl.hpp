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

#pragma	once

#define DEBUG_MY_PRINT
#define DEBUG_MY_DATA

// PX4 includes
#include <px4_platform_common/module_params.h>

// Library includes
//#include <matrix/matrix/math.hpp>
#include <math.h>
#include <lib/geo/geo.h>

// uORB includes
//#include <uORB/topics/parameter_update.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/pure_pursuit_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_servos.h>

#ifdef DEBUG_MY_DATA
#include <uORB/topics/debug_array.h>
#endif // DEBUG_MY_DATA

using namespace time_literals;
using namespace matrix;

// For binary actuators, connected through PCA9685 "servo relays":
#define ACTUATOR_ON  (1.0f)
#define ACTUATOR_OFF (-1.0f)

namespace rover_lawnmower
{

class LawnmowerControl : public ModuleParams
{
public:
	/**
	 * @brief Constructor for LawnmowerControl.
	 * @param parent The parent ModuleParams object.
	 */
	LawnmowerControl(ModuleParams *parent);
	~LawnmowerControl() = default;

	/**
	 * @brief Perform all operations related to lawnmower control.
	 */
	void updateLawnmowerControl(vehicle_control_mode_s vehicle_control_mode);

protected:
	void updateParams() override;

private:

	enum POS_CTRLSTATES : int {
		POS_STATE_NONE,			// undefined/invalid state, no need controlling anything
		POS_STATE_IDLE,			// idle state, no need controlling anything
		STRAIGHT_RUN,		// target waypoint is far away, we can use Pursuit logic and cruise speed
		WP_ARRIVING,			// target waypoint is close, we need to slow down and head straight to it till stop
		WP_ARRIVED,			// reached waypoint, completely stopped. Make sure mission knows about it
		WP_TURNING,			// we need to turn in place to the next waypoint
		WP_DEPARTING,			// we turned to next waypoint and must start accelerating
		POS_STATE_STOPPING,		// we hit a waypoint and need to stop
		POS_STATE_MISSION_START,	// turn on what we need for the mission (lights, gas engine throttle, blades)
		POS_STATE_MISSION_END		// turn off what we needed for the mission at the end or error
	} _pos_ctrl_state {POS_STATE_NONE};	// Position control state machine

	const char *control_state_name(const POS_CTRLSTATES state);

	void advertisePublishers();
	void updateEkfGpsDeviation();
	void updateWaypoints();
	void updateWaypointDistances();
	void vehicleControl();
	void workStateMachine();
	void unwindStateMachine();
	void adjustAcuatorSetpoints();
	void setStateMachineState(const POS_CTRLSTATES desiredState);

#ifdef DEBUG_MY_PRINT
	void debugPrint();
	void debugPrintAuto();
	void debugPrintManual();
	void debugPrintArriveDepart();

	hrt_abstime _debug_print_last_called{0};
	hrt_abstime _debug_print1_last_called{0};

	int _tracing_lev{0}; // Tracing level, set by parameter
#endif // DEBUG_MY_PRINT

#ifdef DEBUG_MY_DATA
	void publishDebugData();
	void publishAuxActuators();
	void publishDebugArray();

	struct debug_array_s _dbg_array;

	hrt_abstime _debug_data_last_called{0};
#endif // DEBUG_MY_DATA

	/**
	 * @brief Update uORB subscriptions
	 */
	void updateSubscriptions();

	// uORB subscriptions
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _position_setpoint_triplet_sub{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription _pure_pursuit_status_sub{ORB_ID(pure_pursuit_status)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)}; /**< notification of manual control updates */
	uORB::Subscription _sensor_gps_sub{ORB_ID(sensor_gps)};
#if defined(DEBUG_MY_PRINT) || defined(DEBUG_MY_DATA)
	uORB::Subscription _actuator_outputs_sub {ORB_ID(actuator_outputs)}; /**< actuator outputs subscription for tracing */
#endif // defined(DEBUG_MY_PRINT) || defined(DEBUG_MY_DATA)

	// uORB publications
	uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};
#ifdef DEBUG_MY_DATA
	uORB::Publication<debug_array_s> _debug_array_pub {ORB_ID(debug_array)};
#endif // DEBUG_MY_DATA

	// Variables
	hrt_abstime _timestamp{0}; // Current timestamp
	float _dt{0.f};	// Time since last update in seconds since last call to updateLawnmowerControl()

	bool _stateHasChanged{false};	// only good inside the loop

	vehicle_control_mode_s 		_vehicle_control_mode{};
	vehicle_global_position_s	_global_pos{};			/**< global vehicle position */
	vehicle_local_position_s	_vehicle_local_position{};
	position_setpoint_triplet_s	_pos_sp_triplet{};		/**< triplet of mission items */
	pure_pursuit_status_s 		_pure_pursuit_status{};
	manual_control_setpoint_s	_manual_control_setpoint{};	/**< r/c channel data */
	sensor_gps_s			_sensor_gps_data{};		/**< raw gps data */
#if defined(DEBUG_MY_PRINT) || defined(DEBUG_MY_DATA)
	actuator_outputs_s		_actuator_outputs {};		/**< actuator outputs */
#endif // defined(DEBUG_MY_PRINT) || defined(DEBUG_MY_DATA)

	MapProjection _global_local_proj_ref{};


	Vector2d _curr_pos{NAN, NAN};
	Vector2f _curr_pos_ned{NAN, NAN};	// local projection - updated when polling

	float _crosstrack_error{0.0f};		// meters, how far we are from the A-B line (A = previous, visited waypoint, B = current waypoint, target)
	float _ekfGpsDeviation{0.0f};		// meters, how far is EKF2 calculated position from GPS reading
	float _vehicle_yaw{NAN};
	float _accel_dist{100.0f};		// meters
	float _decel_dist{100.0f};		// meters

	// Waypoints - from position_setpoint_triplet_s
	Vector2d _curr_wp{NAN, NAN};
	Vector2f _curr_wp_ned{NAN, NAN};
	Vector2d _prev_wp{NAN, NAN};
	Vector2f _prev_wp_ned{NAN, NAN};
	Vector2d _next_wp{NAN, NAN};

	// from position setpoint triplet:
	float _wp_current_dist{NAN};  		// meters, initialize to very large
	float _wp_previous_dist{NAN};		// meters
	float _wp_next_dist{NAN};		// meters

	// from R/C inputs:
	float _torque_control_manual {0.0f};
	float _thrust_control_manual {0.0f};
	float _gas_throttle_manual {0.0f};
	float _cutter_setpoint_manual {-1.0f};
	float _alarm_dev_level_manual {0.0f};
	bool _manual_using_pids{false};
	bool _manual_drive_straight{false};

	// Tools actuators setpoints as produced by State machine:
	float _gas_engine_throttle{0.0f};	// 0...1 - using INDEX_SPOILERS channel
	float _alarm_dev_level{-1.0f};		// horn or other alarm device - using INDEX_AIRBRAKES channel
	float _cutter_setpoint{0.0f};		// -1...1 - tool like lawnmower blades etc. Using INDEX_FLAPS channel

	// Tools actuators actual positions, as polled from actuator_outputs:
	float _gas_throttle_servo_position{0.0f}; // gas throttle servo position, 800...2200us - after mixers
	float _cutter_servo_position{0.0f};	// cutter servo position, 800...2200us - after mixers
	float _alarm_servo_position{0.0f};	// second tool servo position, 800...2200us - after mixers

	// Wheels "servos" for logging:
	float _wheel_left_servo_position{0.0f};	 // left wheel servo position, 800...2200us - after mixers
	float _wheel_right_servo_position{0.0f}; // right wheel servo position

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

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::LM_TRACING_LEV>) _param_lm_tracing_lev,
		(ParamInt<px4::params::LM_GPS_MINFIX>) _param_lm_gps_minfix,

		(ParamFloat<px4::params::LM_ACCEL_DIST>) _param_lm_accel_dist,	 // meters, distance to accelerate
		(ParamFloat<px4::params::LM_DECEL_DIST>) _param_lm_decel_dist,	 // meters, distance to target waypoint to start decelerating
		(ParamFloat<px4::params::LM_WP_PRECISN>) _param_lm_wp_precision, // meters, how close to waypoint we consider it reached

		// Measurement modes - from EKF2 or RTK GPS:
		(ParamInt<px4::params::LM_HD_MEAS_MODE>) _param_lm_hd_meas_mode,
		(ParamInt<px4::params::LM_SP_MEAS_MODE>) _param_lm_sp_meas_mode,

		// Gas engine throttle in different states:
		(ParamFloat<px4::params::LM_GTL_IDLE>) _param_gas_throttle_idle,
		(ParamFloat<px4::params::LM_GTL_DEPART>) _param_gas_throttle_departing,
		(ParamFloat<px4::params::LM_GTL_TURN>) _param_gas_throttle_turning,
		(ParamFloat<px4::params::LM_GTL_ARRIVE>) _param_gas_throttle_arriving,
		(ParamFloat<px4::params::LM_GTL_STRAIGHT>) _param_gas_throttle_straight
	)
};

}
