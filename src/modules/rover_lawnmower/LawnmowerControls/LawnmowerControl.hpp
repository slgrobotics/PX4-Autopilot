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
#define PUBLISH_ADSB

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
#include <uORB/topics/rover_velocity_setpoint.h>
#include <uORB/topics/pure_pursuit_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/sensor_gps.h>
#if (defined(DEBUG_MY_PRINT) || defined(DEBUG_MY_DATA)) && !defined(CONFIG_ARCH_BOARD_PX4_SITL)
#include <uORB/topics/actuator_outputs.h>
#endif // (defined(DEBUG_MY_PRINT) || defined(DEBUG_MY_DATA)) && !defined(CONFIG_ARCH_BOARD_PX4_SITL)
#include <uORB/topics/actuator_servos.h>
#ifdef PUBLISH_ADSB
#include <uORB/topics/transponder_report.h>
#endif // PUBLISH_ADSB

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

/**
 * @brief Class LocationMetrics holds all location-related metrics for the rover/lawnmower control.
 * It includes EKF2 and RTK GPS data, as well as
 * flags indicating the validity of the data and preferences for speed and heading control.
 * Used for tracing and debugging purposes.
 */
class LocationMetrics
{
public:
	LocationMetrics() = default;
	~LocationMetrics() = default;

	// meters, how far is EKF2 calculated position from GPS reading;
	// bearing from GPS to EKF2 position degrees, -180...180
	Vector2f ekfGpsDeviation{NAN, NAN};

	// RTK GPS raw values from _sensor_gps:
	bool gps_data_valid{false};	// true if GPS data is valid (i.e GPS has at least 3D fix)
	int fix_type{0};		// GPS fix type, 0,1=No fix, 2=2D, 3=3D, 4=DGPS, 5=RTK Float, 6=RTK Fixed see msg/SensorGps.msg
	double gps_lat{0.0};		// latitude in degrees
	double gps_lon{0.0};		// longitude in degrees
	float gps_alt{0.0f};		// altitude in meters above WGS
	float gps_cog_rad{0.0f};	// course over ground, radians to absolute North, -PI...PI
	float gps_vel_m_s{0.0f};	// velocity in meters per second
	float gps_yaw{0.0f};		// gps "heading" - radians to absolute North, -PI...PI

	// EKF2 values from _global_pos:
	double ekf_lat{0.0};		// latitude in degrees
	double ekf_lon{0.0};		// longitude in degrees
	float  ekf_alt{0.0f};		// altitude in meters above WGS

	// EKF values from _vehicle_local_position:
	Vector3<bool> ekf_flags;	// xy_valid, v_xy_valid, heading_good_for_control
	bool ekf_data_good{false};	// AND combo of the above flags
	Vector3f ekf_ground_speed{0, 0, 0};
	Vector2f ekf_ground_speed_2d{0, 0};
	float ekf_ground_speed_abs{0.0f}; // meters per second, a.k.a. _actual_speed
	float ekf_x_vel{0.0f};		// velocity in body frame, m/s, x-axis

	// from _vehicle_attitude:
	float ekf_current_yaw{0.0f};	// radians to absolute North, -PI...PI

	bool ekf_override_by_gps{false}; // true if we use RTK GPS data instead of EKF2 for control algorithms

	void update()
	{
		// meters, how far is EKF2 calculated position from GPS reading:
		ekfGpsDeviation(0) = get_distance_to_next_waypoint(ekf_lat, ekf_lon, gps_lat, gps_lon);
		// bearing from EKF2 to GPS position degrees, 0...360
		// Note: this is the bearing from EKF2 to GPS, not the other:
		ekfGpsDeviation(1) = wrap_2pi(get_bearing_to_next_waypoint(ekf_lat, ekf_lon, gps_lat, gps_lon));
	}

};

#ifdef PUBLISH_ADSB
class adsbData
{
public:
	adsbData() = default;
	~adsbData() = default;

	static constexpr uint8_t ADSB_ALTITUDE_TYPE_PRESSURE_QNH{0}; 	// Barometric altitude, QNH pressure setting
	static constexpr uint8_t ADSB_ALTITUDE_TYPE_GEOMETRIC{1};	// Geometric altitude, reported by GPS

	uint16_t emitter_type{transponder_report_s::ADSB_EMITTER_TYPE_UAV};
	uint16_t squawk{1234}; // Squawk code, 4 digits, 0-4095
	char callsign[9] {"PX4TEST"}; // 8 chars max + null terminator
	uint32_t icao_address{0x123456}; // ICAO address, 24 bits, 0x000000 to 0xFFFFFF
	float tslc{0.01f}; 	  // Time since last communication in seconds

	double lat{0.0};	  // latitude in degrees
	double lon{0.0};	  // longitude in degrees
	float altitude{0.0f};	  // altitude in meters
	uint8_t altitude_type{ADSB_ALTITUDE_TYPE_PRESSURE_QNH};
	float heading{0.0f};	  // Course over ground in radians, 0..2pi, 0 is north
	float hor_velocity{0.0f}; // horizontal velocity in m/s
	float ver_velocity{0.0f}; // vertical velocity in m/s
};

#endif // PUBLISH_ADSB

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
	void updateLawnmowerControl(vehicle_control_mode_s vehicle_control_mode, bool isSpotTurning);

	/**
	 * @brief Reset controller, when switching to a new mission.
	 */
	void reset();

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
	void updateWaypoints();
	void updateWaypointDistances();
	void vehicleControl();
	void workStateMachine();
	void unwindStateMachine();
	bool updateBearings();
	void adjustAcuatorSetpoints();
	void setStateMachineState(const POS_CTRLSTATES desiredState);

#ifdef DEBUG_MY_PRINT
	void debugPrint();
	void debugPrintAuto();
	void debugPrintManual();
	void debugPrintArriveDepart();
	void debugPrintCrosstrackStats();

	hrt_abstime _debug_print_last_called{0};
	hrt_abstime _debug_print_ad_last_called{0};
	bool _printed_none_trace{false};

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
	uORB::Subscription _rover_velocity_setpoint_sub{ORB_ID(rover_velocity_setpoint)};
	uORB::Subscription _pure_pursuit_status_sub{ORB_ID(pure_pursuit_status)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)}; /**< notification of manual control updates */
	uORB::Subscription _sensor_gps_sub{ORB_ID(sensor_gps)};
#if (defined(DEBUG_MY_PRINT) || defined(DEBUG_MY_DATA)) && !defined(CONFIG_ARCH_BOARD_PX4_SITL)
	uORB::Subscription _actuator_outputs_sub {ORB_ID(actuator_outputs)}; /**< actuator outputs subscription for tracing */
#endif // (defined(DEBUG_MY_PRINT) || defined(DEBUG_MY_DATA)) && !defined(CONFIG_ARCH_BOARD_PX4_SITL)

	// uORB publications
	uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};
#ifdef DEBUG_MY_DATA
	uORB::Publication<debug_array_s> _debug_array_pub {ORB_ID(debug_array)};
#endif // DEBUG_MY_DATA

#ifdef PUBLISH_ADSB
	uORB::Publication<transponder_report_s> _transponder_report_pub {ORB_ID(transponder_report)};

	void publishTransponderReport(const adsbData &data);

	hrt_abstime _transponder_report_last_published{0}; // Last time transponder report was published
#endif // PUBLISH_ADSB

	// Variables
	hrt_abstime _timestamp{0}; // Current timestamp
	float _dt{0.f};	// Time since last update in seconds since last call to updateLawnmowerControl()

	bool _stateHasChanged{false};	// only good inside the loop

	bool _isSpotTurning{false}; // true if we are in spot turning state, used to adjust the control logic

	vehicle_attitude_s		_vehicle_attitude{};
	vehicle_control_mode_s 		_vehicle_control_mode{};
	vehicle_global_position_s	_global_pos{};			/**< global vehicle position */
	vehicle_local_position_s	_vehicle_local_position{};
	position_setpoint_triplet_s	_pos_sp_triplet{};		/**< triplet of mission items */
	rover_velocity_setpoint_s 	_rover_velocity_setpoint{};	/**< rover velocity setpoint, bearing and speed */
	pure_pursuit_status_s 		_pure_pursuit_status{};
	manual_control_setpoint_s	_manual_control_setpoint{};	/**< r/c channel data */
	sensor_gps_s			_sensor_gps_data{};		/**< raw gps data */
#if (defined(DEBUG_MY_PRINT) || defined(DEBUG_MY_DATA)) && !defined(CONFIG_ARCH_BOARD_PX4_SITL)
	actuator_outputs_s		_actuator_outputs {};		/**< actuator outputs */
#endif // (defined(DEBUG_MY_PRINT) || defined(DEBUG_MY_DATA)) && !defined(CONFIG_ARCH_BOARD_PX4_SITL)

	MapProjection _global_local_proj_ref{};

	LocationMetrics _location_metrics{};	// RTK GPS and EKF2 metrics

	// Vehicle attitude and position from polling:
	float _vehicle_yaw{NAN};		// radians, vehicle yaw, comes from vehicle_attitude_s
	Vector2f _curr_pos_ned{NAN, NAN};	// local projection of the current position in NED coordinates, meters

	// Some values that we calculate locally to decide on throttling thrust near waypoints and state changes:
	float _bearing_to_curr_wp{NAN};		// radians, bearing to the current waypoint, calculated by updateBearings() 0...2*PI, 0 is North
	float _yaw_error{0.0f};			// radians, yaw error to the current waypoint, Positive - right turn, negative - left turn expected.
	float _abbe_error{0.0f};		// meters, heading error at the target point

	// from _rover_velocity_setpoint, published by DifferentialPosControl:
	float _rover_speed_setpoint{NAN};	// meters per second, speed setpoint for the rover/lawnmower, comes from rover_velocity_setpoint_s
	float _rover_bearing_setpoint{NAN};	// radians, bearing setpoint for the rover

	// These come from PurePursuit:
	float _crosstrack_error{NAN};		// meters, how far we are from the A-B line (A = previous, visited waypoint, B = current waypoint, target)
	float _bearing_error{NAN};		// radians, bearing error to the waypoint, comes from PurePursuit, used when turning in place

	// Threshold distances to the waypoints for state changes:
	float _accel_dist{1.0};			// meters
	float _decel_dist{1.0};			// meters

	// Mission waypoints - from position_setpoint_triplet_s
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
	float _torque_control_manual {NAN};
	float _thrust_control_manual {NAN};
	float _gas_throttle_manual {NAN};
	float _cutter_setpoint_manual {-1.0f};
	float _alarm_dev_level_manual {NAN};
	bool _manual_using_pids{false};
	bool _manual_drive_straight{false};

	// Tools actuators setpoints as produced by State machine:
	float _ice_throttle_setpoint{NAN};	// 0...1 - using PCA9685 channel 4
	float _cutter_setpoint{NAN};		// -1...1 - tool like lawnmower blades etc. Using PCA9685 channel 3
	float _alarm_dev_level{-1.0f};		// horn or other alarm device - using PCA9685 channel 6

	// Tools actuators (servos) actual positions, as polled from actuator_outputs:
	float _ice_throttle_servo_position{NAN}; // gas engine throttle servo position, 800...2200us - after mixers
	float _cutter_servo_position{NAN};	// cutter servo position, 800...2200us - after mixers
	float _alarm_servo_position{NAN};	// second tool servo position, 800...2200us - after mixers

	// Wheels "servos" actual values for logging:
	float _wheel_left_servo_position{NAN};	 // left wheel servo position, 800...2200us - after mixers
	float _wheel_right_servo_position{NAN}; // right wheel servo position

	// --------------------------------
	// Mission crosstrack metrics:
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
	int _cte_seconds_outside{0};

	inline void cte_begin_mission()
	{
		_cte_accum_mission = 0.0f; _cte_count_mission = 0; _cte_seconds_outside = 0;
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
		if (PX4_ISFINITE(_crosstrack_error) && hrt_elapsed_time(&_cte_lf_started) > 5_s) {
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
					++_cte_seconds_outside;
					PX4_WARN("+++++++++++++  outside: %i +++++++++++++", _cte_seconds_outside);
				}
			}
		}
	};

	inline void cte_end()
	{
		_crosstrack_error_avg = _cte_accum / _cte_count;
		_crosstrack_error_mission_avg = _cte_accum_mission / _cte_count_mission; // keep it current for tracing
	};
	// --------------------------------

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::LM_TRACING_LEV>) _param_lm_tracing_lev,
		(ParamInt<px4::params::LM_GPS_MINFIX>) _param_lm_gps_minfix,

		(ParamFloat<px4::params::RD_TRANS_TRN_DRV>) _param_rd_trans_trn_drv, // turn to drive transition threshold, radians
		(ParamFloat<px4::params::RD_TRANS_DRV_TRN>) _param_rd_trans_drv_trn, // drive to turn transition threshold, radians

		(ParamFloat<px4::params::LM_ACCEL_DIST>) _param_lm_accel_dist,	 // meters, distance to accelerate
		(ParamFloat<px4::params::LM_DECEL_DIST>)
		_param_lm_decel_dist,	 // meters, distance to target waypoint to start decelerating
		(ParamFloat<px4::params::NAV_ACC_RAD>) _param_nav_acc_rad, // meters, how close to waypoint we consider it reached

		// Measurement modes - from EKF2 or RTK GPS:
		(ParamInt<px4::params::LM_HD_MEAS_MODE>) _param_lm_hd_meas_mode,
		(ParamInt<px4::params::LM_SP_MEAS_MODE>) _param_lm_sp_meas_mode,
		(ParamInt<px4::params::LM_EKF_OVERRIDE>) _param_lm_ekf_override_by_gps,

		// Gas engine throttle in different states:
		(ParamFloat<px4::params::LM_ICE_IDLE>) _param_ice_throttle_idle,
		(ParamFloat<px4::params::LM_ICE_DEPART>) _param_ice_throttle_departing,
		(ParamFloat<px4::params::LM_ICE_TURN>) _param_ice_throttle_turning,
		(ParamFloat<px4::params::LM_ICE_ARRIVE>) _param_ice_throttle_arriving,
		(ParamFloat<px4::params::LM_ICE_STRAIGHT>) _param_ice_throttle_straight
	)
};

}
