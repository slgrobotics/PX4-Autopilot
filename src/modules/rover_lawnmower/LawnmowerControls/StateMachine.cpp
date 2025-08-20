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

void LawnmowerControl::workStateMachine()
{
	_stateHasChanged = false;

	const POS_CTRLSTATES pos_ctrl_state_prev = _pos_ctrl_state;

	// clear intermediate locally-computed variables:
	_bearings_good = false;
	_bearing_to_curr_wp = NAN;
	_yaw_error = NAN;
	_abbe_error = NAN;
	const float nav_acc_margin = 1.5f; // extra margin for braking before hitting the waypoint bubble

	// === Figure out which set of PID parameters to use: ===
	const float turning_params_threshold = math::max(math::min(_param_lm_accel_dist.get(),
					       _param_lm_decel_dist.get()) * 0.8f + _param_nav_acc_rad.get(), 1.0f);
	const bool current_wp_is_close = PX4_ISFINITE(_wp_current_dist) && _wp_current_dist < turning_params_threshold;
	const bool previous_wp_is_close = PX4_ISFINITE(_wp_previous_dist) && _wp_previous_dist < turning_params_threshold;

	if ((current_wp_is_close || previous_wp_is_close) && !_is_flyby_wp) {

		// Close to either current or previous waypoint, use spot turning PIDs parameters:
		adjustRateParams(true);

	} else {
		// Far from both waypoints, we can use normal driving PIDs:
		adjustRateParams(false);
	}

	// =========================================================

	switch (_pos_ctrl_state) {
	case POS_STATE_NONE:				// "wound down" undefined/invalid state, no need controlling anything
		break;

	case POS_STATE_IDLE:				// idle state, no need controlling anything yet, initiate starting the mission

		setStateMachineState(POS_STATE_MISSION_START);

		break;

	case STRAIGHT_RUN: {				// target waypoint is far away, we can use Pursuit and cruise speed

			_bearings_good = updateBearings();

			if (!_bearings_good || !PX4_ISFINITE(_wp_current_dist) || !PX4_ISFINITE(_wp_previous_dist)) {
				// Bearings or distances are not good, force transition to arriving state:
				setStateMachineState(WP_ARRIVING);
				cte_end();
				break;
			}

			const bool is_close_to_wp = _wp_current_dist < _decel_dist; // LM_DECEL_DIST + NAV_ACC_RAD

			const bool is_arriving = is_close_to_wp && !_is_flyby_wp;

			if (is_arriving) {
				// Close enough to destination waypoint, switch from Pursuit to direct heading:
#ifdef DEBUG_MY_PRINT
				PX4_INFO("OK: STRAIGHT_RUN got close %.2f, switching to WP_ARRIVING", (double)_wp_current_dist);
				PX4_INFO("is_flyby_wp: %s close_to_wp %s  _rover_speed_setpoint_abs: %.2f m/s",
					 _is_flyby_wp ? "true" : "false", is_close_to_wp ? "true" : "false", (double)_rover_speed_setpoint_abs);
#endif // DEBUG_MY_PRINT

				setStateMachineState(WP_ARRIVING);
				cte_end();

			} else {
				// Normal straight run, line following.
				if (_wp_current_dist > _param_lm_stats_dist.get()
				    && _wp_previous_dist > _param_lm_stats_dist.get()) {
					// We are far from both waypoints, we can accumulate crosstrack error statistics:
					cte_compute();
				}
			}

		} break;

	case WP_ARRIVING: {				// target waypoint is close, we need to slow down and head straight to it till stop

			_bearings_good = updateBearings();

			if (!_bearings_good || !PX4_ISFINITE(_wp_current_dist) || !PX4_ISFINITE(_wp_previous_dist)) {
				// Bearings or distances are not good, force transition to stopping state:
				setStateMachineState(WP_STOPPING);
				break;
			}

			float stopping_start_dist = math::min(_param_nav_acc_rad.get() * nav_acc_margin,
							      _param_nav_acc_rad.get() + 0.5f); // NAV_ACC_RAD

			if (!_is_flyby_wp && _wp_current_dist < stopping_start_dist) {

#ifdef DEBUG_MY_PRINT
				PX4_INFO("OK: WP_ARRIVING got close, switching to WP_STOPPING");
				PX4_INFO("_wp_current_dist: %.2f <> %.2f m  _rover_speed_setpoint_abs: %.2f m/s",
					 (double)_wp_current_dist, (double)stopping_start_dist, (double)_rover_speed_setpoint_abs);
#endif // DEBUG_MY_PRINT
				// DifferentialSpeedControl has switched from DRIVING to SPOT_TURNING, we try mirroring that.
				// We are also closer than NAV_ACC_RAD radius to waypoint (with 1.2x margin), begin stopping phase:
				//adjustRateParams(true); // adjust yaw PIDs for spot turning
				setStateMachineState(WP_STOPPING);
#ifdef DEBUG_MY_PRINT
				PX4_WARN("WP_STOPPING : vel: ekf: %.2f  gps: %.2f m/s  curr dist: %.2f m",
					 (double)_location_metrics.ekf_x_vel, (double)_location_metrics.gps_vel_m_s, (double)_wp_current_dist);
#endif // DEBUG_MY_PRINT
			}

#ifdef DEBUG_MY_PRINT
			debugPrintArriveDepart();
#endif // DEBUG_MY_PRINT
		}
		break;

	case WP_STOPPING:			// we hit the waypoint's bubble and need to stop

		_bearings_good = updateBearings();

		if (!_bearings_good
		    || !PX4_ISFINITE(_wp_current_dist) // Bearings or target distance not good
		    || _wp_current_dist > _param_nav_acc_rad.get() * nav_acc_margin // we are told to depart from the waypoint
		   ) {
#ifdef DEBUG_MY_PRINT
			PX4_WARN("WP_STOPPING : force transition to arrived state: curr dist: %.2f m",
				 (double)_wp_current_dist);
#endif // DEBUG_MY_PRINT
			// force transition to arrived state:
			setStateMachineState(WP_ARRIVED);
			break;
		}

		// we need to monitor velocity here, and if it is below a threshold, we can switch to WP_ARRIVED state:

#ifdef DEBUG_MY_PRINT
		// PX4_WARN("WP_STOPPING : vel: ekf: %.2f  gps: %.2f m/s  curr dist: %.2f m",
		// 	 (double)_location_metrics.ekf_x_vel, (double)_location_metrics.gps_vel_m_s, (double)_wp_current_dist);
#endif // DEBUG_MY_PRINT

		if (_is_flyby_wp
		    || abs(_location_metrics.ekf_x_vel) < 0.05f) {
#ifdef DEBUG_MY_PRINT
			PX4_WARN("WP_STOPPING : vel: ekf: %.2f  gps: %.2f m/s  curr dist: %.2f m",
				 (double)_location_metrics.ekf_x_vel, (double)_location_metrics.gps_vel_m_s, (double)_wp_current_dist);
			PX4_WARN("WP_STOPPING : ARRIVED : is_flyby_wp: %s  isSpotTurning: %s`",
				 _is_flyby_wp ? "true" : "false", _isSpotTurning ? "true" : "false");
#endif // DEBUG_MY_PRINT
			setStateMachineState(WP_ARRIVED);
		}

		break;

	case WP_ARRIVED:				// reached waypoint, stopped. Or already left it behind, departing

		_bearings_good = updateBearings();

		// See if that was the last waypoint of the mission:
		if (!_bearings_good
		    || !_pos_sp_triplet.current.valid
		    || _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
			// We have arrived to the last waypoint of the mission, switch to end of mission
			setStateMachineState(POS_STATE_MISSION_END);

		} else {
			// We have arrived to a waypoint, but there are more waypoints in the triplet,
			// so we need to turn towards the next waypoint:
			setStateMachineState(WP_TURNING);
		}

		break;

	case WP_TURNING:				// we need to turn in place to the next waypoint

		_accel_dist = _param_lm_accel_dist.get();	// LM_ACCEL_DIST (can be 0 to skip Departure phase)
		_decel_dist = _param_lm_decel_dist.get() + _param_nav_acc_rad.get();	// LM_DECEL_DIST + NAV_ACC_RAD

		_bearings_good = updateBearings();

#ifdef DEBUG_MY_PRINT
		// PX4_WARN("WP_TURNING : vel: %.2f / %.2f m/s  curr dist: %.2f m  yaw err: %.1f deg",
		// 	 (double)_location_metrics.ekf_x_vel, (double)_location_metrics.gps_vel_m_s, (double)_wp_current_dist,
		// 	 (double)math::degrees(_yaw_error));
#endif // DEBUG_MY_PRINT

		if (!_bearings_good) {
			// Bearings are not good, we cannot turn to the next waypoint:
			PX4_ERR("WP_TURNING: bearings not good, cannot turn to next waypoint");
			setStateMachineState(POS_STATE_MISSION_END);
			break;
		}

		if (!_isSpotTurning
		    && fabsf(_yaw_error) < _param_rd_trans_trn_drv.get()
		    && fabsf(_bearing_error) < _param_rd_trans_trn_drv.get()
		   ) {
			// DifferentialSpeedControl has switched from SPOT_TURNING to DRIVING, we try mirroring that.

			_accel_start = _curr_pos_ned; // remember where we started accelerating from.

			setStateMachineState(_accel_dist > FLT_EPSILON ? WP_DEPARTING : STRAIGHT_RUN);
		}

		if (_wp_previous_dist > _accel_dist) {
			// we are stuck in the TURNING state, force transition to DEPARTING:
			_accel_start = _prev_wp_ned; // guess where we started accelerating from.

			setStateMachineState(_accel_dist > FLT_EPSILON ? WP_DEPARTING : STRAIGHT_RUN);
		}

		break;

	case WP_DEPARTING: {				// we turned to next waypoint and must start accelerating

			cte_begin(); // just invalidate _crosstrack_error_avg while departing to avoid confusion

			_bearings_good = updateBearings();

			if (!_bearings_good) {
				// Bearings are not good, we cannot depart:
				PX4_ERR("WP_DEPARTING: bearings not good, cannot depart");
				setStateMachineState(POS_STATE_MISSION_END);
				break;
			}

			float from_accel_start = (_curr_pos_ned -
						  _accel_start).norm(); // meters, how far we are from the point we started accelerating from

			// PX4_INFO("WP_DEPARTING: from_accel_start: %.2f m  _accel_dist: %.2f m",
			// 	 (double)from_accel_start, (double)_accel_dist);

			// turn on tools (cutting deck) - we are on the business part of the mission:
			_cutter_setpoint = ACTUATOR_ON;

			if (from_accel_start > _accel_dist) {
				// we are far enough from departure waypoint and not heading to the first waypoint, switch to Pursuit:
				setStateMachineState(STRAIGHT_RUN);
			}

#ifdef DEBUG_MY_PRINT
			debugPrintArriveDepart();
#endif // DEBUG_MY_PRINT
		}
		break;

	case POS_STATE_MISSION_START:			// turn on what we need for the mission (lights, gas engine throttle, blades)

#ifdef DEBUG_MY_PRINT
		PX4_INFO("Mission started - turn on what we need for the mission (lights, gas engine throttle, blades)");
#endif // DEBUG_MY_PRINT

		// First waypoint of the mission has arrived, go to it. First we need to turn towards it:
		//adjustRateParams(true); //  // adjust yaw PIDs for spot turning
		setStateMachineState(WP_TURNING);

		cte_begin_mission();

		break;

	case POS_STATE_MISSION_END:			// turn off what we needed for the mission at the end or error

#ifdef DEBUG_MY_PRINT
		PX4_INFO("Mission ended - turning off what we needed for the mission");
#endif // DEBUG_MY_PRINT

		//if (_isTurningPids) {
		//	adjustRateParams(false); // reset to straight run PIDs
		//}

		debugPrintCrosstrackStats();

		setStateMachineState(POS_STATE_NONE); // just rest at the end of the mission

		cte_end_mission();

		PX4_WARN("Mission end: mission crosstrack error:  avg: %.1f cm  max: %.1f cm  outside: %i",
			 (double)(_crosstrack_error_mission_avg * 100.0f),
			 (double)(_crosstrack_error_mission_max * 100.0f),
			 _cte_seconds_outside);

		break;

	default:
		PX4_ERR("Unknown Rover State");
		setStateMachineState(POS_STATE_NONE);
		break;
	}


	adjustActuatorSetpoints();

	if (pos_ctrl_state_prev != _pos_ctrl_state) {
#ifdef DEBUG_MY_PRINT
		PX4_INFO("FYI: state changed:  %s  -->  %s", control_state_name(pos_ctrl_state_prev),
			 control_state_name(_pos_ctrl_state));
#endif // DEBUG_MY_PRINT
		_stateHasChanged = true;
	}

}

void LawnmowerControl::unwindStateMachine()
{
	setStateMachineState(POS_STATE_MISSION_END);

	workStateMachine();	// make sure we set the actuators to "off" state
}

bool LawnmowerControl::updateBearings()
{
	// get the direction errors between the current position and "current"" (target) waypoint:

	const Vector2f curr_pos_to_curr_wp = _curr_wp_ned - _curr_pos_ned;
	_bearing_to_curr_wp = wrap_2pi(atan2f(curr_pos_to_curr_wp(1), curr_pos_to_curr_wp(0)));

	// for now, assume we want to turn to the target waypoint. Pursuit may correct that.
	// don't touch double wrap! TODO - why?
	//_yaw_error = wrap_pi(_bearing_to_curr_wp - wrap_pi(_vehicle_yaw)); // where the robot wants to turn
	_yaw_error = wrap_pi(_bearing_to_curr_wp - _vehicle_yaw); // where the robot wants to turn

	// ~28.6 degrees deviation makes sense, NAN for more:
	_abbe_error = abs(_yaw_error) < 0.5f ? _wp_current_dist * sin(_yaw_error) : NAN; // meters at target point

	const bool ret = PX4_ISFINITE(_yaw_error);

	if (!ret) {
		PX4_WARN("updateBearings(): bearings no good");
	}

	return ret;
}

void LawnmowerControl::adjustRateParams(bool setSpotTurningPids)
{
	if (setSpotTurningPids == _isTurningPids) {
		// no need to change PIDs settings, they are already set for the current state:
		return;
	}

	// locate parameters for yaw rate control we want to override:
	param_t p_yaw_rate_p = param_find("RO_YAW_RATE_P");
	param_t p_yaw_rate_i = param_find("RO_YAW_RATE_I");
	param_t p_yaw_rate_lim = param_find("RO_YAW_RATE_LIM");
	param_t p_yaw_p = param_find("RO_YAW_P");

	if (setSpotTurningPids) {
		float new_yaw_rate_p = _param_lm_yaw_rate_t_p.get();

		if (new_yaw_rate_p > FLT_EPSILON) {
			float new_yaw_rate_i = _param_lm_yaw_rate_t_i.get();
			float new_yaw_rate_lim = _param_lm_yaw_rate_t_lim.get();
			float new_yaw_p = _param_lm_yaw_t_p.get();
#ifdef DEBUG_MY_PRINT
			PX4_WARN("Turning PIDs: YAW_RATE: P: %.3f  I: %.4f  LIM: %.1f  YAW_P: %.3f",
				 (double)new_yaw_rate_p, (double)new_yaw_rate_i, (double)new_yaw_rate_lim, (double)new_yaw_p);
#endif // DEBUG_MY_PRINT
			param_set(p_yaw_rate_p, &new_yaw_rate_p);
			param_set(p_yaw_rate_i, &new_yaw_rate_i);
			param_set(p_yaw_rate_lim, &new_yaw_rate_lim);
			param_set(p_yaw_p, &new_yaw_p);
		}

		_isTurningPids = true;

	} else {
		float new_yaw_rate_p = _param_lm_yaw_rate_p.get();

		if (new_yaw_rate_p > FLT_EPSILON) {
			float new_yaw_rate_i = _param_lm_yaw_rate_i.get();
			float new_yaw_rate_lim = _param_lm_yaw_rate_lim.get();
			float new_yaw_p = _param_lm_yaw_p.get();
#ifdef DEBUG_MY_PRINT
			PX4_WARN("Normal PIDs: YAW_RATE: P: %.3f  I: %.4f  LIM: %.1f  YAW_P: %.3f",
				 (double)new_yaw_rate_p, (double)new_yaw_rate_i, (double)new_yaw_rate_lim, (double)new_yaw_p);
#endif // DEBUG_MY_PRINT
			param_set(p_yaw_rate_p, &new_yaw_rate_p);
			param_set(p_yaw_rate_i, &new_yaw_rate_i);
			param_set(p_yaw_rate_lim, &new_yaw_rate_lim);
			param_set(p_yaw_p, &new_yaw_p);
		}

		_isTurningPids = false;
	}

	//param_notify_changes();
}

void LawnmowerControl::adjustActuatorSetpoints()
{
	switch (_pos_ctrl_state) {
	case POS_STATE_NONE:	// undefined/invalid state, no need to control anything
	case POS_STATE_IDLE:	// idle state, just make sure we stay put.

		_ice_throttle_setpoint = _param_ice_throttle_idle.get();	// LM_ICE_IDLE *0.0
		_cutter_setpoint = ACTUATOR_OFF;
		_alarm_dev_level = ACTUATOR_OFF;

		break;

	case WP_ARRIVING:		// target waypoint is close, we need to slow down and head straight to it till stop

		_ice_throttle_setpoint = _param_ice_throttle_arriving.get();	// LM_ICE_ARRIVE *0.8

		break;

	case WP_ARRIVED:		// reached waypoint. Make sure mission knows about it

		_ice_throttle_setpoint = _param_ice_throttle_arriving.get();	// LM_ICE_ARRIVE *0.8

		break;

	case WP_TURNING:		// we need to turn in place towards the next waypoint

		_ice_throttle_setpoint = _param_ice_throttle_turning.get();	// LM_ICE_TURN *0.2

		break;

	case WP_DEPARTING:		// we turned to next waypoint and must start accelerating

		_ice_throttle_setpoint = _param_ice_throttle_departing.get();	// LM_ICE_DEPART *0.8
		//_cutter_setpoint = ACTUATOR_ON;

		break;

	case STRAIGHT_RUN: 	// target waypoint is far away, we can use Pursuit and cruise speed

		_ice_throttle_setpoint = _param_ice_throttle_straight.get();	// LM_ICE_STRAIGHT *1.0

		break;

	case WP_STOPPING: 		// we hit a waypoint and need to stop before we declare "we arrived"

		_ice_throttle_setpoint = _param_ice_throttle_idle.get();	// LM_ICE_IDLE *0.0

		break;

	case POS_STATE_MISSION_START:	// turn on what we need for the mission (lights, gas engine throttle, blades)

		_ice_throttle_setpoint = _param_ice_throttle_idle.get();	// LM_ICE_IDLE *0.0
		_cutter_setpoint = ACTUATOR_OFF;			// keep the tools off until we start departing from the first waypoint of the mission
		_alarm_dev_level = ACTUATOR_OFF;

		break;

	case POS_STATE_MISSION_END:		// turn off what we needed for the mission at the end or error

		_ice_throttle_setpoint = _param_ice_throttle_idle.get();	// LM_ICE_IDLE *0.0
		_cutter_setpoint = ACTUATOR_OFF;
		_alarm_dev_level = ACTUATOR_OFF;

		break;

	default:
		break;
	}
}

void LawnmowerControl::setStateMachineState(const POS_CTRLSTATES desiredState)
{
#ifdef DEBUG_MY_PRINT
	PX4_INFO("FYI: setting new state:  %s  -->  %s", control_state_name(_pos_ctrl_state), control_state_name(desiredState));
#endif // DEBUG_MY_PRINT

	/*
	switch (desiredState) {
	case STRAIGHT_RUN:
		// make small adjustments more effective on straight lines:
		_rate_control.setFeedForwardGain(matrix::Vector3f(0.0f, 0.0f, _param_rate_ff.get()));
		break;

	default:
		//_rate_control.setFeedForwardGain(matrix::Vector3f(0.0f, 0.0f, _param_rate_ff.get() / 50.0f));
		_rate_control.setFeedForwardGain(matrix::Vector3f(0.0f, 0.0f, 0.0f));
		break;
	}
	*/

	_pos_ctrl_state = desiredState;
}

} // namespace rover_lawnmower

