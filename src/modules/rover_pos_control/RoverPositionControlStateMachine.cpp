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

//
// perform some state changes, compute _mission_turning_setpoint and _mission_velocity_setpoint
//

void RoverPositionControl::workStateMachine()
{
	_stateHasChanged = false;

	POS_CTRLSTATES pos_ctrl_state_prev = _pos_ctrl_state;

	// clear intermediate L1 variables:
	_nav_bearing = NAN;
	_target_bearing = NAN;
	_crosstrack_error = NAN;
	_heading_error = NAN;
	_nav_lateral_acceleration_demand = NAN;

	// preset outputs. In certain cases (turning or moving must be stopped) NAN is returned:
	_mission_turning_setpoint = NAN;
	_mission_velocity_setpoint = NAN;

	// Mission (navigator) can change target waypoint any time:
	if (checkNewWaypointArrival()) {
		PX4_WARN("**************************************************************************");
		PX4_WARN("A new setpoint arrived, type: %s", waypoint_type_name(_pos_sp_triplet.current.type));

		/*
		 * _pos_sp_triplet.current.type :
			0	- position setpoint, SETPOINT_TYPE_POSITION
			1	- velocity setpoint, SETPOINT_TYPE_VELOCITY
			2	- loiter setpoint, SETPOINT_TYPE_LOITER
			3	- takeoff setpoint, SETPOINT_TYPE_TAKEOFF
			4	- land setpoint, altitude must be ignored, descend until landing, SETPOINT_TYPE_LAND
			5	- do nothing, switch off motors or keep at idle speed (MC), SETPOINT_TYPE_IDLE
		*/

		PX4_INFO("---- %s   current.valid: %s  previous.valid: %s  nav_state: %d",
			 control_state_name(_pos_ctrl_state),
			 _pos_sp_triplet.current.valid ? "Y" : "N",
			 _pos_sp_triplet.previous.valid ? "Y" : "N",
			 _vehicle_status.nav_state);

		PX4_INFO("           mission_result - valid: %s  finished: %s ",
			 _mission_result.valid ? "Y" : "N",
			 _mission_result.finished ? "Y" : "N");

		if (_pos_sp_triplet.current.valid) {
			// we can have a LOITER waypoint arriving (1) on mission end and (2) when "Go to this point" is clicked on the map at any time.
			// _vehicle_status.nav_state will be MAIN_STATE_AUTO_MISSION = 3 at the mission, and MAIN_STATE_AUTO_LOITER = 4 at go-to

			if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) {
				if (_mission_result.valid && !_mission_result.finished && !_pos_sp_triplet.previous.valid) {
					PX4_WARN("A new setpoint arrived, and it is the first one in the mission");
					setStateMachineState(POS_STATE_MISSION_START); // A new waypoint has arrived, and it is the first one in the mission

				} else {
					PX4_WARN("A new setpoint arrived while in mission, turning to it");
					setStateMachineState(WP_TURNING); // A new mission waypoint has arrived - go to it. First we need to turn towards it.
				}

			} else {
				if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER
				    || _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND
				    || _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
					// ground vehicles don't need to circle around last waypoint

					PX4_WARN("A GO-TO setpoint arrived, type: %s - setting state -> MISSION_END",
						 waypoint_type_name(_pos_sp_triplet.current.type));

					// ground vehicles don't need to circle around last waypoint

					PX4_WARN("A MISSION setpoint arrived, type: %s - setting state -> MISSION_END",
						 waypoint_type_name(_pos_sp_triplet.current.type));
					setStateMachineState(POS_STATE_MISSION_END);

				} else {
					PX4_WARN("A GO-TO setpoint arrived, turning to it");

					setStateMachineState(WP_TURNING); // A new "goto" waypoint has arrived - go to it. First we need to turn towards it.
				}
			}
		}
	}

	switch (_pos_ctrl_state) {
	case POS_STATE_NONE:	// undefined/invalid state, no need to control anything
		break;

	case POS_STATE_IDLE:	// idle state, just make sure we stay put.
		break;

	case WP_ARRIVING:		// target waypoint is close, we need to slow down and head straight to it till stop

		if (updateBearings()) {

			// while arriving, we can hit waypoint so closely, that Mission will advance before we get here.
			// in this case a new waypoint will arrive and cause immediate change of state to WP_TURNING
			// and WP_ARRIVED will not be hit. That's normal.

			if (PX4_ISFINITE(_dist_target) && _dist_target < _wp_close_enough_rad) {
#ifdef DEBUG_MY_PRINT
				PX4_INFO("OK: got close, switching to POS_STATE_STOPPING ===========================");
#endif // DEBUG_MY_PRINT
				// We are closer than GND_WP_PRECISN radius to waypoint, begin stopping phase:
				setStateMachineState(POS_STATE_STOPPING);

			} else if (PX4_ISFINITE(_dist_target)
				   && PX4_ISFINITE(_wp_previous_dist)
				   && _dist_target > _decel_dist + 0.2f
				   && fabsf(_heading_error) < 0.1f) {

#ifdef DEBUG_MY_PRINT
				PX4_INFO_RAW("OK: L1 heading recovered, switching to L1_GOTO_WAYPOINT ===========================\n");
				PX4_INFO_RAW("*** trgt_berng: %.2f  curr_hdg: %.2f  hdg_error: %.2f\n",
					     (double) math::degrees(_target_bearing), (double) math::degrees(_current_heading),
					     (double) math::degrees(_heading_error));
				PX4_INFO_RAW("*** dist_trgt: %.2f   _decel_dist: %.2f\n",
					     (double) _dist_target, (double) _decel_dist);
				PX4_INFO_RAW("===================================================================================\n");
#endif // DEBUG_MY_PRINT

				setStateMachineState(L1_GOTO_WAYPOINT);

			} else {
				// See above when we analyzed how close to target wp we are, to switch to POS_STATE_STOPPING
				// Here we just head straight to target wp:

				_mission_turning_setpoint = computeTurningSetpoint();

				setDefaultMissionSpeed();	// will be adusted later by adjustThrustAndYaw()
			}

#ifdef DEBUG_MY_PRINT
			debugPrintArriveDepart();
#endif // DEBUG_MY_PRINT

		} else {
			PX4_WARN("Lost _heading_error while WP_ARRIVING");
			setStateMachineState(POS_STATE_IDLE);	// somehow we lost heading error
		}

		break;

	case WP_ARRIVED:		// reached waypoint. Make sure mission knows about it

		// while arriving, we can hit waypoint so closely, that Mission will advance before we get WP_ARRIVING case.
		// in this case WP_ARRIVED will not be hit. That's normal.

		_acceptance_radius *= 2.0f;  // We hit the radius and decelerated to a stop. Make sure we advance the mission.

		// Leave _mission_turning_setpoint and _mission_velocity_setpoint as NANs

		if (!updateBearings()) {
			PX4_WARN("WP_ARRIVED: Lost _heading_error");
		}

		/*
		SETPOINT_TYPE_POSITION=0	# position setpoint
		SETPOINT_TYPE_VELOCITY=1	# velocity setpoint
		SETPOINT_TYPE_LOITER=2		# loiter setpoint
		SETPOINT_TYPE_TAKEOFF=3		# takeoff setpoint
		SETPOINT_TYPE_LAND=4		# land setpoint, altitude must be ignored, descend until landing
		SETPOINT_TYPE_IDLE=5		# do nothing, switch off motors or keep at idle speed (MC)
		*/

		if (_pos_sp_triplet.current.valid
		    && (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER
			|| _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND
			|| _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_IDLE)) {
			// ground vehicles don't need to circle around last waypoint

			PX4_WARN("In WP_ARRIVED - Current setpoint type: %s - switching state -> MISSION_END",
				 waypoint_type_name(_pos_sp_triplet.current.type));
			setStateMachineState(POS_STATE_MISSION_END);
		}

		break;

	case WP_TURNING: // we need to turn in place towards the next waypoint

		if (updateBearings()) {

			if (math::abs_t(_heading_error) < math::radians(_param_turn_precision.get())) {	// GND_TURN_PRECISN

				// When turning at waypoint, wait several seconds after turn goal is reached.
				// This allows PID oscillations to cease in aggressive turns.

				if (_turn_goal_last_reached == 0) {
					_turn_goal_last_reached = _now;
				}

				// GND_TURN_WAIT
				float turn_wait_sec = _param_turn_wait.get();

				if (turn_wait_sec < FLT_EPSILON || hrt_elapsed_time(&_turn_goal_last_reached) > turn_wait_sec * 1000_ms) {
					_turn_goal_last_reached = 0;
					_wp_dist_at_turn = _wp_current_dist;	// Remember for speed factor calculations

					_accel_dist = _param_accel_dist.get();	// GND_ACCEL_DIST
					_decel_dist = _param_decel_dist.get();	// GND_DECEL_DIST

					if (_wp_dist_at_turn <= _accel_dist + _decel_dist) {
						// short leg, no L1 segment:
						_accel_dist = _decel_dist = _wp_dist_at_turn / 2.0f + 0.2f; // with little extra
					}

					bool is_first_leg = !PX4_ISFINITE(_wp_previous_dist) && PX4_ISFINITE(_wp_current_dist);

					if (is_first_leg) {
						// We are departing from landing point towards the first waypoint.

						PX4_WARN("WP_TURNING - first leg");

						setStateMachineState(WP_ARRIVING); // Can't use L1 on the first leg

					} else {
						setStateMachineState(WP_DEPARTING);		// turned towards next waypoint, can depart now
					}

					resetVelocitySmoothing(); // trajectory computation starts here
				}

			} else {
				_mission_turning_setpoint = computeTurningSetpoint();
				const float turn_speed = _param_turn_speed.get();	// GND_TURN_SPEED
				_mission_velocity_setpoint = abs(turn_speed) < FLT_EPSILON ? NAN : turn_speed;

#ifdef DEBUG_MY_PRINT
				debugPrintArriveDepart();
#endif // DEBUG_MY_PRINT
			}

		} else {
			PX4_WARN("Lost _heading_error while WP_TURNING");
			setStateMachineState(POS_STATE_IDLE);	// somehow in turning we lost heading error
		}

		break;

	case WP_DEPARTING:		// we turned to first/next waypoint and must start accelerating

		if (updateBearings()) {

			if (_wp_previous_dist < _accel_dist) {
				_mission_turning_setpoint = computeTurningSetpoint();

				setDefaultMissionSpeed();	// will be adusted later by adjustThrustAndYaw()

				// we can turn on tools (cutting deck) as we know the previous waypoint exists, i.e. we are on the business part of the mission:
				_cutter_setpoint = ACTUATOR_ON;

#ifdef DEBUG_MY_PRINT
				debugPrintArriveDepart();
#endif // DEBUG_MY_PRINT

			} else {
				// we are far enough from departure waypoint and not heading to the first waypoint, switch to L1:
				setStateMachineState(L1_GOTO_WAYPOINT);
			}

		} else {
			PX4_WARN("Lost _heading_error while WP_DEPARTING");
			setStateMachineState(POS_STATE_IDLE);	// somehow we lost heading error
		}

		break;

	case L1_GOTO_WAYPOINT: {	// target waypoint is far away, we can use L1 and cruise speed

			bool is_arriving = PX4_ISFINITE(_wp_current_dist) ? _wp_current_dist < _decel_dist :
					   false;		// GND_DECEL_DIST or half leg

			if (is_arriving) {
				// Close enough to destination waypoint, switch from L1 to direct heading:
				setStateMachineState(WP_ARRIVING);

			} else {

				if (updateBearings()) {

					if (fabsf(_heading_error) > 1.0f && PX4_ISFINITE(_wp_previous_dist) && _wp_previous_dist > _accel_dist * 1.5f) {
#ifdef DEBUG_MY_PRINT
						PX4_INFO("==== Overshot: aceptnce_rad: %.2f  hdng_err: %.2f", (double)_acceptance_radius,
							 (double)math::degrees(_heading_error));

						debugPrintAll();
#endif // DEBUG_MY_PRINT
						// we are so much off course, or overshot the waypoint, that we need to switch to non-L1 algorithm:
						setStateMachineState(WP_ARRIVING);
						_mission_turning_setpoint = 0.0f;
						setDefaultMissionSpeed();	// will be adusted later by adjustThrustAndYaw()

					} else {

						// We are far from destination and more or less are pointed in its direction.

						navigate_L1();	// compute all L1 variables

						_mission_turning_setpoint = computeTurningSetpoint();

						setDefaultMissionSpeed();	// will be adusted later by adjustThrustAndYaw()
					}

				} else {
					PX4_WARN("Lost _heading_error while in L1_GOTO_WAYPOINT");
					setStateMachineState(POS_STATE_IDLE);	// somehow we lost heading error
				}
			}

		}
		break;

	case POS_STATE_STOPPING: 		// we hit a waypoint and need to stop before we declare "we arrived"

		// We are required to stop, watch the speed decreasing.

		// Leave _mission_turning_setpoint and _mission_velocity_setpoint as NANs

#ifdef DEBUG_MY_PRINT

		if (_tracing_lev > 4) {
			PX4_INFO("STOPPING: _ground_speed_abs: %.4f   _x_vel: %.4f   _dist_target: %.4f", (double)_ground_speed_abs,
				 (double)_x_vel, (double)_dist_target);
		}

#endif // DEBUG_MY_PRINT

		if (PX4_ISFINITE(_x_vel)) {

			if (fabsf(_x_vel) < _param_stop_precision.get())	// GND_STOP_PRECISN
				// if(_x_vel < _param_stop_precision.get())	// GND_STOP_PRECISN
				// if(_ground_speed_abs < _param_stop_precision.get())		- can't do this, as _x_vel quickly turns negative and abs starts to grow
			{
				if (_mission_result.valid && _mission_result.finished) {
					setStateMachineState(POS_STATE_MISSION_END); // just rest at the end of the mission

				} else {
					setStateMachineState(WP_ARRIVED);	// arrived, can turn to next waypoint
				}
			}

		} else {
			PX4_WARN("Lost x_vel while POS_STATE_STOPPING");
			setStateMachineState(POS_STATE_IDLE);	// somehow in stopping we lost speed measurement
		}

		break;

	case POS_STATE_MISSION_START:	// turn on what we need for the mission (lights, gas engine throttle, blades)

#ifdef DEBUG_MY_PRINT
		PX4_INFO("Mission started - turn on what we need for the mission (lights, gas engine throttle, blades)");
#endif // DEBUG_MY_PRINT

		// First waypoint of the mission has arrived, go to it. First we need to turn towards it:
		setStateMachineState(WP_TURNING);
		break;

	case POS_STATE_MISSION_END:		// turn off what we needed for the mission at the end or error

#ifdef DEBUG_MY_PRINT
		PX4_INFO("Mission ended - turn off what we needed for the mission");
#endif // DEBUG_MY_PRINT

		setStateMachineState(POS_STATE_IDLE); // just rest at the end of the mission

		break;

	default:
		PX4_ERR("Unknown Rover State");
		setStateMachineState(POS_STATE_NONE);
		break;
	}

	adjustAcuatorSetpoints();

	if (pos_ctrl_state_prev != _pos_ctrl_state) {
#ifdef DEBUG_MY_PRINT
		PX4_INFO("FYI: state changed:  %s  -->  %s", control_state_name(pos_ctrl_state_prev),
			 control_state_name(_pos_ctrl_state));
#endif // DEBUG_MY_PRINT
		_stateHasChanged = true;

		resetTorqueControls();
		resetThrustControls();
	}

}

void RoverPositionControl::navigate_L1()
{
	// new version of L1 operates on float vectors:
	Vector2f curr_pos_local{_local_pos.x, _local_pos.y};
	Vector2f curr_wp_local = _global_local_proj_ref.project(_curr_wp(0), _curr_wp(1));
	Vector2f prev_wp_local = _global_local_proj_ref.project(_prev_wp(0), _prev_wp(1));

	//PX4_INFO_RAW("LOC:  (%f,%f) A: (%f,%f)  B: (%f,%f)\n", (double) curr_pos_local(0), (double) curr_pos_local(1), (double) prev_wp_local(0), (double) prev_wp_local(1), (double) curr_wp_local(0), (double) curr_wp_local(1));

	//PX4_INFO_RAW("Speed: %.6f   %.6f\n", (double) _ground_speed_2d(0), (double) _ground_speed_2d(1));

	// compute yaw (lateral) acceleration demand using L1 control:

	_gnd_control.navigate_waypoints(prev_wp_local, curr_wp_local, curr_pos_local, _ground_speed_2d);

	// store the result of above computation - collect all data from L1 controller and position triplet:
	_nav_bearing = _gnd_control.nav_bearing();	// bearing from current position to L1 point
	_target_bearing =
		_gnd_control.target_bearing();		// the direction between the rover position and next (current) waypoint
	_nav_lateral_acceleration_demand = _gnd_control.nav_lateral_acceleration_demand();

	_crosstrack_error = _gnd_control.crosstrack_error();  // distance meters from AB line

	//PX4_INFO_RAW("BEARING: tgt: %f  nav: %f   DEMAND: %f\n", (double)math::degrees(_target_bearing), (double)math::degrees(_nav_bearing), (double) _nav_lateral_acceleration_demand);
	//PX4_INFO_RAW("CROSS: _crosstrack_error: %f\n", (double) _crosstrack_error+);
}

void RoverPositionControl::adjustAcuatorSetpoints()
{
	switch (_pos_ctrl_state) {
	case POS_STATE_NONE:	// undefined/invalid state, no need to control anything
	case POS_STATE_IDLE:	// idle state, just make sure we stay put.

		_gas_engine_throttle = _param_gas_throttle_idle.get();
		_cutter_setpoint = ACTUATOR_OFF;
		_alarm_dev_level = ACTUATOR_OFF;

		break;

	case WP_ARRIVING:		// target waypoint is close, we need to slow down and head straight to it till stop

		_gas_engine_throttle = _param_gas_throttle_arriving.get();

		break;

	case WP_ARRIVED:		// reached waypoint. Make sure mission knows about it

		_gas_engine_throttle = _param_gas_throttle_arriving.get();

		break;

	case WP_TURNING:		// we need to turn in place towards the next waypoint

		_gas_engine_throttle = _param_gas_throttle_turning.get();

		break;

	case WP_DEPARTING:		// we turned to next waypoint and must start accelerating

		_gas_engine_throttle = _param_gas_throttle_departing.get();
		//_cutter_setpoint = ACTUATOR_ON;

		break;

	case L1_GOTO_WAYPOINT: 	// target waypoint is far away, we can use L1 and cruise speed

		_gas_engine_throttle = _param_gas_throttle_straight.get();

		break;

	case POS_STATE_STOPPING: 		// we hit a waypoint and need to stop before we declare "we arrived"

		_gas_engine_throttle = _param_gas_throttle_idle.get();

		break;

	case POS_STATE_MISSION_START:	// turn on what we need for the mission (lights, gas engine throttle, blades)

		_gas_engine_throttle = _param_gas_throttle_idle.get();
		_cutter_setpoint = ACTUATOR_OFF;			// keep the tools off until we start departing from the first waypoint of the mission
		_alarm_dev_level = ACTUATOR_OFF;

		break;

	case POS_STATE_MISSION_END:		// turn off what we needed for the mission at the end or error

		_gas_engine_throttle = _param_gas_throttle_idle.get();
		_cutter_setpoint = ACTUATOR_OFF;
		_alarm_dev_level = ACTUATOR_OFF;

		break;

	default:
		break;
	}
}

void RoverPositionControl::setStateMachineState(const POS_CTRLSTATES desiredState)
{
#ifdef DEBUG_MY_PRINT
	PX4_INFO("FYI: setting new state:  %s  -->  %s", control_state_name(_pos_ctrl_state), control_state_name(desiredState));
#endif // DEBUG_MY_PRINT

	switch (desiredState) {
	case L1_GOTO_WAYPOINT:
		// make small adjustments more effective on straight lines:
		_rate_control.setFeedForwardGain(matrix::Vector3f(0.0f, 0.0f, _param_rate_ff.get()));
		break;

	default:
		//_rate_control.setFeedForwardGain(matrix::Vector3f(0.0f, 0.0f, _param_rate_ff.get() / 50.0f));
		_rate_control.setFeedForwardGain(matrix::Vector3f(0.0f, 0.0f, 0.0f));
		break;
	}

	_rate_control.setFeedForwardGain(matrix::Vector3f(0.0f, 0.0f, 0.0f));
	_pos_ctrl_state = desiredState;
}

void RoverPositionControl::setActControls()
{
	// Smooth and constrain:

	_torque_control = math::constrain(_mission_torque_ema.Compute(
			PX4_ISFINITE(_mission_torque_effort) ? _mission_torque_effort : 0.0f), -1.0f, 1.0f);

	_thrust_control = math::constrain(_mission_thrust_ema.Compute(
			PX4_ISFINITE(_mission_thrust_effort) ? _mission_thrust_effort : 0.0f), -1.0f, 1.0f);
}

bool RoverPositionControl::updateBearings()
{
	// get the direction between the current position and next (target) waypoint:

	_target_bearing = get_bearing_to_next_waypoint(_current_position(0), _current_position(1), _curr_wp(0), _curr_wp(1));

	_heading_error = wrap_pi(_target_bearing - wrap_pi(_current_heading));

	return PX4_ISFINITE(_heading_error);
}

// --------------------------------------------------------------------------------------------------------------
// computing actuator responses for heavy differential drive rover:

float RoverPositionControl::computeTurningSetpoint()
{
	// In most states we just convert Heading error (radians) to Turning Setpoint. It could be scaled later:
	float turning_setpoint = PX4_ISFINITE(_heading_error) ? _heading_error : 0.0f;
	float max_turning_sp = 0.5f;

	switch (_pos_ctrl_state) {

	case L1_GOTO_WAYPOINT:

		if (PX4_ISFINITE(_crosstrack_error) && _param_line_following_p.get() > FLT_EPSILON) {	// GND_LF_P > 0

			// Use heading PID based on L1 _crosstrack_error:
			//max_turning_sp = 3.0f;
			turning_setpoint = -_crosstrack_error * _param_lf_scaler.get();	// GND_LF_SCALER

			//PX4_INFO_RAW("prev_dist: %.1f   crosstrack err: %.3f   miss trng setpnt: %.3f\n", (double) _wp_previous_dist, (double)_crosstrack_error, (double)turning_setpoint);

		} else if (PX4_ISFINITE(_nav_lateral_acceleration_demand)) {

			// Use L1 _nav_lateral_acceleration_demand as it was intended:

			turning_setpoint = _nav_lateral_acceleration_demand * _param_l1_scaler.get();	// GND_L1_SCALER

		} else {
			PX4_WARN("L1_GOTO_WAYPOINT lat_accel_demand is NULL    prev_dist: %.1f  crosstrack err: %.3f\n",
				 (double) _wp_previous_dist, (double)_crosstrack_error);

			turning_setpoint = 0.0f;
		}

		break;

	default:
		break;
	}

	//return math::constrain(turning_setpoint, -max_turning_sp, max_turning_sp);
	return math::sq(math::constrain(turning_setpoint, -max_turning_sp, max_turning_sp)) * sign(turning_setpoint);
}

float RoverPositionControl::computeTorqueEffort()
{
	if (!PX4_ISFINITE(_mission_turning_setpoint)) {
		// we are in overshot or stopping at waypoint, or otherwise don't want to turn:

		//PX4_WARN("_mission_turning_setpoint=%f  in computeTorqueEffort()", (double)_mission_turning_setpoint);

		return NAN;
	}

	// We have control effort, calculated by L1 control, or derived from _heading_error when in turn, departing or arriving.
	// Apply Rate Control to it.

	float torque_effort = NAN;

	_rates_setpoint.roll = _rates_setpoint.pitch = 0.0f;

	switch (_pos_ctrl_state) {

	case L1_GOTO_WAYPOINT: {

			float lf_rate_scaler = 1.0f;
			_rates_setpoint.yaw = _rates_setpoint_yaw = NAN;

			if (_param_line_following_p.get() > FLT_EPSILON) {
				// Use heading PID based on _crosstrack_error, not L1 acceleration demand:
				torque_effort = pid_calculate(&_line_following_ctrl, 0.0f, -_mission_turning_setpoint * 0.1f, 0.0f,
							      _dt); // already constrained with GND_LF_MAX

				lf_rate_scaler = _param_line_following_rate_scaler.get(); // GND_LF_RATE_SC

			} else {
				torque_effort = _mission_turning_setpoint; // for the Rate Controller below, when using L1
			}

			if (lf_rate_scaler > FLT_EPSILON) {
				// Use calculated value as input to Rate Controller:
				_rates_setpoint.yaw = _rates_setpoint_yaw = math::constrain(torque_effort *
						      _param_heading_rate_scaler.get() * lf_rate_scaler,	  // GND_HEADING_RATE, GND_LF_RATE_SC
						      -_param_heading_trim.get(), _param_heading_trim.get()); // GND_HEADING_TRIM

				torque_effort = control_yaw_rate(_angular_velocity, _rates_setpoint);
			}
		}
		break;

	case WP_TURNING:

		_rates_setpoint.yaw = _rates_setpoint_yaw = sign(_mission_turning_setpoint) *
				      _param_turn_rate_sp.get(); // GND_TURN_RATE

		torque_effort = control_yaw_rate(_angular_velocity, _rates_setpoint);

		break;

	default:

		_rates_setpoint.yaw = _rates_setpoint_yaw = math::constrain(_mission_turning_setpoint *
				      _param_heading_rate_scaler.get(),	// GND_HEADING_RATE
				      -_param_heading_trim.get(), _param_heading_trim.get()); // GND_HEADING_TRIM

		torque_effort = control_yaw_rate(_angular_velocity, _rates_setpoint);

		break;
	}

	//PX4_WARN("Y-RATE: trng_efrt: %.3f  rates_sp_yaw: %.3f  torq_efrt: %.3f", (double)_mission_turning_setpoint, (double)_rates_setpoint_yaw, (double)torque_effort);

	return math::constrain(torque_effort, -1.0f, 1.0f);
}

float RoverPositionControl::computeVelocitySetpoint()
{
	float velocity_sp = _mission_velocity_setpoint;

	switch (_pos_ctrl_state) {
	case WP_ARRIVING:		// target waypoint is close, we need to slow down and head straight to it till stop
	case WP_DEPARTING:		// we turned to next waypoint and must accelerate
	case L1_GOTO_WAYPOINT: {

			float braking_distance = math::min(_wp_previous_dist, _wp_current_dist);
			float final_speed = 0.0f; //_param_turn_speed.get();	// final_speed, GND_TURN_SPEED

			// Compute the maximum possible speed on the track given the desired speed,
			// remaining distance, the maximum acceleration and the maximum jerk.
			// We assume a constant acceleration profile with a delay of 2*accel/jerk
			// (time to reach the desired acceleration from opposite max acceleration)
			// Equation to solve: vel_final^2 = vel_initial^2 - 2*accel*(x - vel_initial*2*accel/jerk)
			float max_velocity = math::trajectory::computeMaxSpeedFromDistance(
						     _param_rdd_max_jerk.get(),	 // RDD_MAX_JERK
						     _param_rdd_max_accel.get(), // RDD_MAX_ACCEL
						     braking_distance, final_speed);

			max_velocity = math::min(max_velocity, _mission_velocity_setpoint);

			_forwards_velocity_smoothing.updateDurations(max_velocity);

			const float dt = math::min(_dt, 5.0f);	// no more than 5 seconds
			_forwards_velocity_smoothing.updateTraj(dt);

			float smooth_speed = _forwards_velocity_smoothing.getCurrentVelocity();

			// decrease it when heading error grows (but error is constrained between 0.1 and 0.8 radians):
			//velocity_sp =  math::interpolate<float>(abs(_heading_error) / 3, 0.1f, 0.8f, smooth_speed, 0.0f);
			velocity_sp = smooth_speed;

			//PX4_INFO_RAW("mission_vel_sp: %.4f   max: %.4f   smooth: %.4f   velocity_sp: %.4f   hdg_err: %.4f\n",
			//	     (double)_mission_velocity_setpoint, (double)max_velocity, (double)smooth_speed,
			//	     (double)velocity_sp, (double)_heading_error);

			velocity_sp = math::max(_param_turn_speed.get(), velocity_sp);	// GND_TURN_SPEED
		}
		break;

	default:
		break;
	}

	velocity_sp = _velocity_setpoint_ema.Compute(velocity_sp); // smooth the jitter for the speed PID's input

	return velocity_sp;
}

float RoverPositionControl::computeThrustEffort()
{
	// Control the thrust - by the speed PID, setpoint adjusted for proximity and sharp turns.

	if (!PX4_ISFINITE(_mission_velocity_setpoint)) {
		return NAN;
	}

	// At this moment _mission_velocity_setpoint is likely a smoothed cruise speed (global or per waypoint).
	// PID_MODE_DERIVATIV_CALC calculates discrete derivative from previous error, val_dot in pid_calculate() will be ignored
	// Let PID speed control compute thrust increment. Acceleration argument ignored in PID_MODE_DERIVATIV_CALC mode:
	float thrust = pid_calculate(&_speed_ctrl, _mission_velocity_setpoint, _x_vel_ema, 0.0f, _dt);

	//float speed_err = _x_vel_ema - _mission_velocity_setpoint;
	//PX4_INFO("SPEED: trgt: %.4f xvel: %.4f xvel_ema: %.4f err: %.4f thrust: %.4f", (double)_mission_velocity_setpoint, (double)_x_vel, (double)_x_vel_ema, (double)speed_err, (double)thrust);

	return math::constrain(thrust, _param_thrust_min.get(),
			       _param_thrust_max.get()); // can be NAN.  GND_THR_MIN, GND_THR_MAX
}

void RoverPositionControl::adjustThrustAndTorque()
{
	// computes _mission_thrust (by PID or for slowing down) and computes _mission_torque_effort from _mission_turning_setpoint (for more gentle turns).

	if (!_control_mode.flag_armed) {
		_mission_torque_effort = _mission_thrust_effort = NAN;

	} else {
		_mission_torque_effort = computeTorqueEffort();	// can be a factor in speed calculation below. Can be NAN.

		if (PX4_ISFINITE(_mission_velocity_setpoint)) {
			if (_manual_using_pids) {

				//_speed_proximity_factor = 1.0f;

			} else if (PX4_ISFINITE(_wp_current_dist)) {

				_mission_velocity_setpoint = computeVelocitySetpoint();

			} else {

				PX4_WARN("_wp_current_dist NAN");
			}

			_mission_thrust_effort =
				computeThrustEffort();	// compute effort from _mission_velocity_setpoint using speed PID. Can be NAN.

		} else {
			if (_pos_ctrl_state != POS_STATE_IDLE) {
				PX4_WARN("_mission_velocity_setpoint=NAN  in adjustThrustAndTorque() - in %s", control_state_name(_pos_ctrl_state));
			}

			_mission_thrust_effort = NAN;
		}
	}

	// Now reset what is needed when asked to stop:

	if (!PX4_ISFINITE(_mission_torque_effort)) {
		resetTorqueControls();
	}

	if (!PX4_ISFINITE(_mission_thrust_effort)) {
		resetTorqueControls();
		resetThrustControls();
	}
}

void RoverPositionControl::resetTorqueControls()
{
	//PX4_WARN("resetTorqueControls");

	_rate_control.resetIntegral();
	pid_reset_integral(&_line_following_ctrl);

	_mission_torque_ema.Reset();
	//_mission_thrust_ema.Reset();
}

void RoverPositionControl::resetThrustControls()
{
	//PX4_WARN("resetThrustControls");

	//pid_reset_integral(&_speed_ctrl);

	//_velocity_setpoint_ema.Reset();
	//_mission_thrust_ema.Reset();
}

void RoverPositionControl::resetVelocitySmoothing()
{
	//PX4_WARN("resetVelocitySmoothing");

	// start computing trajectories from scratch:
	_forwards_velocity_smoothing.reset(0.0f, _x_vel, 0.0f); // accel, vel, pos
}

void RoverPositionControl::setDefaultMissionSpeed()
{
	// target speed is GND_SPEED_TRIM - later scaled for waypoint proximity and heading deviation, and smoothed:
	_mission_velocity_setpoint = _param_gndspeed_trim.get();

	if (PX4_ISFINITE(_pos_sp_triplet.current.cruising_speed) && _pos_sp_triplet.current.cruising_speed > 0.1f) {
		_mission_velocity_setpoint = _pos_sp_triplet.current.cruising_speed;
	}
}

