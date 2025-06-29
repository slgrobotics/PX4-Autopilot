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

	POS_CTRLSTATES pos_ctrl_state_prev = _pos_ctrl_state;


	switch (_pos_ctrl_state) {
	case POS_STATE_NONE:				// "wound down" undefined/invalid state, no need controlling anything
		break;

	case POS_STATE_IDLE:				// idle state, no need controlling anything yet, initiate starting the mission

		setStateMachineState(POS_STATE_MISSION_START);

		break;

	case STRAIGHT_RUN: {				// target waypoint is far away, we can use Pursuit and cruise speed

			bool is_arriving = PX4_ISFINITE(_wp_current_dist) ?
					   _wp_current_dist < _decel_dist : // GND_DECEL_DIST or half leg
					   false;

			if (is_arriving) {
				// Close enough to destination waypoint, switch from Pursuit to direct heading:
				setStateMachineState(WP_ARRIVING);
				cte_end();

			} else {
				// Normal run.
				// maybe, set desired speed?
			}

		} break;

	case WP_ARRIVING:				// target waypoint is close, we need to slow down and head straight to it till stop

		if (PX4_ISFINITE(_wp_current_dist) && _wp_current_dist < _param_lm_wp_precision.get()) {
#ifdef DEBUG_MY_PRINT
			PX4_INFO("OK: got close, switching to POS_STATE_STOPPING ===========================");
#endif // DEBUG_MY_PRINT
			// We are closer than GND_WP_PRECISN radius to waypoint, begin stopping phase:
			setStateMachineState(POS_STATE_STOPPING);
		}

#ifdef DEBUG_MY_PRINT
		debugPrintArriveDepart();
#endif // DEBUG_MY_PRINT

		break;

	case WP_ARRIVED:				// reached waypoint, completely stopped. Make sure mission knows about it

		// See if that was the last waypoint of the mission:
		if (!_pos_sp_triplet.current.valid
		    || (_pos_sp_triplet.current.valid
			&& _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND)) {
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
		_decel_dist = _param_lm_decel_dist.get();	// LM_DECEL_DIST

		if (fabsf(math::degrees(_bearing_error)) < 5.0f) { // TODO: make it a parameter?

			// We've turned close enough to the desired bearing, switch to Departing or Straight Run state:
			setStateMachineState(_accel_dist > FLT_EPSILON ? WP_DEPARTING : STRAIGHT_RUN);
		}

		break;

	case WP_DEPARTING:				// we turned to next waypoint and must start accelerating

		cte_begin(); // just invalidate _crosstrack_error_avg to avoid confusion

		if (_wp_previous_dist < _accel_dist) {  // TODO: is_first_leg here?

			// just turn on tools (cutting deck) - we are on the business part of the mission:
			_cutter_setpoint = ACTUATOR_ON;

#ifdef DEBUG_MY_PRINT
			debugPrintArriveDepart();
#endif // DEBUG_MY_PRINT

		} else {
			// we are far enough from departure waypoint and not heading to the first waypoint, switch to Pursuit:
			setStateMachineState(STRAIGHT_RUN);
			cte_begin();
		}

		break;

	case POS_STATE_STOPPING:			// we hit a waypoint and need to stop

		// we need to monitor velocity here, and if it is below a threshold, we can switch to WP_ARRIVED state:
		setStateMachineState(WP_ARRIVED);

		break;

	case POS_STATE_MISSION_START:			// turn on what we need for the mission (lights, gas engine throttle, blades)

#ifdef DEBUG_MY_PRINT
		PX4_INFO("Mission started - turn on what we need for the mission (lights, gas engine throttle, blades)");
#endif // DEBUG_MY_PRINT

		// First waypoint of the mission has arrived, go to it. First we need to turn towards it:
		setStateMachineState(WP_TURNING);

		cte_begin_mission();

		break;

	case POS_STATE_MISSION_END:			// turn off what we needed for the mission at the end or error

#ifdef DEBUG_MY_PRINT
		PX4_INFO("Mission ended - turning off what we needed for the mission");
#endif // DEBUG_MY_PRINT

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


	adjustAcuatorSetpoints();

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

void LawnmowerControl::adjustAcuatorSetpoints()
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

	case POS_STATE_STOPPING: 		// we hit a waypoint and need to stop before we declare "we arrived"

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

