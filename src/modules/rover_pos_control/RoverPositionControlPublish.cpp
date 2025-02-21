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

#ifdef PUBLISH_THRUST_TORQUE

void RoverPositionControl::publishTorqueSetpoint(const hrt_abstime &timestamp_sample, float torque)
{
	/*
	vehicle_torque_setpoint_s v_torque_sp = {};
	v_torque_sp.timestamp = _timestamp;
	v_torque_sp.timestamp_sample = timestamp_sample;
	v_torque_sp.xyz[0] = 0;
	v_torque_sp.xyz[1] = 0;
	v_torque_sp.xyz[2] = torque;

	_vehicle_torque_setpoint_pub.publish(v_torque_sp);
	*/
}

void RoverPositionControl::publishThrustSetpoint(const hrt_abstime &timestamp_sample, float thrust)
{
	/*
	vehicle_thrust_setpoint_s v_thrust_sp = {};
	v_thrust_sp.timestamp = _timestamp;
	v_thrust_sp.timestamp_sample = timestamp_sample;
	v_thrust_sp.xyz[0] = thrust;
	v_thrust_sp.xyz[1] = 0.0f;
	v_thrust_sp.xyz[2] = 0.0f;

	_vehicle_thrust_setpoint_pub.publish(v_thrust_sp);
	*/
}

/*
#else // PUBLISH_THRUST_TORQUE

void RoverPositionControl::publishWheelMotors(const hrt_abstime &timestamp_sample)
{
	actuator_motors_s actuator_motors{};
	actuator_motors.timestamp_sample = timestamp_sample;
	actuator_motors.timestamp = _timestamp;

	actuator_motors.reversible_flags = _param_r_rev.get(); // should be 3 see rc.rover_differential_defaults
	_wheel_speeds.copyTo(actuator_motors.control);

	_actuator_motors_pub.publish(actuator_motors);
}
*/

#endif // PUBLISH_THRUST_TORQUE

void RoverPositionControl::publishAuxActuators(const hrt_abstime &timestamp_sample)
{
	/*
	bool test_switch = _timestamp - _debug_print_last_called > 250_ms;

	//if(test_switch)	{ PX4_INFO("+"); } else { PX4_INFO("-"); }

	float test_signal = test_switch ? 0.5333f : -0.5444f;
	*/

	// the following is used to control cutter, gas engine throttle, alarm - via Control Allocation mixing:

	// we could set PCA9685 servos 3,4,5 to "RC passthrough" as follows in the Control Allocation ("Actuators") setting:
	//		3 - RC FLAPS (param set PCA9685_FUNC3 406) - leftmost switch, cutter blades or 203 for Servo3
	//		4 - RC AUX1  (param set PCA9685_FUNC4 407) - gas engine throttle, left knob, or 204 for Servo4
	//		5 - RC AUX2  (param set PCA9685_FUNC5 408) - right knob, or 207 for Servo7

	// We can control three remaining PCA9685 outputs via CA "Servo6..8" assignment:
	actuator_servos_s actuator_servos{};
	actuator_servos.timestamp_sample = timestamp_sample;
	actuator_servos.timestamp = _timestamp;

	for (unsigned i = 0; i < 8; ++i) {
		actuator_servos.control[i] = 0.0; //test_signal;
	}

	// Direct servo control, PCA9685 channels 5,6,7,8:
	// param set PCA9685_FUNC5 205
	// param set PCA9685_FUNC6 206
	// param set PCA9685_FUNC7 207    // 408 - right knob
	// param set PCA9685_FUNC8 208

	bool strobe_on = _nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;

	//bool horn_on = _alarm_dev_level > SIGMA;
	bool horn_on = !_control_mode.flag_control_manual_enabled &&
		       (_sensor_gps_data.fix_type < _gps_minfix || !PX4_ISFINITE(_sensor_gps_data.heading));

#ifndef PUBLISH_THRUST_TORQUE
	_wheel_speeds.copyTo(actuator_servos.control);
#endif // PUBLISH_THRUST_TORQUE

	actuator_servos.control[2] = _cutter_setpoint;		 // PCA9685 channel 3
	actuator_servos.control[3] = _gas_engine_throttle;	 // PCA9685 channel 4
	actuator_servos.control[4] = strobe_on ? ACTUATOR_ON : ACTUATOR_OFF; // PCA9685 channel 5 - Strobe
	actuator_servos.control[5] = horn_on ? ACTUATOR_ON : ACTUATOR_OFF;   // PCA9685 channel 6 - Horn
	actuator_servos.control[6] = _alarm_dev_level;		 // PCA9685 channel 7 - duplicate R/C ch 6, right knob
	actuator_servos.control[7] = 0; //test_signal;		 // PCA9685 channel 8 - spare

	_actuator_servos_pub.publish(actuator_servos);
}

void RoverPositionControl::publishControllerStatus()
{
	// publish controller status, mostly for tracing and tuning:

	position_controller_status_s pos_ctrl_status{};  // PositionControllerStatus.msg

	pos_ctrl_status.nav_roll = 0.0f;	// Roll setpoint [rad]
	pos_ctrl_status.nav_pitch = 0.0f;	// Pitch setpoint [rad]

	pos_ctrl_status.nav_bearing = _nav_bearing;		// Bearing angle[rad]
	pos_ctrl_status.target_bearing = _target_bearing;	// Bearing angle from aircraft to current target [rad]
	pos_ctrl_status.xtrack_error = _crosstrack_error;	// Signed track error [m]

	pos_ctrl_status.wp_dist = _wp_current_dist;	// Distance to active (next) waypoint [m]

	pos_ctrl_status.acceptance_radius =	// Current horizontal acceptance radius [m]
		_acceptance_radius;		// if large enough, will be used for mission advancement to next WP

	// _pos_sp_triplet.current is always valid here
	pos_ctrl_status.type =
		_pos_sp_triplet.current.type;	// Current (applied) position setpoint type (see PositionSetpoint.msg)

	// SETPOINT_TYPE_POSITION=0	 position setpoint
	// SETPOINT_TYPE_VELOCITY=1	 velocity setpoint
	// SETPOINT_TYPE_LOITER=2	 loiter setpoint
	// SETPOINT_TYPE_TAKEOFF=3	 takeoff setpoint
	// SETPOINT_TYPE_LAND=4		 land setpoint, altitude must be ignored, descend until landing
	// SETPOINT_TYPE_IDLE=5		 do nothing, switch off motors or keep at idle speed (MC)

	pos_ctrl_status.timestamp = _timestamp;

	_pos_ctrl_status_pub.publish(pos_ctrl_status);
}
