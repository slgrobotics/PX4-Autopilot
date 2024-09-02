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

// computing actuator responses for heavy differential drive rover:

float RoverPositionControl::computeYawRateSetpoint()
{
	// In most states we just convert Heading error (radians) to Turning Setpoint. It could be scaled later:
	float turning_setpoint = PX4_ISFINITE(_heading_error) ? _heading_error : 0.0f;
	float max_turning_sp = 0.5f;

	return math::constrain(turning_setpoint, -max_turning_sp, max_turning_sp);
}

void RoverPositionControl::computeRdGuidance()
{
	// Guidance logic - first yaw rate:

	float desired_yaw_rate = -pid_calculate(&_pid_heading, 0.f, _heading_error, 0.f, _dt);

	desired_yaw_rate = math::constrain(desired_yaw_rate, -_max_yaw_rate, _max_yaw_rate);	// RD_MAX_YAW_RATE

	// now speed:
	float desired_speed = 0.0f;

	float braking_distance = math::min(_wp_previous_dist, _wp_current_dist);
	float final_speed = _param_turn_speed.get();	// final_speed, GND_TURN_SPEED

	// Compute the maximum possible speed on the track given the desired speed,
	// remaining distance, the maximum acceleration and the maximum jerk.
	// We assume a constant acceleration profile with a delay of 2*accel/jerk
	// (time to reach the desired acceleration from opposite max acceleration)
	// Equation to solve: vel_final^2 = vel_initial^2 - 2*accel*(x - vel_initial*2*accel/jerk)

	if (_param_rd_max_jerk.get() > FLT_EPSILON && _param_rd_max_accel.get() > FLT_EPSILON) {
		desired_speed = math::trajectory::computeMaxSpeedFromDistance(_param_rd_max_jerk.get(),
				_param_rd_max_accel.get(), braking_distance, final_speed);

		desired_speed = math::constrain(desired_speed, 0.0f, _max_leg_speed);
	}

	// Return setpoints:
	_rd_guidance.desired_speed = desired_speed;
	_rd_guidance.desired_yaw_rate = desired_yaw_rate;
}

float RoverPositionControl::computeTorqueEffort()
{
	if (!PX4_ISFINITE(_mission_yaw_rate_setpoint)) {
		// we are in overshot or stopping at waypoint, or otherwise don't want to turn:

		//PX4_WARN("_mission_yaw_rate_setpoint=%f  in computeTorqueEffort()", (double)_mission_yaw_rate_setpoint);

		resetTorqueControls();
		return NAN;
	}

	// We have control effort, calculated by Pursuit control, or derived from _heading_error when in turn, departing or arriving.
	// Apply Rate Control to it.

	float torque_effort = NAN;

	_rates_setpoint.roll = _rates_setpoint.pitch = 0.0f;

	switch (_pos_ctrl_state) {

	case L1_GOTO_WAYPOINT: {

			_rates_setpoint.yaw = _mission_yaw_rate_setpoint;

			switch (_param_lf_use_rates_controller.get()) {	// GND_LF_USE_RATE
			case 0:
				// 0 - no heading PID or Rate Control
				// torque_effort derived directly from _nav_bearing. Ignore _mission_yaw_rate_setpoint
				torque_effort = _heading_error * _param_line_following_p.get();	// GND_LF_P
				torque_effort = math::constrain(torque_effort, -0.5f, 0.5f);
				break;

			case 1:
				// 1 - just heading PID
				// torque_effort derived directly from heading PID output, _mission_yaw_rate_setpoint
				torque_effort = _mission_yaw_rate_setpoint * _param_line_following_p.get();	// GND_LF_P
				torque_effort = math::constrain(torque_effort, -0.5f, 0.5f);
				break;

			case 2:
				// 2 - heading PID and Rate Control
				torque_effort = control_yaw_rate();
				break;
			}

			/*
			PX4_INFO_RAW("xtrk: %.1f cm  msn_trng: %.3f   yaw: %.4f  spd: %.3f  trq_eff: %.3f\n",
					(double)(_crosstrack_error * 100.0f),
					(double)_mission_yaw_rate_setpoint,
					(double)_yaw_rate_setpoint,
					(double)_rd_guidance.desired_speed,
					(double)torque_effort);
			*/
		}
		break;

	case WP_TURNING:

		// Just use constant yaw rate - GND_TURN_RATE:
		_rates_setpoint.yaw = sign(_mission_yaw_rate_setpoint)
				      * math::radians(_param_turn_rate_sp.get()); // GND_TURN_RATE, deg/s

		torque_effort = control_yaw_rate();

		break;

	case WP_DEPARTING:
	case WP_ARRIVING:
	case WP_ARRIVED:
	case POS_STATE_STOPPING: {

			// Departures and Arrivals, use heading error for yaw rate setpoint:

			float max_yaw_rate_setpoint = math::radians(_param_rate_depart_arrive_trim.get()); // GND_RATE_AD_TRIM, deg/s

			_rates_setpoint.yaw = math::constrain(
						      _mission_yaw_rate_setpoint * _param_heading_ad_rate_scaler.get(), // GND_RATE_AD_SC
						      -max_yaw_rate_setpoint, max_yaw_rate_setpoint); // constrained with GND_RATE_AD_TRIM

			torque_effort = control_yaw_rate();

		}
		break;

	default:

		break;
	}

	//PX4_WARN("Y-RATE: trng_efrt: %.3f  rates_sp_yaw: %.3f  torq_efrt: %.3f", (double)_mission_yaw_rate_setpoint, (double)_rates_setpoint_yaw, (double)torque_effort);

	return math::constrain(torque_effort, -1.0f, 1.0f);
}

void RoverPositionControl::computeWheelSpeeds()
{
	// see src/modules/rover_differential/RoverDifferential.cpp

	float forward_speed = _thrust_control;
	float speed_diff = _torque_control +
			   _param_rd_rate_ztrq.get(); // RD_RATE_ZTRQ - small adustment for servo discrepancies on a straight line

	float combined_velocity = fabsf(forward_speed) + fabsf(speed_diff);

	if (combined_velocity > 1.0f) {
		// Prioritize yaw rate
		float excess_velocity = fabsf(combined_velocity - 1.0f);
		forward_speed -= sign(forward_speed) * excess_velocity;
	}

	// Calculate the left and right wheel speeds
	_wheel_speeds = Vector2f(forward_speed - speed_diff, forward_speed + speed_diff);

	/*
		PX4_INFO_RAW("Vsp: %.3f  THR %f ->  %f  TRQ %f -> %f  Whls: %f   %f\n",
			(double)_mission_velocity_setpoint,
			(double)_mission_thrust_effort, (double)_thrust_control, (double)_mission_torque_effort,
			(double)_torque_control, (double)_wheel_speeds(0), (double)_wheel_speeds(1));
	*/

	if (!_control_mode.flag_armed) {
		_wheel_speeds = {0.0f, 0.0f}; // stop
	}

	_wheel_speeds = matrix::constrain(_wheel_speeds, -1.0f, 1.0f);
}

float RoverPositionControl::computeThrustEffort()
{
	// Control the thrust - by the speed PID, setpoint adjusted for proximity and sharp turns.

	if (!PX4_ISFINITE(_mission_velocity_setpoint)) {
		resetThrustControls();
		return NAN;
	}

	// At this moment _mission_velocity_setpoint is likely a smoothed cruise speed (global or per waypoint).
	// PID_MODE_DERIVATIV_CALC calculates discrete derivative from previous error, val_dot in pid_calculate() will be ignored
	// Let PID speed control compute thrust increment. Acceleration argument ignored in PID_MODE_DERIVATIV_CALC mode:
	float thrust = pid_calculate(&_speed_ctrl, _mission_velocity_setpoint, _x_vel_ema, 0.0f, _dt);

	//float speed_err = _x_vel_ema - _mission_velocity_setpoint;
	//PX4_INFO("SPEED: trgt: %.4f xvel: %.4f xvel_ema: %.4f err: %.4f thrust: %.4f", (double)_mission_velocity_setpoint, (double)_x_vel, (double)_x_vel_ema, (double)speed_err, (double)thrust);

	// can be NAN:
	return math::constrain(thrust * _param_gnd_thrust_scaler.get(),	// GND_THRUST_SC
			       _param_thrust_min.get(),
			       _param_thrust_max.get()); // GND_THR_MIN, GND_THR_MAX
}

void RoverPositionControl::adjustThrustAndTorque()
{
	// computes _mission_thrust (by PID or for slowing down) and computes _mission_torque_effort from _mission_yaw_rate_setpoint (for more gentle turns).

	if (!_control_mode.flag_armed) {

		resetRdGuidance();

		_mission_torque_effort = _mission_thrust_effort = NAN;

	} else {

		/*
		PX4_INFO_RAW("des_yaw: %.4f  des_spd: %.4f  mis_vel_sp: %.3f\n",
				(double)_rd_guidance.desired_yaw_rate,
				(double)_rd_guidance.desired_speed,
				(double)_mission_velocity_setpoint);
		*/

		_mission_torque_effort = computeTorqueEffort();	// can be a factor in speed calculation below. Can be NAN.

		if (PX4_ISFINITE(_mission_velocity_setpoint)) {

			if (!_manual_using_pids) {
				if (PX4_ISFINITE(_wp_current_dist)) {

					// smooth the jitter for the speed PID's input
					_mission_velocity_setpoint = _velocity_setpoint_ema.Compute(_mission_velocity_setpoint);

				} else {

					PX4_WARN("_wp_current_dist NAN");
				}
			}

			// compute effort from _mission_velocity_setpoint using speed PID. Can be NAN.
			_mission_thrust_effort = computeThrustEffort();

		} else {
			if (_pos_ctrl_state != POS_STATE_IDLE) {
				PX4_WARN("%s : _mission_velocity_setpoint=NAN  in adjustThrustAndTorque()", control_state_name(_pos_ctrl_state));
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
	pid_reset_integral(&_pid_heading);

	_mission_torque_ema.Reset();
}

void RoverPositionControl::resetThrustControls()
{
	//PX4_WARN("resetThrustControls");

	pid_reset_integral(&_speed_ctrl);

	_velocity_setpoint_ema.Reset();
	_mission_thrust_ema.Reset();
}

void RoverPositionControl::setMaxLegSpeed()
{
	// target speed between waypoints is RD_MISS_SPD_DEF - later scaled for waypoint proximity and heading deviation, and smoothed:
	_max_leg_speed = _param_rd_miss_spd_def.get();

	if (PX4_ISFINITE(_pos_sp_triplet.current.cruising_speed) && _pos_sp_triplet.current.cruising_speed > FLT_EPSILON) {
		// mission plan can override it:
		_max_leg_speed = _pos_sp_triplet.current.cruising_speed;
	}
}

void RoverPositionControl::updateEkfGpsDeviation()
{
	// meters, how far is EKF2 calculated position from GPS reading:
	_ekfGpsDeviation = get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon,
			   _sensor_gps_data.latitude_deg, _sensor_gps_data.longitude_deg);
}

void RoverPositionControl::computeCrosstrackError()
{
	// See src/lib/l1/ECL_L1_Pos_Controller.cpp

	if (!PX4_ISFINITE(_wp_previous_dist)) {
		_crosstrack_error = NAN;
		return;
	}

	// vector operations change operands values, so get local copies:
	Vector2f curr_pos_local = _curr_pos_ned;
	Vector2f curr_wp_local = _curr_wp_ned;
	Vector2f prev_wp_local = _prev_wp_ned;

	// calculate vector from A to B
	Vector2f vector_AB = curr_wp_local - prev_wp_local;

	// check if waypoints are on top of each other
	if (vector_AB.length() < 1.0e-6f) {
		_crosstrack_error = NAN;
		return;
	}

	vector_AB.normalize();

	// calculate the vector from waypoint A to the aircraft
	Vector2f vector_A_to_vehicle = curr_pos_local - prev_wp_local;

	// calculate crosstrack error (output only)
	_crosstrack_error = vector_AB % vector_A_to_vehicle;   // distance meters from AB line
}
