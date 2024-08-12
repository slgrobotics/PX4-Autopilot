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

float RoverPositionControl::computeTurningSetpoint()
{
	// In most states we just convert Heading error (radians) to Turning Setpoint. It could be scaled later:
	float turning_setpoint = PX4_ISFINITE(_heading_error) ? _heading_error : 0.0f;
	float max_turning_sp = 0.5f;

	/*
	switch (_pos_ctrl_state) {

	case L1_GOTO_WAYPOINT:

		if (PX4_ISFINITE(_nav_lateral_acceleration_demand)) {

			// Use L1 _nav_lateral_acceleration_demand as it was intended:

			turning_setpoint = _nav_lateral_acceleration_demand * _param_l1_scaler.get();	// GND_L1_SCALER

		} else {
			PX4_WARN("L1_GOTO_WAYPOINT lat_accel_demand is NULL    prev_dist: %.1f  crosstrack err: %.3f\n",
				 (double) _wp_previous_dist, (double)_crosstrack_error);

			turning_setpoint = 0.0f;
		}

		break;

	case WP_ARRIVING:
	case WP_DEPARTING:

		//if (_x_vel > 0.1f && PX4_ISFINITE(_heading_error_vel)) {
		//	turning_setpoint = _heading_error_vel;
		//}

		break;

	default:
		break;
	}
	*/

	return math::constrain(turning_setpoint, -max_turning_sp, max_turning_sp);
	//return math::sq(math::constrain(turning_setpoint, -max_turning_sp, max_turning_sp)) * sign(turning_setpoint);
}

void RoverPositionControl::computeRdGuidance()
{
	// see src/modules/rover_differential/RoverDifferentialGuidance/RoverDifferentialGuidance.cpp

	// Guidance logic - first yaw rate:
	const float yaw = _current_heading;	// The yaw orientation of the vehicle in radians.
	const float actual_speed = _x_vel_ema;	// The forward velocity of the vehicle in m/s.

	const float pp_desired_heading = _pure_pursuit.calcDesiredHeading(_curr_wp_ned, _prev_wp_ned, _curr_pos_ned,
					 math::max(actual_speed, 0.f));

	const float pp_heading_error = matrix::wrap_pi(pp_desired_heading - yaw);

	float desired_yaw_rate = pid_calculate(&_pid_heading, pp_heading_error, 0.f, 0.f, _dt);

	desired_yaw_rate = math::constrain(desired_yaw_rate, -_max_yaw_rate, _max_yaw_rate);

	_heading_error = pp_heading_error; // make Pure Pursuit heading error global

	// now speed:
	float desired_speed =
		_mission_velocity_setpoint; // RD_MISS_SPD_DEF - default mission speed, superceeded by leg speed from triplet

	float braking_distance = math::min(_wp_previous_dist, _wp_current_dist);
	float final_speed = 0.0f; //_param_turn_speed.get();	// final_speed, GND_TURN_SPEED

	// Compute the maximum possible speed on the track given the desired speed,
	// remaining distance, the maximum acceleration and the maximum jerk.
	// We assume a constant acceleration profile with a delay of 2*accel/jerk
	// (time to reach the desired acceleration from opposite max acceleration)
	// Equation to solve: vel_final^2 = vel_initial^2 - 2*accel*(x - vel_initial*2*accel/jerk)

	if (_param_rd_max_jerk.get() > FLT_EPSILON && _param_rd_max_accel.get() > FLT_EPSILON) {
		desired_speed = math::trajectory::computeMaxSpeedFromDistance(_param_rd_max_jerk.get(),
				_param_rd_max_accel.get(), braking_distance, final_speed);

		//const float max_speed = _param_rd_max_speed.get();	// RD_MAX_SPEED - not RD_MISS_SPD_DEF here
		const float max_speed = _mission_velocity_setpoint;

		desired_speed = math::constrain(desired_speed, -max_speed, max_speed);
	}

	/*
	const float distance_to_next_wp = get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon, _curr_wp(0),
					  _curr_wp(1));

	if (_param_rd_max_jerk.get() > FLT_EPSILON && _param_rd_max_accel.get() > FLT_EPSILON) {
		desired_speed = math::trajectory::computeMaxSpeedFromDistance(_param_rd_max_jerk.get(),
				_param_rd_max_accel.get(), distance_to_next_wp, 0.0f);
	}
	*/

	/*
	// Closed loop speed control
	float throttle{0.f};

	if (fabsf(desired_speed) < FLT_EPSILON) {
		pid_reset_integral(&_pid_throttle);

	} else {
		throttle = pid_calculate(&_pid_throttle, desired_speed, actual_speed, 0,
					 dt);

		if (_param_rd_max_speed.get() > FLT_EPSILON) { // Feed-forward term
			throttle += math::interpolate<float>(desired_speed,
							     0.f, _param_rd_max_speed.get(),
							     0.f, 1.f);
		}
	}

	throttle = math::constrain(throttle, 0.f, 1.f);
	*/

	// Return setpoints:
	_rd_guidance.desired_speed = desired_speed;
	_rd_guidance.desired_yaw_rate = desired_yaw_rate;
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

			_rates_setpoint.yaw = _yaw_rate_setpoint = _rd_guidance.desired_yaw_rate;

			torque_effort = control_yaw_rate(_angular_velocity, _rates_setpoint);

			/*
			PX4_INFO_RAW("xtrk: %.1f cm  msn_trng: %.3f   yaw: %.4f  spd: %.3f  trq_eff: %.3f\n",
					(double)(_crosstrack_error * 100.0f),
					(double)_mission_turning_setpoint,
					(double)_yaw_rate_setpoint,
					(double)_rd_guidance.desired_speed,
					(double)torque_effort);
			*/

#ifdef OLD_L1_LOGIC
			// Start with Arrive-Depart calculations (heading error based):

			float turning_setpoint_hdg = _mission_turning_setpoint *
						     _param_line_following_heading_error_scaler.get(); // GND_LF_HDG_SC

			// See what L1 could add to this:

			float turning_setpoint_l1 = 0.0f;
			float l1_scaler = _param_l1_scaler.get(); // GND_L1_SCALER

			if (l1_scaler > FLT_EPSILON) {

				if (PX4_ISFINITE(_nav_lateral_acceleration_demand)) {

					// Use L1 _nav_lateral_acceleration_demand as it was intended:

					turning_setpoint_l1 = _nav_lateral_acceleration_demand * l1_scaler; // GND_L1_SCALER

				} else {
					PX4_WARN("L1_GOTO_WAYPOINT lat_accel_demand is NULL    prev_dist: %.1f  crosstrack err: %.3f\n",
						 (double) _wp_previous_dist, (double)_crosstrack_error);
				}
			}

			float turning_setpoint_lf = 0.0f;
			float lf_scaler = _param_line_following_pid_output_scaler.get(); // GND_LF_PID_SC

			if (lf_scaler > FLT_EPSILON) {	// GND_LF_PID_SC

				turning_setpoint_lf = pid_calculate(&_line_following_ctrl, 0.0f, _crosstrack_error * lf_scaler, 0.0f,
								    _dt); // constrained with GND_LF_MAX

			}

			float max_yaw_rate_setpoint = _param_rate_depart_arrive_trim.get(); // GND_RATE_AD_TRIM

			float setpoint_yaw = math::constrain(
						     turning_setpoint_hdg + turning_setpoint_l1 + turning_setpoint_lf,
						     -max_yaw_rate_setpoint, max_yaw_rate_setpoint); // constrained with GND_RATE_AD_TRIM

			const bool use_rates_controller = _param_lf_use_rates_controller.get() > 0; // GND_LF_USE_RATE > 0 or outside corridor

			if (use_rates_controller) {

				// Use calculated value as "yaw rate setpoint" - input to Rate Controller:

				_rates_setpoint.yaw = _rates_setpoint_yaw = setpoint_yaw;

				torque_effort = control_yaw_rate(_angular_velocity, _rates_setpoint);

			} else {

				_rates_setpoint_yaw = NAN;
				// experimental factor to leave GND_LF_HDG_SC and others intact while switching off GND_LF_USE_RATE:
				torque_effort = setpoint_yaw * 0.2f;
			}

			/*
			PX4_INFO_RAW("%.4f/%.4f/%.4f  xtrk: %.1f cm   msn_trng_sp: %.3f   sp_yaw: %.3f  torque_effort: %.3f\n",
				     (double)turning_setpoint_hdg, (double)turning_setpoint_l1, (double)turning_setpoint_lf,
				     (double)(_crosstrack_error * 100.0f),
				     (double)_mission_turning_setpoint,
				     (double)_rates_setpoint_yaw,
				     (double)torque_effort);
			*/

			/*
			// Far from the line use traditional L1 Controller, _nav_lateral_acceleration_demand as it was intended:

			float setpoint_yaw =
				_mission_turning_setpoint; // for the Rate Controller below, basically scaled _nav_lateral_acceleration_demand

			float lf_corridor_boundary = _param_line_following_width.get() / 2.0f;	// GND_LF_WIDTH - set to 0 for pure L1 control

			bool in_corridor = PX4_ISFINITE(_crosstrack_error) && abs(_crosstrack_error) < lf_corridor_boundary;

			if (in_corridor) {

				//float h_err = _heading_error;
				float h_err = _heading_error_vel;

				// Weighted L1 and heading error control within the GND_LF_WIDTH corridor

				//float hdg_err_weight = -1.0f / math::sq(lf_corridor_boundary) * math::sq(_crosstrack_error) +
				//		       1.0f; // flat 1 within 0, quadratic to sides till 0
				//float l1_weight = 1.0f - hdg_err_weight; // strong at the border, weak in center

				//float l1_weight = abs(_crosstrack_error / lf_corridor_boundary); // 0 in center, 1 at the boundary

				//float hdg_err_weight = 1.0f - l1_weight; // 1 in center, 0 at the boundary

				// adjust the L1 result with weighted heading error, to improve line following near the line:
				//float setpoint_yaw_hdg = sqrt_signed(h_err) * _param_line_following_heading_error_scaler.get(); // GND_LF_HDG_SC
				float setpoint_yaw_hdg = h_err * _param_line_following_heading_error_scaler.get(); // GND_LF_HDG_SC

				//setpoint_yaw = setpoint_yaw_hdg * hdg_err_weight + setpoint_yaw * l1_weight;

				const bool use_lf_pid = _param_line_following_p.get() > FLT_EPSILON;
				float pid_adjustment = 0.0f;

				setpoint_yaw = 0.0f;

				if (use_lf_pid) {

					// Use heading PID based on _crosstrack_error:

					bool is_moving_to_center = _crosstrack_error * h_err > 0; // we are moving towards the centerline

					/ *
					if (!is_moving_to_center) {

						// we need a correction only if we are moving away from the centerline:

						// heading error has reverse sign, this is just adding the two:
						float total_error = _crosstrack_error - setpoint_yaw_hdg * 10.0f;

						PX4_INFO_RAW("%.3f\n", (double)total_error);
					} else {
						resetTorqueControls(); // let the vehicle move in the same direction
					}
					* /

					// heading error has reverse sign, this is just adding the two:
					float total_error = _crosstrack_error - setpoint_yaw_hdg * 10.0f;

					if (is_moving_to_center) {

						// we need much weaker correction if we are already moving towards the centerline:

						//setpoint_yaw = total_error * 0.01f; // _crosstrack_error;
						//setpoint_yaw = _mission_turning_setpoint; // ok
						//setpoint_yaw = 0.0f; // not good
						setpoint_yaw = setpoint_yaw_hdg;

						pid_reset_integral(&_line_following_ctrl);

						//resetTorqueControls();

					} else {
						//resetTorqueControls();

						PX4_INFO_RAW("%.3f   %.3f\n", (double)total_error, (double)yaw_responsiveness_factor());

						float lf_pid_output = pid_calculate(&_line_following_ctrl, 0.0f, total_error, 0.0f, _dt); // constrained with GND_LF_MAX

						pid_adjustment = lf_pid_output * _param_line_following_pid_output_scaler.get(); // GND_LF_PID_SC

						// adjust the weighted result with PID result:
						setpoint_yaw += pid_adjustment;
					}
				}

				//PX4_INFO_RAW("err xtrk: %.1f cm abbe: %.2f m  msn_trng_sp: %.3f  sp_yaw_hdg: %.3f  pid_adj: %.3f  sp_yaw: %.3f\n",
				//	     (double)(_crosstrack_error * 100.0f), (double)_abbe_error, (double)_mission_turning_setpoint, (double)setpoint_yaw_hdg,
				//	     (double)pid_adjustment, (double)setpoint_yaw);

				//PX4_INFO_RAW("%.3f/%.3f err xtrk: %.1f cm abbe: %.2f m  msn_trng_sp: %.3f  sp_yaw_hdg: %.3f  pid_adj: %.3f  sp_yaw: %.3f\n",
				//	     (double)hdg_err_weight, (double)l1_weight,
				//	     (double)(_crosstrack_error * 100.0f), (double)_abbe_error, (double)_mission_turning_setpoint, (double)setpoint_yaw_hdg,
				//	     (double)pid_adjustment, (double)setpoint_yaw);

				//PX4_INFO_RAW("%.4f/%.4f  xtrk: %.1f cm   msn_trng_sp: %.3f  setpoint_yaw_hdg: %.3f  pid_adj: %.3f  sp_yaw: %.3f\n",
				//		(double)hdg_err_weight, (double)l1_weight,
				//		(double)(_crosstrack_error * 100.0f), (double)_mission_turning_setpoint, (double)setpoint_yaw_hdg,
				//		(double)pid_adjustment, (double)setpoint_yaw);

			}

			const bool use_rates_controller = !in_corridor
							  || _param_lf_use_rates_controller.get() > 0; // GND_LF_USE_RATE > 0 or outside corridor

			if (use_rates_controller) {

				// Use calculated value as "yaw rate setpoint" - input to Rate Controller:
				_rates_setpoint.yaw = _rates_setpoint_yaw = setpoint_yaw;

				torque_effort = control_yaw_rate(_angular_velocity, _rates_setpoint);

			} else {
				// only within the corridor:
				_rates_setpoint_yaw = NAN;
				torque_effort = setpoint_yaw /
						10.0f; // experimental factor to leave GND_LF_HDG_SC intact while switching GND_LF_USE_RATE
			}
			*/
#endif // OLD_L1_LOGIC
		}
		break;

	case WP_TURNING:

		// Just use constant yaw rate - GND_TURN_RATE:
		_rates_setpoint.yaw = _yaw_rate_setpoint =
					      sign(_mission_turning_setpoint) * _param_turn_rate_sp.get(); // GND_TURN_RATE

		torque_effort = control_yaw_rate(_angular_velocity, _rates_setpoint);

		break;

	case WP_DEPARTING:
	case WP_ARRIVING:
	case WP_ARRIVED:
	case POS_STATE_STOPPING: {

			// Departures and Arrivals, use heading error for yaw rate setpoint:

			float max_yaw_rate_setpoint = _param_rate_depart_arrive_trim.get(); // GND_RATE_AD_TRIM

			_rates_setpoint.yaw = _yaw_rate_setpoint = math::constrain(
						      _mission_turning_setpoint * _param_heading_ad_rate_scaler.get(), // GND_RATE_AD_SC
						      -max_yaw_rate_setpoint, max_yaw_rate_setpoint); // constrained with GND_RATE_AD_TRIM

			torque_effort = control_yaw_rate(_angular_velocity, _rates_setpoint);

		}
		break;

	default:

		break;
	}

	//PX4_WARN("Y-RATE: trng_efrt: %.3f  rates_sp_yaw: %.3f  torq_efrt: %.3f", (double)_mission_turning_setpoint, (double)_rates_setpoint_yaw, (double)torque_effort);

	return math::constrain(torque_effort, -1.0f, 1.0f);
}

float RoverPositionControl::adjustMissionVelocitySetpoint()
{
	float velocity_sp =
		_mission_velocity_setpoint;	// start with default RD_MISS_SPD_DEF or the leg speed - see setDefaultMissionSpeed()

	switch (_pos_ctrl_state) {
	case WP_ARRIVING:		// target waypoint is close, we need to slow down and head straight to it till stop
	case WP_DEPARTING:		// we turned to next waypoint and must accelerate
	case L1_GOTO_WAYPOINT: {

			float max_velocity = _rd_guidance.desired_speed; // already limited by _mission_velocity_setpoint

			_forwards_velocity_smoothing.updateDurations(max_velocity);

			_forwards_velocity_smoothing.updateTraj(_dt);

			float smooth_speed = _forwards_velocity_smoothing.getCurrentVelocity();

			// decrease it when heading error grows (but error is constrained between 0.1 and 0.8 radians):
			velocity_sp = abs(_heading_error) > 0.5f ?  math::interpolate<float>(abs(_heading_error) / 3, 0.1f, 0.8f, smooth_speed,
					0.0f) : smooth_speed;
			//velocity_sp = smooth_speed;

			//PX4_INFO_RAW("mission_vel_sp: %.4f   max: %.4f   smooth: %.4f   velocity_sp: %.4f   hdg_err: %.4f\n",
			//	     (double)_mission_velocity_setpoint, (double)max_velocity, (double)smooth_speed,
			//	     (double)velocity_sp, (double)_heading_error);

			velocity_sp = math::max(_param_turn_speed.get(), velocity_sp);	// GND_TURN_SPEED is a minimum
		}
		break;

	default:
		break;
	}

	velocity_sp = _velocity_setpoint_ema.Compute(velocity_sp); // smooth the jitter for the speed PID's input

	return velocity_sp; // will become new _mission_velocity_setpoint value
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

		resetRdGuidance();

		_mission_torque_effort = _mission_thrust_effort = NAN;

	} else {

		computeRdGuidance();	// computes desired speed and yaw rate in _rd_guidance

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

					_mission_velocity_setpoint = adjustMissionVelocitySetpoint();

				} else {

					PX4_WARN("_wp_current_dist NAN");
				}
			}

			_mission_thrust_effort =
				computeThrustEffort();	// compute effort from _mission_velocity_setpoint using speed PID. Can be NAN.

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
	pid_reset_integral(&_line_following_ctrl);
	pid_reset_integral(&_pid_heading);

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
	// target speed is RD_MISS_SPD_DEF - later scaled for waypoint proximity and heading deviation, and smoothed:
	_mission_velocity_setpoint = _param_rd_miss_spd_def.get();

	if (PX4_ISFINITE(_pos_sp_triplet.current.cruising_speed) && _pos_sp_triplet.current.cruising_speed > 0.1f) {
		_mission_velocity_setpoint = _pos_sp_triplet.current.cruising_speed;
	}
}

void RoverPositionControl::updateEkfGpsDeviation()
{
	// meters, how far is EKF2 calculated position from GPS reading:
	_ekfGpsDeviation = get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon,
			   _sensor_gps_data.latitude_deg, _sensor_gps_data.longitude_deg);
}

void RoverPositionControl::compute_crosstrack_error()
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
