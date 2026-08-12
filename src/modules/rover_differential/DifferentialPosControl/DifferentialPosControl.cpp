/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include "DifferentialPosControl.hpp"

using namespace time_literals;

DifferentialPosControl::DifferentialPosControl(ModuleParams *parent) : ModuleParams(parent)
{
	_pure_pursuit_status_pub.advertise();
	_rover_speed_setpoint_pub.advertise();
	_rover_attitude_setpoint_pub.advertise();

	updateParams();
}

void DifferentialPosControl::updateParams()
{
	ModuleParams::updateParams();
}

void DifferentialPosControl::updatePosControl()
{
	updateSubscriptions();

	hrt_abstime timestamp = hrt_absolute_time();


	if (_target_waypoint_ned.isAllFinite()) {
		float distance_to_target = (_target_waypoint_ned - _curr_pos_ned).norm();
		bool isArrivalFast = PX4_ISFINITE(_arrival_speed) && _arrival_speed > FLT_EPSILON;

		float distance_from_stopped = (_curr_pos_ned - _stopped_ned).norm();

		pure_pursuit_status_s pure_pursuit_status{};
		pure_pursuit_status.timestamp = timestamp;

		if (distance_to_target > _param_nav_acc_rad.get() || _arrival_speed > FLT_EPSILON) {
			if (_param_ro_decel_limit.get() > FLT_EPSILON && _param_ro_jerk_limit.get() > FLT_EPSILON) {
				_speed_setpoint = math::min(math::trajectory::computeMaxSpeedFromDistance(_param_ro_jerk_limit.get(),
							    _param_ro_decel_limit.get(), distance_to_target, fabsf(_arrival_speed)), _cruising_speed);
			}
		}

		if (isArrivalFast || distance_to_target > _param_nav_acc_rad.get()) {

			// apply PurePursuit - figure out desired yaw to closely follow the line from previous waypoint to current one:
			float yaw_setpoint = PurePursuit::calcTargetBearing(pure_pursuit_status, _param_pp_lookahd_gain.get(),
					     _param_pp_lookahd_max.get(), _param_pp_lookahd_min.get(), _target_waypoint_ned,
					     _start_ned, _curr_pos_ned,
					     fabsf(PX4_ISFINITE(_vehicle_speed) ? _vehicle_speed : 0.f));  // _speed_setpoint from last cycle can be used here

			float heading_error = matrix::wrap_pi(yaw_setpoint - _vehicle_yaw);

			// if (fabsf(matrix::wrap_pi(heading_error - M_PI_F)) < 0.1f) {

			// 	if (fabsf(_turn_bias) < FLT_EPSILON) {
			// 		_turn_bias = sign(heading_error) * 0.5f; // remember where we started turning
			// 		printf("++++++++++ Pos : turning bias: %.2f deg\n", (double)math::degrees(_turn_bias));
			// 	}

			// 	// A ~180 degrees turn. Escape the oscillations trap.
			// 	printf("========== Pos : pursuit yaw_setpoint: %.2f deg,  heading err: %.2f bias: %.2f\n",
			// 	       (double)math::degrees(yaw_setpoint),
			// 	       (double)math::degrees(heading_error),
			// 	       (double)math::degrees(_turn_bias));

			// 	heading_error = _turn_bias;
			// 	yaw_setpoint = matrix::wrap_pi(_vehicle_yaw + heading_error); // use a turn direction bias instead

			// 	printf("========== Pos : using %.2f deg instead,  heading err: %.2f\n", (double)math::degrees(yaw_setpoint),
			// 	       (double)math::degrees(heading_error));

			// } else if (fabsf(_turn_bias) > FLT_EPSILON) {
			// 	printf("+=+=+=+=+= Pos : reset turning bias: heading_error: %.2f\n", (double)math::degrees(heading_error));
			// 	_turn_bias = 0.0; // reset the turn direction
			// }

			// if out yaw is not aligned well with the target waypoint, we need to turn on the spot:
			if (_current_state == DrivingState::DRIVING && fabsf(heading_error) > _param_rd_trans_drv_trn.get()) {
				_current_state = DrivingState::SPOT_TURNING;

			} else if (_current_state == DrivingState::SPOT_TURNING && fabsf(heading_error) < _param_rd_trans_trn_drv.get()) {
				_current_state = DrivingState::DRIVING;

				_stopped_ned = _curr_pos_ned; // next trapezoid calculation will start here, where we stopped

			}

			if (_current_state == DrivingState::SPOT_TURNING) {
				// speed profile while SPOT_TURNING - (TBD: small constant speed by parameter):
				_speed_setpoint = 0.f; // stop during spot turning

			} else {
				// speed profile while DRIVING - trapezoid calculation:

				// assume we are driving towards the target waypoint and might be decelerating.
				// shift target to the edge of the acceptance radius if arrival speed not zero:
				//float arr_dep_distance = isArrivalFast ?  distance_to_target - _param_nav_acc_rad.get() : distance_to_target;
				float arr_dep_distance = distance_to_target - _param_rd_acc_rad_margin.get() * _param_nav_acc_rad.get();
				float arr_dep_speed = isArrivalFast ?  _arrival_speed : 0.f;
				float acc_dec_limit = _param_ro_decel_limit.get();

				if (distance_from_stopped > FLT_EPSILON && distance_from_stopped < distance_to_target) {
					// we are departing from the start point and accelerating:
					arr_dep_distance = distance_from_stopped;
					arr_dep_speed = _vehicle_speed;
					acc_dec_limit = _param_ro_accel_limit.get();
				}

				if (arr_dep_distance < FLT_EPSILON) {
					arr_dep_distance = 0.f;
				}

				_speed_setpoint = math::trajectory::computeMaxSpeedFromDistance(_param_ro_jerk_limit.get(),
						  acc_dec_limit, arr_dep_distance, fabsf(arr_dep_speed));
				_speed_setpoint = math::min(_speed_setpoint, _param_ro_speed_limit.get());

				if (PX4_ISFINITE(_cruising_speed)) {
					_speed_setpoint = sign(_cruising_speed) * math::min(_speed_setpoint, fabsf(_cruising_speed));
				}
			}

			_pure_pursuit_status_pub.publish(pure_pursuit_status);

			rover_speed_setpoint_s rover_speed_setpoint{};
			rover_speed_setpoint.timestamp = timestamp;
			rover_speed_setpoint.speed_body_x = _speed_setpoint;
			rover_speed_setpoint.speed_body_y = 0.f;
			_rover_speed_setpoint_pub.publish(rover_speed_setpoint);

			rover_attitude_setpoint_s rover_attitude_setpoint{};
			rover_attitude_setpoint.timestamp = timestamp;
			rover_attitude_setpoint.yaw_setpoint = _speed_setpoint > -FLT_EPSILON ? yaw_setpoint : matrix::wrap_pi(
					yaw_setpoint + M_PI_F);
			_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);

		}  else {

			pure_pursuit_status.lookahead_distance = NAN;
			pure_pursuit_status.target_bearing = NAN;
			pure_pursuit_status.crosstrack_error = NAN;
			pure_pursuit_status.distance_to_waypoint = NAN;
			pure_pursuit_status.bearing_to_waypoint = NAN;
			_pure_pursuit_status_pub.publish(pure_pursuit_status);

			rover_speed_setpoint_s rover_speed_setpoint{};
			rover_speed_setpoint.timestamp = timestamp;
			rover_speed_setpoint.speed_body_x = 0.f;
			rover_speed_setpoint.speed_body_y = 0.f;
			//rover_speed_setpoint.state = (int)_current_state; // would be nice to have this field published
			_rover_speed_setpoint_pub.publish(rover_speed_setpoint);

			rover_attitude_setpoint_s rover_attitude_setpoint{};
			rover_attitude_setpoint.timestamp = timestamp;
			rover_attitude_setpoint.yaw_setpoint = _vehicle_yaw;
			_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);

			if (!_stopped && fabsf(_vehicle_speed) < FLT_EPSILON) {
				_stopped = true;
				_target_waypoint_ned = _curr_pos_ned;
			}

			if (_stopped && _updated_reset_counter != _reset_counter) {
				_target_waypoint_ned = _curr_pos_ned;
				_reset_counter = _updated_reset_counter;
			}
		}
	}

}

void DifferentialPosControl::updateSubscriptions()
{
	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		_vehicle_attitude_quaternion = matrix::Quatf(vehicle_attitude.q);
		_vehicle_yaw = matrix::Eulerf(_vehicle_attitude_quaternion).psi();
	}

	if (_vehicle_local_position_sub.updated()) {
		vehicle_local_position_s vehicle_local_position{};
		_vehicle_local_position_sub.copy(&vehicle_local_position);
		_updated_reset_counter = vehicle_local_position.xy_reset_counter;

		_curr_pos_ned = Vector2f(vehicle_local_position.x, vehicle_local_position.y);

		if (fabs(vehicle_local_position.ref_lat - _lat0) > 1e-9 || fabs(vehicle_local_position.ref_lon - _lon0) > 1e-9) {
			_lat0 = vehicle_local_position.ref_lat;
			_lon0 = vehicle_local_position.ref_lon;

			printf("\n********* NewProjection Ref: lat: %.7f, lon: %.7f ****\n",
			       vehicle_local_position.ref_lat, vehicle_local_position.ref_lon);
		}

		if (vehicle_local_position.v_xy_valid) {
			Vector3f velocity_ned(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz);
			Vector3f velocity_xyz = _vehicle_attitude_quaternion.rotateVectorInverse(velocity_ned);
			Vector2f velocity_2d = Vector2f(velocity_xyz(0), velocity_xyz(1));
			_vehicle_speed = velocity_2d.norm() > _param_ro_speed_th.get() ? sign(velocity_2d(0)) * velocity_2d.norm() : 0.f;

		} else {
			_vehicle_speed = NAN;
		}
	}

	if (_rover_position_setpoint_sub.updated()) {
		rover_position_setpoint_s rover_position_setpoint;
		_rover_position_setpoint_sub.copy(&rover_position_setpoint);
		_start_ned = Vector2f(rover_position_setpoint.start_ned[0], rover_position_setpoint.start_ned[1]);

		PX4_WARN("\n==== start_ned:   %f %f\n   curr_pos_ned: %f %f\n",
			 (double)_start_ned(0), (double)_start_ned(1),
			 (double)_curr_pos_ned(0), (double)_curr_pos_ned(1));

		if (!_start_ned.isAllFinite()) {

			PX4_WARN("\n=== !_start_ned.isAllFinite() - assigning current position as start: %f %f\n",
				 (double)_curr_pos_ned(0), (double)_curr_pos_ned(1));

			// if start_ned is not set, use current position as start:
			_start_ned = _curr_pos_ned;
		}

		_stopped_ned = _start_ned; // reset stopped position to start position (a.k.a. previous waypoint)

		_arrival_speed = PX4_ISFINITE(rover_position_setpoint.arrival_speed) ? rover_position_setpoint.arrival_speed : 0.f;
		_cruising_speed = PX4_ISFINITE(rover_position_setpoint.cruising_speed) ? rover_position_setpoint.cruising_speed :
				  _param_ro_speed_limit.get();
		_target_waypoint_ned = Vector2f(rover_position_setpoint.position_ned[0], rover_position_setpoint.position_ned[1]);
		_stopped = false;

		PX4_WARN("\n=== new rover_pos_setpoint:  arrival_speed=%.2f  cruising_speed=%.2f\n",
			 (double)_arrival_speed, (double)_cruising_speed);
	}
}

bool DifferentialPosControl::runSanityChecks()
{
	bool ret = true;

	if (_param_ro_speed_limit.get() < FLT_EPSILON) {
		ret = false;
	}

	return ret;
}
