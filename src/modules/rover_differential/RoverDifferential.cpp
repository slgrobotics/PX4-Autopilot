/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "RoverDifferential.hpp"

RoverDifferential::RoverDifferential() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	updateParams();
	_rover_differential_setpoint_pub.advertise();
}

bool RoverDifferential::init()
{
	ScheduleOnInterval(10_ms); // 100 Hz
	return true;
}

void RoverDifferential::updateParams()
{
	ModuleParams::updateParams();
	_max_yaw_rate = _param_rd_max_yaw_rate.get() * M_DEG_TO_RAD_F;
}

void RoverDifferential::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	updateSubscriptions();

	// Generate and publish attitude and velocity setpoints
	hrt_abstime timestamp = hrt_absolute_time();

	switch (_nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL: {
			manual_control_setpoint_s manual_control_setpoint{};

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				rover_differential_setpoint_s rover_differential_setpoint{};
				rover_differential_setpoint.timestamp = timestamp;
				rover_differential_setpoint.forward_speed_setpoint = NAN;
				rover_differential_setpoint.forward_speed_setpoint_normalized = manual_control_setpoint.throttle;
				rover_differential_setpoint.yaw_setpoint = NAN;
				rover_differential_setpoint.yaw_rate_setpoint_normalized = manual_control_setpoint.roll * _param_rd_man_yaw_scale.get();
				rover_differential_setpoint.yaw_rate_setpoint = NAN;
				_rover_differential_setpoint_pub.publish(rover_differential_setpoint);
			}
		} break;

	case vehicle_status_s::NAVIGATION_STATE_ACRO: {
			manual_control_setpoint_s manual_control_setpoint{};

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				rover_differential_setpoint_s rover_differential_setpoint{};
				rover_differential_setpoint.timestamp = timestamp;
				rover_differential_setpoint.forward_speed_setpoint = NAN;
				rover_differential_setpoint.forward_speed_setpoint_normalized = manual_control_setpoint.throttle;
				rover_differential_setpoint.yaw_rate_setpoint = math::interpolate<float>(manual_control_setpoint.roll,
						-1.f, 1.f, -_max_yaw_rate, _max_yaw_rate);
				rover_differential_setpoint.yaw_rate_setpoint_normalized = NAN;
				rover_differential_setpoint.yaw_setpoint = NAN;
				_rover_differential_setpoint_pub.publish(rover_differential_setpoint);
			}

		} break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
		_rover_differential_guidance.computeGuidance(_vehicle_yaw, _vehicle_forward_speed, _nav_state);
		break;

	default: // Unimplemented nav states will stop the rover
		rover_differential_setpoint_s rover_differential_setpoint{};
		rover_differential_setpoint.forward_speed_setpoint = NAN;
		rover_differential_setpoint.forward_speed_setpoint_normalized = 0.f;
		rover_differential_setpoint.yaw_rate_setpoint = NAN;
		rover_differential_setpoint.yaw_rate_setpoint_normalized = 0.f;
		rover_differential_setpoint.yaw_setpoint = NAN;
		_rover_differential_setpoint_pub.publish(rover_differential_setpoint);
		break;
	}

	_rover_differential_control.computeMotorCommands(_vehicle_yaw, _vehicle_yaw_rate, _vehicle_forward_speed);

}

void RoverDifferential::updateSubscriptions()
{

	if (_parameter_update_sub.updated()) {
		parameter_update_s parameter_update;
		_parameter_update_sub.copy(&parameter_update);
		updateParams();
	}

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status{};
		_vehicle_status_sub.copy(&vehicle_status);
		_nav_state = vehicle_status.nav_state;
	}

	if (_vehicle_angular_velocity_sub.updated()) {
		vehicle_angular_velocity_s vehicle_angular_velocity{};
		_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity);
		_vehicle_yaw_rate = vehicle_angular_velocity.xyz[2];
	}

	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		_vehicle_attitude_quaternion = matrix::Quatf(vehicle_attitude.q);
		_vehicle_yaw = matrix::Eulerf(_vehicle_attitude_quaternion).psi();
	}

	if (_vehicle_local_position_sub.updated()) {
		vehicle_local_position_s vehicle_local_position{};
		_vehicle_local_position_sub.copy(&vehicle_local_position);
		Vector3f velocity_in_local_frame(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz);
		Vector3f velocity_in_body_frame = _vehicle_attitude_quaternion.rotateVectorInverse(velocity_in_local_frame);
		_vehicle_forward_speed = velocity_in_body_frame(0);
	}
}

int RoverDifferential::task_spawn(int argc, char *argv[])
{
	RoverDifferential *instance = new RoverDifferential();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int RoverDifferential::custom_command(int argc, char *argv[])
{
	return print_usage("unk_timestampn command");
}

int RoverDifferential::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Rover Differential controller.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rover_differential", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int rover_differential_main(int argc, char *argv[])
{
	return RoverDifferential::main(argc, argv);
}
