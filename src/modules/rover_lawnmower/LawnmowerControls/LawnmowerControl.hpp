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

// uORB includes
//#include <uORB/topics/parameter_update.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/pure_pursuit_status.h>

#ifdef DEBUG_MY_DATA
#include <uORB/topics/debug_array.h>
#endif // DEBUG_MY_DATA

using namespace time_literals;
using namespace matrix;

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

	void advertisePublishers();
	void vehicleControl();

#ifdef DEBUG_MY_PRINT
	void debugPrint();
	void debugPrintAuto();
	void debugPrintManual();

	hrt_abstime _debug_print_last_called{0};

	int _tracing_lev{0}; // Tracing level, set by parameter
#endif // DEBUG_MY_PRINT

#ifdef DEBUG_MY_DATA
	void debugPublishData();
	void publishDebugArray();

	struct debug_array_s _dbg_array;
	orb_advert_t _pub_dbg_array;

	hrt_abstime _debug_data_last_called{0};
#endif // DEBUG_MY_DATA

	/**
	 * @brief Update uORB subscriptions
	 */
	void updateSubscriptions();

	// uORB subscriptions
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _pure_pursuit_status_sub{ORB_ID(pure_pursuit_status)};

	// uORB publications
	//uORB::Publication<rover_velocity_setpoint_s> _rover_velocity_setpoint_pub{ORB_ID(rover_velocity_setpoint)};

	// Variables
	hrt_abstime _timestamp{0}; // Current timestamp
	float _dt{0.f};	// Time since last update in seconds since last call to updateLawnmowerControl()

	vehicle_control_mode_s _vehicle_control_mode{};
	pure_pursuit_status_s _pure_pursuit_status{};

	Vector2f _curr_pos_ned{};
	Vector2f _start_ned{};
	float _vehicle_yaw{0.f};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::LM_TRACING_LEV>) _param_lm_tracing_lev
	)
};

}
