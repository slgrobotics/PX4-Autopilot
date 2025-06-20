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

// PX4 includes
#include <px4_platform_common/module_params.h>

// Library includes
#include <math.h>

// uORB includes
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_control_mode.h>

using namespace time_literals;

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
	void updateLawnmowerControl();

protected:
	void updateParams() override;

private:

	void trace();

	// Variables
	hrt_abstime _timestamp{0};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::LM_TRACING_LEV>) _param_lm_tracing_lev
	)
};

}
