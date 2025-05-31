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

#pragma once

#include <matrix/math.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>

using namespace matrix;

/**
 * @file StanleyPursuit.hpp
 *
 * Implementation of stanley pursuit guidance logic.
 *
 * Acknowledgements and References:
 *
 *    This implementation has been built for PX4 based on the idea from [1] (not including any code).
 *
 *    [1] https://ieeexplore.ieee.org/document/4282788
 *    [2] https://ai.stanford.edu/~gabeh/papers/hoffmann_stanley_control07.pdf
 *    [3] https://dingyan89.medium.com/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081
 *
 * Stanley pursuit is a path following algorithm that in this implementation tries to eliminate two discrepancies:
 *    (a) the angle between the yaw of the vehicle and the P--C line between the Previous and Current waypoints
 *    (b) crosstrack distance (between the vehicle and the P--C line)
 *
 * Graphic interpretation:
 * 	Legend:
 * 		C: Current waypoint
 * 		P: Previous waypoint
 * 		V: Vehicle
 * 		|: Line segment
 * 							 C
 *                                                      /
 * 		  				       /  hdg
 * 						      /  |      s-err
 * 						     /   |   /     .
 * 						    /    |  /   .
 * 						   /     | /  .
 * 						  /. _   |/ .
 * 	Orientation:				 /     ' V
 * 	         	    N (0 rad)		/  x-trk
 * 			        ^ +X (NED)     P
 * 				|
 * 				| D
 *  	   (-1.5708 rad) <----- ⨂ -----> E (1.5708 rad)
 * 				|         +Y (NED)
 * 				|
 * 				⌄
 * 			(+- 3.14159 rad)         +Z (NED) pointing DOWN
 *
 * Input:  Current/prev waypoint and the vehicle position in NED frame as well as the vehicle ground speed.
 * Output: Calculates the desired heading towards the P--C line segment.
*/
class StanleyPursuit : public ModuleParams
{
public:
	StanleyPursuit(ModuleParams *parent);
	~StanleyPursuit() = default;

	/**
	 * @brief Return steering angle towards the line segment from the previous to the current waypoint.
	 * Exceptions:
	 * 	Will return NAN if input is invalid.
	 * @param curr_wp_ned North/East coordinates of current waypoint in NED frame [m].
	 * @param prev_wp_ned North/East coordinates of previous waypoint in NED frame [m].
	 * @param curr_pos_ned North/East coordinates of current position of the vehicle in NED frame [m].
	 * @param vehicle_speed Vehicle ground speed [m/s].
	 *
	 * @param ST_XTRACK_GAIN Tuning parameter [-]
	 * @param ST_SOFTENING Maximum lookahead distance [m]
	 */
	float calcDesiredHeading(const Vector2f &curr_wp_ned, const Vector2f &prev_wp_ned, const Vector2f &curr_pos_ned,
				 float vehicle_speed);

protected:
	/**
	 * @brief Update the parameters of the module.
	 */
	void updateParams() override;

	struct {
		param_t xtrack_gain;
		param_t softening_factor;
	} _param_handles{};

	struct {
		float xtrack_gain{1.f};
		float softening_factor{.1f};
	} _params{};
private:
};
