/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file rover_pos_control_params.c
 *
 * Parameters defined by the position control task for ground rovers
 *
 * This is a modification of the fixed wing params and it is designed for ground rovers.
 * It has been developed starting from the fw  module, simplified and improved with dedicated items.
 *
 * All the ackowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Marco Zorzi <mzorzi@student.ethz.ch>
 */

/*
 * Controller parameters, accessible via MAVLink
 */

//========================= PID controller for Line Following ===================================================

/**
 * When in Line Following State, use Rate Controller for yaw output
 *
 * 0 - no PID or Rate Control, 1 - just PID, 2 - PID and Rate Control
 *
 * @min 0
 * @max 2
 * @group Rover Position Control
 */
PARAM_DEFINE_INT32(GND_LF_USE_RATE, 2);

/**
 * Line Following proportional gain for heading error when GND_LF_USE_RATE=0
 *
 * This is the proportional gain for the Line Following closed loop controller
 *
 * @unit norm
 * @min 0.005
 * @max 50.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_LF_P, 1.0f);

/**
 * Derivative gain for heading error controller
 *
 * This is the derivative gain for the Heading closed loop controller
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(RD_YAW_D, 0.0f);

//===============================================================================================================

/**
 * Default ground speed setpoint for missions
 *
 * This is the maximum speed that can be used by the controller.
 *
 * @unit m/s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.01
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(RD_MISS_SPD_DEF, 1.5f);

/**
 * Thrust limit max
 *
 * This is the maximum thrust % that can be used by the controller.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_THR_MAX, 1.0f);

/**
 * Thrust limit min
 *
 * This is the minimum thrust % that can be used by the controller.
 * Set to 0 for rover
 *
 * @unit norm
 * @min -1.0
 * @max 0.0
 * @decimal 2
 * @increment 0.01
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_THR_MIN, -1.0f);

/**
 * Control mode for speed measurement
 *
 * This allows the user to choose between ekf2 or gps when measuring speed
 *
 * @min -1
 * @max 1
 * @value 0 use ekf2 speed estimate
 * @value 1 use RTK gps speed, if available
 * @group Rover Position Control
 */
PARAM_DEFINE_INT32(GND_SP_MEAS_MODE, 0);

/**
 * Control mode for heading measurement
 *
 * This allows the user to choose between ekf2 or gps when measuring heading
 *
 * @min -1
 * @max 1
 * @value 0 use ekf2 heading estimate
 * @value 1 use RTK gps heading, if available
 * @group Rover Position Control
 */
PARAM_DEFINE_INT32(GND_HD_MEAS_MODE, 1);

/**
 * Control mode for coordinates and heading being substituded in EKF2 data by GPS Lat/Lon/Heading values
 *
 * This allows the user to choose between ekf2 or gps when calculating position and orientation
 *
 * @min -1
 * @max 1
 * @value 0 use ekf2 lat/lon/heading estimates
 * @value 1 substitute RTK gps lat/lon/heading in EKF2 data, if available
 * @group Rover Position Control
 */
PARAM_DEFINE_INT32(GND_EKF_OVERRIDE, 1);

/**
 * Speed proportional gain
 *
 * This is the proportional gain for the speed closed loop controller
 *
 * @unit norm
 * @min 0.005
 * @max 50.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_SPEED_P, 0.5f);

/**
 * Speed Integral gain
 *
 * This is the integral gain for the speed closed loop controller
 *
 * @unit norm
 * @min 0.00
 * @max 50.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_SPEED_I, 1.0f);

/**
 * Speed derivative gain
 *
 * This is the derivative gain for the speed closed loop controller
 *
 * @unit norm
 * @min 0.00
 * @max 50.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_SPEED_D, 0.0f);

/**
 * Speed integral maximum value
 *
 * This is the maxim value the integral can reach to prevent wind-up.
 *
 * @unit norm
 * @min 0.005
 * @max 50.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_SPEED_IMAX, 0.3f);

/**
 * Maximum output (thrust effort) of speed PID
 *
 *
 * @unit m/s
 * @min 0.0
 * @max 10
 * @decimal 1
 * @increment 0.1
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_SPEED_MAX, 10.0f);

/**
 * An EMA to smooth velocity setpoint for the speed PID's input
 *
 * Removes quick jolts in velocity setpoint before PID
 *
 * @min 0
 * @max 50
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_SPEED_SP_EMA, 5);

// -----------------------------------------------------------------------------------------------------------------------------
// added to support heavy differential drive rover:

// ================  Acceleration (wp departure) and deceleration (wp arrival) tuning =========================

/**
 * Distance to accelerate from a waypoint before using Stanley Pursuit control
 *
 * @unit m
 * @min 0.0
 * @max 200.0
 * @decimal 1
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_ACCEL_DIST, 2.0f);

/**
 * Distance to decelerate straight to a waypoint not using any Pursuit logic
 *
 * @unit m
 * @min 1.0
 * @max 200.0
 * @decimal 1
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_DECEL_DIST, 1.0f);

/**
 * Arrival minimum speed factor
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.05
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_SPD_ARR_MIN, 0.1f);

/**
 * Departure minimum speed factor
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.05
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_SPD_DEP_MIN, 0.1f);

// ================  Tresholds for turns and stopping ===========================================================

/**
 * Waypoint precision distance
 *
 * This is the distance at which the next waypoint is activated. This should be set
 * to about 2-4x of robot's wheel base and not smaller than one meter (due to GPS accuracy).
 *
 *
 * @unit m
 * @min 0.01
 * @max 5.0
 * @decimal 2
 * @increment 0.05
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_WP_PRECISN, 0.5f);

/**
 * When turning at waypoint, apply some forward or backward movement.
 *
 * @unit m/s
 * @min -1.0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_TURN_SPEED, 0.05f);

/**
 * When turning at waypoint, what to consider a success
 *
 * @unit deg
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @increment 0.1
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_TURN_PRECISN, 3.0f);

/**
 * When turning at waypoint, how many seconds to wait after turn goal is reached.
 *
 * This allows PID oscillations to cease in aggressive turns
 *
 * @unit s
 * @min 0.0
 * @max 10.0
 * @decimal 1
 * @increment 0.1
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_TURN_WAIT, 0.0f);

/**
 * When stoping at waypoint, what to consider a success
 *
 * @unit m/s
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_STOP_PRECISN, 0.05f);


// ================  More parameters for heading conrol ===========================================================

/**
 * Heading declination (Z offset) to compensate for compass shift in turns, arrival, departure
 *
 * @unit deg
 * @min -45.0
 * @max 45.0
 * @decimal 1
 * @increment 0.1
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_HEADING_DECL, 0.0f);

/**
 * When departing or arriving, limits the yaw rate setpoint for Rate Control
 *
 * @unit deg/s
 * @min 0.0
 * @max 90.0
 * @decimal 3
 * @increment 0.1
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_RATE_AD_TRIM, 30f);

/**
 * When departing or arriving, controls yaw rate setpoint for Rate Control
 *
 * @unit norm
 * @min 0.0
 * @max 100.0
 * @decimal 3
 * @increment 0.1
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_RATE_AD_SC, 2.0f);

/**
 * When turning at waypoint, set yaw rate setpoint for Rate Control
 *
 * @unit deg/s
 * @min 0.1
 * @max 90.0
 * @decimal 1
 * @increment 0.1
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_TURN_RATE, 30f);


// ================  Line following - Yaw RateControl ====================================================

/**
 * Rover Rate Proportional Gain
 *
 * @unit norm
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_RATE_P, 0.6f);

/**
 * Rover Rate Integral Gain
 *
 * @unit norm
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_RATE_I, 0.2f);

/**
 * Rover Rate Differential Gain
 *
 * @unit norm
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_RATE_D, 0.0f);

/**
 * Rover Rate Feed Forward Gain
 *
 * @unit norm
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_RATE_FF, 0.0f);

/**
 * Rover Rate Maximum Integral Limit
 *
 * @unit norm
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_RATE_IMAX, 0.5f);

/**
 * Rover Rate Minimum Speed to compute Integral
 *
 * @unit m/s
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_RATE_IMINSPD, 0.1f);

// ================ Scalers to convert body thrust and torque to velocity and yaw rate: ============

/**
 * Scaler to convert body thrust to diff drive kinematics velocity
 *
 * @unit norm
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_THRUST_SC, 1.0f);

/**
 * Scaler to convert body torque to diff drive kinematics yaw rate
 *
 * @unit norm
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_TORQUE_SC, 1.0f);

// ================  Other parameters ===========================================================

/**
 * Tracing level, 5 - High  0 - no tracing
 *
 * @min 0
 * @max 5
 * @group Rover Position Control
 */
PARAM_DEFINE_INT32(GND_TRACING_LEV, 2);

/**
 * Minimal GPS fix when heading may be taken from GPS course-over-ground (cog)
 *
 * @min 3
 * @max 6
 * @group Rover Position Control
 */
PARAM_DEFINE_INT32(GND_GPS_MINFIX, 6);

/**
 * Gas engine throttle level on idle
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_GTL_IDLE, 0.0f);

/**
 * Gas engine throttle level in departure
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_GTL_DEPART, 0.8f);

/**
 * Gas engine throttle level in turns
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_GTL_TURN, 0.2f);

/**
 * Gas engine throttle level in arrival
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_GTL_ARRIVE, 0.8f);

/**
 * Gas engine throttle level on straight run
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_GTL_STRAIGHT, 1.0f);

/**
 * Minimum yaw responsiveness when Gas engine throttle is at max
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_GTL_YAWF_MIN, 0.5f);

/**
 * Smoothing EMA period for speed measurements
 *
 * This allows the user to remove jitter from speed measurements
 *
 * @min 0
 * @max 100
 * @group Rover Position Control
 */
PARAM_DEFINE_INT32(GND_EMA_M_PERIOD, 0);

/**
 * Smoothing EMA period for thrust/torque outputs
 *
 * This allows the user to remove jitter from throttle and torque actuators
 *
 * @min 0
 * @max 100
 * @group Rover Position Control
 */
PARAM_DEFINE_INT32(GND_EMA_O_PERIOD, 0);

/**
 * When in manual mode, use PIDs for speed and yaw outputs
 *
 * To debug and tune response of throttle and torque actuators
 *
 * @min 0
 * @max 1
 * @group Rover Position Control
 */
PARAM_DEFINE_INT32(GND_MAN_USE_PID, 0);

/**
 * When in manual mode, zero torque/yaw outputs, drive straight
 *
 * To help balancing left and right wheel actuators
 *
 * @min 0
 * @max 1
 * @group Rover Position Control
 */
PARAM_DEFINE_INT32(GND_MAN_STRAIGHT, 0);

// ------------------------------------

/**
 * When within corridor(close to line), multiply torque by RD_RATE_FTRQ
 *
 * @unit m
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(RD_RATE_CORW, 0.3f);

/**
 * A "torque correction" - torque multiplier when we just need to go straight
 *
 * @unit norm
 * @min 0
 * @max 10.0
 * @decimal 2
 * @increment 0.01
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(RD_RATE_FTRQ, 5.0f);

/**
 * A "zero torque correction" - added to torque value at all times
 *
 * @unit norm
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(RD_RATE_ZTRQ, 0.0f);

/**
 * Manual mode yaw rate scale
 *
 * @unit norm
 * @min 0.01
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(RD_MAN_YAW_SCALE, 0.25f);

// end heavy rover additions
// -----------------------------------------------------------------------------------------------------------------------------
