/****************************************************************************
 *
 *   Copyright (C) 2024 PX4 Development Team. All rights reserved.
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

/******************************************************************
 * Test code for the Stanley Pursuit algorithm
 * Run this test only using "make tests TESTFILTER=StanleyPursuit"
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
 * 			        ^	       P
 * 				|
 * 				| D
 *  	   (-1.5708 rad) <----- ⨂ -----> E (1.5708 rad)
 * 				|
 * 				|
 * 				⌄
 * 			(+- 3.14159 rad)
 *
 * NOTE:
 * 	The tuning parameters for the stanley pursuit algorithm are set to the following for all tests:
 * 	   ST_XTRACK_GAIN = 1.f
 * 	   ST_SOFTENING = 0.1.f
 *
******************************************************************/

#include <gtest/gtest.h>
#include <lib/stanley_pursuit/StanleyPursuit.hpp>

using namespace matrix;

class StanleyPursuitTest : public ::testing::Test
{
public:
	StanleyPursuit stanley_pursuit{nullptr};
};

//#ifdef QQQ
TEST_F(StanleyPursuitTest, InvalidSpeed)
{
	//      V   C
	//         /
	//  	  /
	//	 /
	//	P
	const Vector2f curr_wp_ned(10.f, 10.f);
	const Vector2f prev_wp_ned(0.f, 0.f);
	const Vector2f curr_pos_ned(10.f, 0.f);
	// NaN speed
	const float desired_heading = stanley_pursuit.calcDesiredHeading(curr_wp_ned, prev_wp_ned, curr_pos_ned, NAN);
	EXPECT_FALSE(PX4_ISFINITE(desired_heading));
}

TEST_F(StanleyPursuitTest, InvalidWaypoints)
{
	//	V   C
	//         /
	//  	  /
	//	 /
	//	P
	const Vector2f curr_wp_ned(10.f, 10.f);
	const Vector2f prev_wp_ned(0.f, 0.f);
	const Vector2f curr_pos_ned(10.f, 0.f);
	const float vehicle_speed{5.f};
	// Prev WP is NAN
	const float desired_heading1 = stanley_pursuit.calcDesiredHeading(curr_wp_ned, Vector2f(NAN, NAN), curr_pos_ned,
				       vehicle_speed);
	// Curr WP is NAN
	const float desired_heading2 = stanley_pursuit.calcDesiredHeading(Vector2f(NAN, NAN), prev_wp_ned, curr_pos_ned,
				       vehicle_speed);

	// Curr Pos is NAN
	const float desired_heading3 = stanley_pursuit.calcDesiredHeading(curr_wp_ned, prev_wp_ned, Vector2f(NAN, NAN),
				       vehicle_speed);
	EXPECT_FALSE(PX4_ISFINITE(desired_heading1));
	EXPECT_FALSE(PX4_ISFINITE(desired_heading2));
	EXPECT_FALSE(PX4_ISFINITE(desired_heading3));
}

/*
TEST_F(StanleyPursuitTest, OutOfLookahead)
{
	const float vehicle_speed{5.f};
	//	V   C
	//         /
	//  	  /
	//	 /
	//	P
	const float desired_heading1 = stanley_pursuit.calcDesiredHeading(Vector2f(10.f, 10.f), Vector2f(0.f, 0.f),
				       Vector2f(10.f,
						0.f),
				       vehicle_speed);
	//	    V
	//
	//	P ----- C
	const float desired_heading2 = stanley_pursuit.calcDesiredHeading(Vector2f(0.f, 20.f), Vector2f(0.f, 0.f),
				       Vector2f(10.f,
						10.f),
				       vehicle_speed);
	EXPECT_NEAR(desired_heading1, M_PI_2_F + M_PI_4_F, FLT_EPSILON); // Fallback: Bearing to closest point on path
	EXPECT_NEAR(desired_heading2, M_PI_F, FLT_EPSILON); 		 // Fallback: Bearing to closest point on path
}
*/
//#endif // QQQ

TEST_F(StanleyPursuitTest, WaypointOverlap)
{
	const float vehicle_speed{5.f};

	//	    C/P
	//
	//
	//
	//	V
	const float desired_heading1_1 = stanley_pursuit.calcDesiredHeading(Vector2f(10.f, 10.f), Vector2f(10.f, 10.f),
					 Vector2f(0.f, 0.f), vehicle_speed);
	const float desired_heading1_2 = stanley_pursuit.calcDesiredHeading(Vector2f(10.f, -10.f), Vector2f(10.f, -10.f),
					 Vector2f(0.f, 0.f), vehicle_speed);
	const float desired_heading1_3 = stanley_pursuit.calcDesiredHeading(Vector2f(-10.f, -10.f), Vector2f(-10.f, -10.f),
					 Vector2f(0.f, 0.f), vehicle_speed);
	const float desired_heading1_4 = stanley_pursuit.calcDesiredHeading(Vector2f(-10.f, 10.f), Vector2f(-10.f, 10.f),
					 Vector2f(0.f, 0.f), vehicle_speed);
	// same with 60 and 150 degrees:
	const float desired_heading2_1 = stanley_pursuit.calcDesiredHeading(Vector2f(10.f, 10.f * sqrt(3.0f)), Vector2f(10.f,
					 10.f * sqrt(3.0f)), Vector2f(0.f, 0.f), vehicle_speed);
	const float desired_heading2_2 = stanley_pursuit.calcDesiredHeading(Vector2f(10.f, -10.f * sqrt(3.0f)), Vector2f(10.f,
					 -10.f * sqrt(3.0f)), Vector2f(0.f, 0.f), vehicle_speed);
	const float desired_heading2_3 = stanley_pursuit.calcDesiredHeading(Vector2f(-10.f * sqrt(3.0f), -10.f),
					 Vector2f(-10.f * sqrt(3.0f), -10.f), Vector2f(0.f, 0.f), vehicle_speed);
	const float desired_heading2_4 = stanley_pursuit.calcDesiredHeading(Vector2f(-10.f * sqrt(3.0f), 10.f),
					 Vector2f(-10.f * sqrt(3.0f), 10.f), Vector2f(0.f, 0.f), vehicle_speed);

	//	    V
	//
	//
	//
	//	C/P
	const float desired_heading2 = stanley_pursuit.calcDesiredHeading(Vector2f(0.f, 0.f), Vector2f(0.f, 0.f), Vector2f(10.f,
				       10.f),
				       vehicle_speed);
	const float desired_heading3 = stanley_pursuit.calcDesiredHeading(Vector2f(0.f, 0.f), Vector2f(0.f, 0.f), Vector2f(10.f,
				       -10.f),
				       vehicle_speed);
	// 30 (150) degrees angle:
	const float desired_heading4 = stanley_pursuit.calcDesiredHeading(Vector2f(0.f, 0.f), Vector2f(0.f, 0.f),
				       Vector2f(10.f * sqrt(3.0f),
						-10.f),
				       vehicle_speed);

	// Fallback: Bearing to closest point when P and C overlap:
	EXPECT_NEAR(desired_heading1_1, M_PI_4_F, FLT_EPSILON);
	EXPECT_NEAR(desired_heading1_2, -M_PI_4_F, FLT_EPSILON);
	EXPECT_NEAR(desired_heading1_3, -M_PI_4_F * 3.0f, FLT_EPSILON);
	EXPECT_NEAR(desired_heading1_4, M_PI_4_F * 3.0f, FLT_EPSILON);

	EXPECT_NEAR(desired_heading2_1, M_PI_F / 3.0f, FLT_EPSILON);
	EXPECT_NEAR(desired_heading2_2, -M_PI_F / 3.0f, FLT_EPSILON);
	EXPECT_NEAR(desired_heading2_3, -M_PI_F * 5.0f / 6.0f, FLT_EPSILON * 4.0f);
	EXPECT_NEAR(desired_heading2_4, M_PI_F * 5.0f / 6.0f, FLT_EPSILON * 4.0f);

	EXPECT_NEAR(desired_heading2, -(M_PI_4_F + M_PI_2_F), FLT_EPSILON);
	EXPECT_NEAR(desired_heading3, (M_PI_F * 3.0f / 4.0f), FLT_EPSILON);
	EXPECT_NEAR(desired_heading4, (M_PI_F * 5.0f / 6.0f), FLT_EPSILON * 4.0f);
}

TEST_F(StanleyPursuitTest, HighSpeedMeansParallelCourse)
{
	const float vehicle_speed{1.e+9f};

	//	     V
	//	P ------ C
	const float desired_heading1 = stanley_pursuit.calcDesiredHeading(Vector2f(0.f, 20.f), Vector2f(0.f, 0.f), Vector2f(5.f,
				       10.f), vehicle_speed);
	//	     V
	//	C ------ P
	const float desired_heading2 = stanley_pursuit.calcDesiredHeading(Vector2f(0.f, 0.f), Vector2f(0.f, 20.f), Vector2f(5.f,
				       10.f), vehicle_speed);

	// 30 degrees, swapping P and C:
	const float desired_heading3 = stanley_pursuit.calcDesiredHeading(Vector2f(10.f * sqrt(3.0f), 10.f), Vector2f(0.f, 0.f),
				       Vector2f(5.f, 5.f), vehicle_speed);
	const float desired_heading4 = stanley_pursuit.calcDesiredHeading(Vector2f(0.f, 0.f), Vector2f(10.f * sqrt(3.0f), 10.f),
				       Vector2f(5.f, 5.f), vehicle_speed);
	// same with negative Y:
	const float desired_heading5 = stanley_pursuit.calcDesiredHeading(Vector2f(10.f * sqrt(3.0f), -10.f), Vector2f(0.f,
				       0.f), Vector2f(5.f, 5.f), vehicle_speed);
	const float desired_heading6 = stanley_pursuit.calcDesiredHeading(Vector2f(0.f, 0.f), Vector2f(10.f * sqrt(3.0f),
				       -10.f), Vector2f(5.f, 5.f), vehicle_speed);

	EXPECT_NEAR(desired_heading1, M_PI_2_F, FLT_EPSILON);
	EXPECT_NEAR(desired_heading2, -M_PI_2_F, FLT_EPSILON);
	EXPECT_NEAR(desired_heading3, M_PI_F / 6.0f, FLT_EPSILON);
	EXPECT_NEAR(desired_heading4, -M_PI_F * 5.0f / 6.0f, FLT_EPSILON * 4.0f);

	EXPECT_NEAR(desired_heading5, -M_PI_F / 6.0f, FLT_EPSILON);
	EXPECT_NEAR(desired_heading6, M_PI_F * 5.0f / 6.0f, FLT_EPSILON * 4.0f);
}

TEST_F(StanleyPursuitTest, CourseContributionDueToCrosstrack)
{
	const float vehicle_speed{1.0f};

	//	     V       10 cm crosstrack
	//	P ------ C
	const float desired_heading1 = stanley_pursuit.calcDesiredHeading(Vector2f(0.f, 20.f), Vector2f(0.f, 0.f),
				       Vector2f(0.1f, 10.f), vehicle_speed);
	//	P ------ C
	//	     V       10 cm crosstrack
	const float desired_heading2 = stanley_pursuit.calcDesiredHeading(Vector2f(0.f, 20.f), Vector2f(0.f, 0.f),
				       Vector2f(-0.1f, 10.f), vehicle_speed);
	//	     C
	//       V  /
	//         /
	//	  P
	const float desired_heading3 = stanley_pursuit.calcDesiredHeading(Vector2f(20.f, 20.f), Vector2f(0.f, 0.f),
				       Vector2f(11.0f, 9.0f), vehicle_speed);
	//	     C
	//          /
	//         /  V
	//	  P
	const float desired_heading4 = stanley_pursuit.calcDesiredHeading(Vector2f(20.f, 20.f), Vector2f(0.f, 0.f),
				       Vector2f(9.0f, 11.0f), vehicle_speed);
	// V         C
	// (far)    /
	//         /
	//	  P
	const float desired_heading5 = stanley_pursuit.calcDesiredHeading(Vector2f(20.f, 20.f), Vector2f(0.f, 0.f),
				       Vector2f(20.0f, 0.0f), vehicle_speed);
	//           C
	//          /
	//         /
	//	  P        V (far)
	const float desired_heading6 = stanley_pursuit.calcDesiredHeading(Vector2f(20.f, 20.f), Vector2f(0.f, 0.f),
				       Vector2f(0.0f, 20.0f), vehicle_speed);

	// expect the heading to lean about 5 degrees towards the C:
	EXPECT_NEAR(desired_heading1, M_PI_2_F + 0.09f, 0.001f);
	EXPECT_NEAR(desired_heading2, M_PI_2_F - 0.09f, 0.001f);
	EXPECT_NEAR(desired_heading3, M_PI_2_F + 0.13f, 0.01f);		// 97 degrees
	EXPECT_NEAR(desired_heading4, -0.13f, 0.01f);			// -7 degrees
	EXPECT_NEAR(desired_heading5, M_PI_4_F * 3.0f - 0.08f, 0.01f);	// 130 degrees
	EXPECT_NEAR(desired_heading6, -M_PI_4_F + 0.08f, 0.01f);	// -40 degrees
}

#ifdef QQQ
TEST_F(StanleyPursuitTest, CurrAndPrevSameNorthCoordinate)
{
	const float vehicle_speed{5.f};
	//	P -- V -- C
	const float desired_heading1 = stanley_pursuit.calcDesiredHeading(Vector2f(0.f, 20.f), Vector2f(0.f, 0.f), Vector2f(0.f,
				       10.f),
				       vehicle_speed);

	//	     V
	//	P ------ C
	const float desired_heading2 = stanley_pursuit.calcDesiredHeading(Vector2f(0.f, 20.f), Vector2f(0.f, 0.f),
				       Vector2f(5.f / sqrtf(2.f), 10.f),
				       vehicle_speed);
	//	     V
	//	C ------ P
	const float desired_heading3 = stanley_pursuit.calcDesiredHeading(Vector2f(0.f, 0.f), Vector2f(0.f, 20.f),
				       Vector2f(5.f / sqrtf(2.f), 10.f),
				       vehicle_speed);
	//	     V
	//
	//	P ------ C
	const float desired_heading4 = stanley_pursuit.calcDesiredHeading(Vector2f(0.f, 20.f), Vector2f(0.f, 0.f),
				       Vector2f(10.f,
						10.f),
				       vehicle_speed);

	EXPECT_NEAR(desired_heading1, M_PI_2_F, FLT_EPSILON);
	EXPECT_NEAR(desired_heading2, M_PI_2_F + M_PI_4_F, FLT_EPSILON);
	EXPECT_NEAR(desired_heading3, -(M_PI_2_F + M_PI_4_F), FLT_EPSILON);
	EXPECT_NEAR(desired_heading4, M_PI_F, FLT_EPSILON); // Fallback: Bearing to closest point on path
}
#endif // QQQ
