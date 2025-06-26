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

#include "LawnmowerControl.hpp"

namespace rover_lawnmower
{

void LawnmowerControl::advertisePublishers()
{
	//_rover_velocity_setpoint_pub.advertise();

#ifdef DEBUG_MY_DATA
	// advertise debug array:
	_dbg_array.id = 1;
	strncpy(_dbg_array.name, "rover_dbg", 10);

	_debug_array_pub.advertise();

#endif // DEBUG_MY_DATA

	_actuator_servos_pub.advertise();

}

void LawnmowerControl::publishAuxActuators()
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
	actuator_servos.timestamp_sample = _timestamp;
	actuator_servos.timestamp = _timestamp;

	for (unsigned i = 0; i < 8; ++i) {
		actuator_servos.control[i] = 0.0; //test_signal;
	}

	// Direct servo control, PCA9685 channels 5,6,7,8:
	// param set PCA9685_FUNC5 205
	// param set PCA9685_FUNC6 206
	// param set PCA9685_FUNC7 207    // 408 - right knob
	// param set PCA9685_FUNC8 208

	bool strobe_on =
		_vehicle_control_mode.flag_control_auto_enabled; // _nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;

	//bool horn_on = _alarm_dev_level > SIGMA;
	bool horn_on = !_vehicle_control_mode.flag_control_manual_enabled &&
		       (_sensor_gps_data.fix_type < _param_lm_gps_minfix.get() || !PX4_ISFINITE(_sensor_gps_data.heading));

	actuator_servos.control[2] = _cutter_setpoint;		 // PCA9685 channel 3
	actuator_servos.control[3] = _gas_engine_throttle;	 // PCA9685 channel 4
	actuator_servos.control[4] = strobe_on ? ACTUATOR_ON : ACTUATOR_OFF; // PCA9685 channel 5 - Strobe
	actuator_servos.control[5] = horn_on ? ACTUATOR_ON : ACTUATOR_OFF;   // PCA9685 channel 6 - Horn
	actuator_servos.control[6] = _alarm_dev_level;		 // PCA9685 channel 7 - duplicate R/C ch 6, right knob
	actuator_servos.control[7] = 0; //test_signal;		 // PCA9685 channel 8 - spare

	_actuator_servos_pub.publish(actuator_servos);
}

#ifdef PUBLISH_ADSB
void LawnmowerControl::publishTransponderReport(const adsbData &data)
{
	// ADSB Transponder Report - - -> MAVLink ADSB_VEHICLE message - - > QGroundControl
	//
	// How it works:
	//    We publish transponder_report message with PX4_ADSB_FLAGS_RETRANSLATE set.
	//    The src/modules/mavlink/streams/ADSB_VEHICLE.hpp subscribes to it and converts it
	//    to MAVLINK_MSG_ID_ADSB_VEHICLE message, sends it to QGC.
	//    It is visible in Mavlink Inspector and shows on the map as well.
	//    No need to set any parameters, it works out of the box.

	if (hrt_elapsed_time(&_transponder_report_last_published) > 10_ms) {

		transponder_report_s tr{};

		tr.timestamp = _timestamp;

		tr.emitter_type = data.emitter_type; // Type from ADSB_EMITTER_TYPE enum, UAV (14) in this case
		tr.squawk = data.squawk; // Squawk code, 4 digits, 0-4095
		strncpy(&tr.callsign[0], data.callsign, sizeof(tr.callsign) - 1);
		tr.callsign[sizeof(tr.callsign) - 1] = 0;
		tr.tslc = data.tslc; // Time since last communication in seconds
		tr.icao_address = data.icao_address;

		tr.lat = data.lat; // Latitude, expressed as degrees
		tr.lon = data.lon; // Longitude, expressed as degrees
		tr.altitude = data.altitude; // Altitude in meters, above WGS84 ellipsoid
		tr.altitude_type = data.altitude_type; // ADSB_ALTITUDE_TYPE_PRESSURE_QNH=0, reported by GPS=1
		tr.heading = data.heading; // Course over ground in radians, 0..2pi, 0 is north
		tr.hor_velocity	= data.hor_velocity; // The horizontal velocity in m/s
		tr.ver_velocity = data.ver_velocity; // The vertical velocity in m/s, positive is up

		tr.flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS |
			   transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING |
			   transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY |
			   transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE |
			   // Important: this report should be retransmitted via MAVLINK:
			   transponder_report_s::PX4_ADSB_FLAGS_RETRANSLATE |
			   transponder_report_s::PX4_ADSB_FLAGS_VALID_SQUAWK |
			   // If this is a UAV, we do not have a callsign
			   (transponder_report_s::ADSB_EMITTER_TYPE_UAV & data.emitter_type ?
			    0 : transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN);

		for (int i = 0; i < PX4_GUID_BYTE_LENGTH ; i++) {
			tr.uas_id[i] = 0xe0 + i; //simulate GUID
		}

#ifdef DEBUG_MY_PRINT
		//PX4_INFO_RAW("ADS-B: calsgn=%s icao_adr=0x%06x lat=%.6f lon=%.6f alt=%.2f, hdg=%.2f vel=%.2f\n",
		//		tr.callsign, tr.icao_address, tr.lat, tr.lon, (double)tr.altitude,
		//		(double)tr.heading, (double)tr.hor_velocity);
#endif // DEBUG_MY_PRINT

		_transponder_report_pub.publish(tr);

		_transponder_report_last_published = _timestamp;
	}
}
#endif // PUBLISH_ADSB


#ifdef DEBUG_MY_DATA

void LawnmowerControl::publishDebugArray()
{
	_dbg_array.timestamp = _timestamp; // hrt_elapsed_time(&_app_started_time);

	_debug_array_pub.publish(_dbg_array);
}

#endif // DEBUG_MY_DATA

} // namespace rover_lawnmower

