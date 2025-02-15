/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file ms5611.cpp
 * Driver for the MS5611 and MS5607 barometric pressure sensor connected via I2C or SPI.
 */

// - modified by Sergei Grichine Sept 2020

#include "MS5611.hpp"
#include "ms5611.h"

#include <cdev/CDev.hpp>

MS5611::MS5611(device::Device *interface, ms5611::prom_u &prom_buf, const I2CSPIDriverConfig &config) :
	I2CSPIDriver(config),
	_interface(interface),
	_prom(prom_buf.s),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_measure_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": measure")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err"))
{
	switch (config.devid_driver_index) {
	case DRV_BARO_DEVTYPE_MS5611:
		_device_type = MS5611_DEVICE;
		break;

	case DRV_BARO_DEVTYPE_MS5607:
	default:
		_device_type = MS5607_DEVICE;
		break;
	}
}

MS5611::~MS5611()
{
	// free perf counters
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);

	delete _interface;
}

int
MS5611::init()
{
	int ret;

	/* do a first measurement cycle to populate reports with valid data */
	_measure_phase = 0;

	while (true) {
		/* do temperature first */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		px4_usleep(MS5611_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* now do a pressure measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		px4_usleep(MS5611_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		if (_device_type == MS5607_DEVICE) {
			if (_last_pressure < 52'000.f) {
				/* This is likely not this device, abort */
				ret = -EINVAL;
				break;

			} else if (_last_pressure > 150'000.f) {
				/* This is likely not this device, abort */
				ret = -EINVAL;
				break;
			}
		}

		switch (_device_type) {
		default:

		/* fall through */
		case MS5611_DEVICE:
			_interface->set_device_type(DRV_BARO_DEVTYPE_MS5611);
			break;

		case MS5607_DEVICE:
			_interface->set_device_type(DRV_BARO_DEVTYPE_MS5607);
			break;
		}

		ret = OK;

		break;
	}

	if (ret == 0) {
		_initialized = true;
		start();
	}

	return ret;
}

void
MS5611::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_measure_phase = 0;

	/* schedule a cycle to start things */
	ScheduleDelayed(MS5611_CONVERSION_INTERVAL);
}

void
MS5611::RunImpl()
{
	int ret;
	unsigned dummy;

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		ret = collect();

		if (ret != OK) {
			if (ret == -6) {
				/*
				 * The ms5611 seems to regularly fail to respond to
				 * its address; this happens often enough that we'd rather not
				 * spam the console with a message for this.
				 */
			} else {
				//DEVICE_LOG("collection error %d", ret);
			}

			/* issue a reset command to the sensor */
			_interface->ioctl(IOCTL_RESET, dummy);
			/* reset the collection state machine and try again - we need
			 * to wait 2.8 ms after issuing the sensor reset command
			 * according to the MS5611 datasheet
			 */
			ScheduleDelayed(2800);
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;
	}

	/* measurement phase */
	ret = measure();

	if (ret != OK) {
		/* issue a reset command to the sensor */
		_interface->ioctl(IOCTL_RESET, dummy);
		/* reset the collection state machine and try again */
		start();
		return;
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(MS5611_CONVERSION_INTERVAL);
}

#ifndef REALDEV
int measureCnt = 0;
#endif // REALDEV

int
MS5611::measure()
{
	perf_begin(_measure_perf);

	/*
	 * In phase zero, request temperature; in other phases, request pressure.
	 */
#ifdef REALDEV
	unsigned addr = (_measure_phase == 0) ? ADDR_CMD_CONVERT_D2 : ADDR_CMD_CONVERT_D1;

	/*
	 * Send the command to begin measuring.
	 */
	int ret = _interface->ioctl(IOCTL_MEASURE, addr);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

#else //REALDEV
	int ret = OK;
#endif //REALDEV

	perf_end(_measure_perf);

	return ret;
}

int
MS5611::collect()
{
	uint32_t raw;

	perf_begin(_sample_perf);

	/* read the most recent measurement - read offset/size are hardcoded in the interface */
	const hrt_abstime timestamp_sample = hrt_absolute_time();
#ifdef REALDEV
	int ret = _interface->read(0, (void *)&raw, 0);

	if (ret < 0) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	// According to the sensor docs:
	// If the conversion is not executed before the ADC read command, or the
	// ADC read command is repeated, it will give 0 as the output result.
	//
	// We have seen 0 during the init phase on I2C, therefore, we add this
	// protection in.
	if (raw == 0) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

#else // REALDEV
	// we need to provide a little noise to the measurements, or it will be marked STALE:
	int i_myrand = timestamp_sample & 0xF;
	float myrand = ((float)i_myrand) / 1000.0f;   // about 5cm altitude noise amplitude

	// float the lawnmower a couple meters above ground about 20 seconds after start:
	if (measureCnt < 2000) {
		measureCnt++;

	} else {
		myrand += 0.3f; // 0.3f here raises drone altitude up 2.5 meters
	}

#endif // REALDEV

	/* handle a measurement */
	if (_measure_phase == 0) {

#ifdef REALDEV
		/* temperature offset (in ADC units) */
		int32_t dT = (int32_t)raw - ((int32_t)_prom.c5_reference_temp << 8);

		/* absolute temperature in centidegrees - note intermediate value is outside 32-bit range */
		int32_t TEMP = 2000 + (int32_t)(((int64_t)dT * _prom.c6_temp_coeff_temp) >> 23);

		/* base sensor scale/offset values */
		if (_device_type == MS5611_DEVICE) {
			/* Perform MS5611 Caculation */
			_OFF  = ((int64_t)_prom.c2_pressure_offset << 16) + (((int64_t)_prom.c4_temp_coeff_pres_offset * dT) >> 7);
			_SENS = ((int64_t)_prom.c1_pressure_sens << 15) + (((int64_t)_prom.c3_temp_coeff_pres_sens * dT) >> 8);

			/* MS5611 temperature compensation */
			if (TEMP < 2000) {

				int32_t T2 = POW2(dT) >> 31;

				int64_t f = POW2((int64_t)TEMP - 2000);
				int64_t OFF2 = 5 * f >> 1;
				int64_t SENS2 = 5 * f >> 2;

				if (TEMP < -1500) {

					int64_t f2 = POW2(TEMP + 1500);
					OFF2 += 7 * f2;
					SENS2 += 11 * f2 >> 1;
				}

				TEMP -= T2;
				_OFF  -= OFF2;
				_SENS -= SENS2;
			}

		} else if (_device_type == MS5607_DEVICE) {
			/* Perform MS5607 Caculation */
			_OFF  = ((int64_t)_prom.c2_pressure_offset << 17) + (((int64_t)_prom.c4_temp_coeff_pres_offset * dT) >> 6);
			_SENS = ((int64_t)_prom.c1_pressure_sens << 16) + (((int64_t)_prom.c3_temp_coeff_pres_sens * dT) >> 7);

			/* MS5607 temperature compensation */
			if (TEMP < 2000) {

				int32_t T2 = POW2(dT) >> 31;

				int64_t f = POW2((int64_t)TEMP - 2000);
				int64_t OFF2 = 61 * f >> 4;
				int64_t SENS2 = 2 * f;

				if (TEMP < -1500) {
					int64_t f2 = POW2(TEMP + 1500);
					OFF2 += 15 * f2;
					SENS2 += 8 * f2;
				}

				TEMP -= T2;
				_OFF  -= OFF2;
				_SENS -= SENS2;
			}
		}

		_last_temperature = TEMP / 100.0f;
#else // REALDEV
		_last_temperature = 25.123f + myrand / 10.0f;
#endif // REALDEV

	} else {
		/* pressure calculation, result in Pa */
#ifdef REALDEV
		int32_t P = (((raw * _SENS) >> 21) - _OFF) >> 15;

		_last_pressure = P;
#else // REALDEV
		raw = 1013.25f;

		_last_pressure = raw - myrand;		/* in millibar */
#endif // REALDEV

		// publish
		if (_initialized && PX4_ISFINITE(_last_pressure) && PX4_ISFINITE(_last_temperature)) {
			sensor_baro_s sensor_baro{};
			sensor_baro.timestamp_sample = timestamp_sample;
			sensor_baro.device_id = _interface->get_device_id();
			sensor_baro.pressure = _last_pressure;
			sensor_baro.temperature = _last_temperature;
			sensor_baro.error_count = perf_event_count(_comms_errors);
			sensor_baro.timestamp = hrt_absolute_time();
			_sensor_baro_pub.publish(sensor_baro);
		}

	}

	/* update the measurement state machine */
	INCREMENT(_measure_phase, MS5611_MEASUREMENT_RATIO + 1);

	perf_end(_sample_perf);

	return OK;
}

void MS5611::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);

	printf("device:         %s\n", _device_type == MS5611_DEVICE ? "ms5611" : "ms5607");
}

namespace ms5611
{

/**
 * MS5611 crc4 cribbed from the datasheet
 */
bool
crc4(uint16_t *n_prom)
{
	int16_t cnt;
	uint16_t n_rem;
	uint16_t crc_read;
	uint8_t n_bit;

	n_rem = 0x00;

	/* save the read crc */
	crc_read = n_prom[7];

	/* remove CRC byte */
	n_prom[7] = (0xFF00 & (n_prom[7]));

	for (cnt = 0; cnt < 16; cnt++) {
		/* uneven bytes */
		if (cnt & 1) {
			n_rem ^= (uint8_t)((n_prom[cnt >> 1]) & 0x00FF);

		} else {
			n_rem ^= (uint8_t)(n_prom[cnt >> 1] >> 8);
		}

		for (n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & 0x8000) {
				n_rem = (n_rem << 1) ^ 0x3000;

			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	/* final 4 bit remainder is CRC value */
	n_rem = (0x000F & (n_rem >> 12));
	n_prom[7] = crc_read;

	/* return true if CRCs match */
	return (0x000F & crc_read) == (n_rem ^ 0x00);
}

} // namespace ms5611
