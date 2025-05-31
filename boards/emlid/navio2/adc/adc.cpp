/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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

#include <board_config.h>

#include <drivers/drv_adc.h>

#include <px4_platform_common/log.h>

#include <fcntl.h>
#include <stdint.h>
#include <unistd.h>

#include "mcp3008Spi.hpp"

#define ADC_SPIDEV_BUS 0
#define ADC_SPIDEV_DEV 0

#define ADC_LOW_SPI_BUS_SPEED	1000*1000
#define ADC_HIGH_SPI_BUS_SPEED	20*1000*1000

#define ADC_SPI_BITS_PER_WORD   8

#define ADC_MAX_ADC_CHAN 8
#define ADC_MAX_CHAN 6

int _channels_fd[ADC_MAX_CHAN];

// mcp3008Spi(int bus_number, int device_id, spi_mode_e spi_mode, int bus_frequency)
// SPIDEV_MODE0 = SPI_MODE_0 (defined in SPI.hpp, linux/spi/spidev.h), speed = 1MHz

mcp3008Spi *a2d = nullptr;

int px4_arch_adc_init(uint32_t base_address)
{
	for (int i = 0; i < ADC_MAX_CHAN; i++) {
		_channels_fd[i] = -1;
	}

	a2d = new mcp3008Spi(ADC_SPIDEV_BUS, ADC_SPIDEV_DEV, SPIDEV_MODE0, ADC_LOW_SPI_BUS_SPEED);

	int ret = a2d->init();

	if (ret != OK) {
		PX4_ERR("SPI init failed");
		return ret;
	}

	ret = a2d->probe();

	if (ret != OK) {
		PX4_ERR("SPI probe failed");
		return ret;
	}

	return ret;
}

void px4_arch_adc_uninit(uint32_t base_address)
{
	a2d->uninit();

	delete a2d;
	a2d = nullptr;

	for (int i = 0; i < ADC_MAX_CHAN; i++) {
		//::close(_channels_fd[i]);
		_channels_fd[i] = -1;
	}
}

uint32_t px4_arch_adc_sample(uint32_t base_address, unsigned channel)
{
	if (channel > ADC_MAX_CHAN) {
		PX4_ERR("px4_arch_adc_sample(): channel %d out of range: %d", channel, ADC_MAX_CHAN);
		return UINT32_MAX; // error
	}

	// see https://github.com/halherta/RaspberryPi-mcp3008Spi

	int a2dVal = 0;
	int a2dChannel = channel;

	unsigned char data[3];
	data[0] = 1;  //  first byte transmitted -> start bit
	data[1] = 0b10000000 | (((a2dChannel & 7) << 4)); // second byte transmitted -> (SGL/DIF = 1, D2=D1=D0=0)
	data[2] = 0; // third byte transmitted....don't care

	if (a2d->spiWriteRead(data, sizeof(data)) != PX4_OK) {
		PX4_ERR("channel %d failed SPI transfer", channel);
		return UINT32_MAX; // error
	}

	a2dVal = 0;
	a2dVal = (data[1] << 8) & 0b1100000000; //merge data[1] & data[2] to get result
	a2dVal |= (data[2] & 0xff);


	return a2dVal;
}

float px4_arch_adc_reference_v()
{
	return BOARD_ADC_POS_REF_V;     // 3.3 for MCP3008 on the custom RPi Hat
}

uint32_t px4_arch_adc_dn_fullcount()
{
	return 1 << 10; // 10 bit ADC for MCP3008
}
