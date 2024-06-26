/****************************************************************************
 *
 * Copyright (C) 2019-2021 PX4 Development Team. All rights reserved.
 * Author: Igor Misic <igy1000mb@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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


#if (CONFIG_STM32_HAVE_IP_DMA_V1)
//Do nothing. IP DMA V1 MCUs are not supported.
#else

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/micro_hal.h>
#include <stm32_dma.h>
#include <stm32_tim.h>
#include <px4_arch/dshot.h>
#include <px4_arch/io_timer.h>
#include <drivers/drv_dshot.h>


#define MOTOR_PWM_BIT_1				14u
#define MOTOR_PWM_BIT_0				7u
#define DSHOT_TIMERS				MAX_IO_TIMERS
#define MOTORS_NUMBER				DIRECT_PWM_OUTPUT_CHANNELS
#define ONE_MOTOR_DATA_SIZE			16u
#define ONE_MOTOR_BUFF_SIZE			17u
#define ALL_MOTORS_BUF_SIZE			(MOTORS_NUMBER * ONE_MOTOR_BUFF_SIZE)
#define DSHOT_THROTTLE_POSITION		5u
#define DSHOT_TELEMETRY_POSITION	4u
#define NIBBLES_SIZE 				4u
#define DSHOT_NUMBER_OF_NIBBLES		3u
#define DSHOT_END_OF_STREAM 		16u
#define MAX_NUM_CHANNELS_PER_TIMER	4u // CCR1-CCR4

#define DSHOT_DMA_SCR (DMA_SCR_PRIHI | DMA_SCR_MSIZE_32BITS | DMA_SCR_PSIZE_32BITS | DMA_SCR_MINC | \
		       DMA_SCR_DIR_M2P | DMA_SCR_TCIE | DMA_SCR_HTIE | DMA_SCR_TEIE | DMA_SCR_DMEIE)

typedef struct dshot_handler_t {
	bool			init;
	DMA_HANDLE		dma_handle;
	uint32_t		dma_size;
} dshot_handler_t;

#if defined(CONFIG_ARMV7M_DCACHE)
#  define DMA_BUFFER_MASK    (ARMV7M_DCACHE_LINESIZE - 1)
#  define DMA_ALIGN_UP(n)    (((n) + DMA_BUFFER_MASK) & ~DMA_BUFFER_MASK)
#else
#define DMA_ALIGN_UP(n) (n)
#endif
#define DSHOT_BURST_BUFFER_SIZE(motors_number) (DMA_ALIGN_UP(sizeof(uint32_t)*ONE_MOTOR_BUFF_SIZE*motors_number))

static dshot_handler_t dshot_handler[DSHOT_TIMERS] = {};
static uint8_t dshot_burst_buffer_array[DSHOT_TIMERS * DSHOT_BURST_BUFFER_SIZE(MAX_NUM_CHANNELS_PER_TIMER)]
px4_cache_aligned_data() = {};
static uint32_t *dshot_burst_buffer[DSHOT_TIMERS] = {};

int up_dshot_init(uint32_t channel_mask, unsigned dshot_pwm_freq, bool enable_bidirectional_dshot)
{
	unsigned buffer_offset = 0;

	for (int timer_index = 0; timer_index < DSHOT_TIMERS; timer_index++) {
		dshot_handler[timer_index].init = false;
	}

	for (unsigned timer = 0; timer < DSHOT_TIMERS; ++timer) {
		if (io_timers[timer].base == 0) { // no more timers configured
			break;
		}

		// we know the uint8_t* cast to uint32_t* is fine, since we're aligned to cache line size
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
		dshot_burst_buffer[timer] = (uint32_t *)&dshot_burst_buffer_array[buffer_offset];
#pragma GCC diagnostic pop
		buffer_offset += DSHOT_BURST_BUFFER_SIZE(io_timers_channel_mapping.element[timer].channel_count_including_gaps);

		if (buffer_offset > sizeof(dshot_burst_buffer_array)) {
			return -EINVAL; // something is wrong with the board configuration or some other logic
		}
	}

	/* Init channels */
	int ret_val = OK;
	int channels_init_mask = 0;

	for (unsigned channel = 0; (channel_mask != 0) && (channel < MAX_TIMER_IO_CHANNELS) && (OK == ret_val); channel++) {
		if (channel_mask & (1 << channel)) {
			uint8_t timer = timer_io_channels[channel].timer_index;

			if (io_timers[timer].dshot.dma_base == 0) { // board does not configure dshot on this timer
				continue;
			}

			ret_val = io_timer_channel_init(channel, IOTimerChanMode_Dshot, NULL, NULL);
			channel_mask &= ~(1 << channel);

			if (OK == ret_val) {
				dshot_handler[timer].init = true;
				channels_init_mask |= 1 << channel;

			} else if (ret_val == -EBUSY) {
				/* either timer or channel already used - this is not fatal */
				ret_val = 0;
			}
		}
	}

	for (uint8_t timer_index = 0; (timer_index < DSHOT_TIMERS) && (OK == ret_val); timer_index++) {

		if (true == dshot_handler[timer_index].init) {
			dshot_handler[timer_index].dma_size = io_timers_channel_mapping.element[timer_index].channel_count_including_gaps *
							      ONE_MOTOR_BUFF_SIZE;
			io_timer_set_dshot_mode(timer_index, dshot_pwm_freq,
						io_timers_channel_mapping.element[timer_index].channel_count_including_gaps);

			dshot_handler[timer_index].dma_handle = stm32_dmachannel(io_timers[timer_index].dshot.dmamap);

			if (NULL == dshot_handler[timer_index].dma_handle) {
				ret_val = ERROR;
			}
		}
	}

	return ret_val == OK ? channels_init_mask : ret_val;
}

int up_bdshot_get_erpm(uint8_t channel, int *erpm)
{
	// Not implemented
	return -1;
}

int up_bdshot_channel_status(uint8_t channel)
{
	// Not implemented
	return -1;
}

void up_bdshot_status(void)
{
}

void up_dshot_trigger(void)
{
	for (uint8_t timer = 0; (timer < DSHOT_TIMERS); timer++) {

		if (true == dshot_handler[timer].init) {

			// Flush cache so DMA sees the data
			up_clean_dcache((uintptr_t)dshot_burst_buffer[timer],
					(uintptr_t)dshot_burst_buffer[timer] +
					DSHOT_BURST_BUFFER_SIZE(io_timers_channel_mapping.element[timer].channel_count_including_gaps));

			px4_stm32_dmasetup(dshot_handler[timer].dma_handle,
					   io_timers[timer].base + STM32_GTIM_DMAR_OFFSET,
					   (uint32_t)(dshot_burst_buffer[timer]),
					   dshot_handler[timer].dma_size,
					   DSHOT_DMA_SCR);

			// Clean UDE flag before DMA is started
			io_timer_update_dma_req(timer, false);
			// Trigger DMA (DShot Outputs)
			stm32_dmastart(dshot_handler[timer].dma_handle, NULL, NULL, false);
			io_timer_update_dma_req(timer, true);

		}
	}
}

/**
* bits 	1-11	- throttle value (0-47 are reserved, 48-2047 give 2000 steps of throttle resolution)
* bit 	12		- dshot telemetry enable/disable
* bits 	13-16	- XOR checksum
**/
void dshot_motor_data_set(unsigned motor_number, uint16_t throttle, bool telemetry)
{
	uint16_t packet = 0;
	uint16_t checksum = 0;

	packet |= throttle << DSHOT_THROTTLE_POSITION;
	packet |= ((uint16_t)telemetry & 0x01) << DSHOT_TELEMETRY_POSITION;

	uint16_t csum_data = packet;

	/* XOR checksum calculation */
	csum_data >>= NIBBLES_SIZE;

	for (unsigned i = 0; i < DSHOT_NUMBER_OF_NIBBLES; i++) {
		checksum ^= (csum_data & 0x0F); // XOR data by nibbles
		csum_data >>= NIBBLES_SIZE;
	}

	packet |= (checksum & 0x0F);

	unsigned timer = timer_io_channels[motor_number].timer_index;
	uint32_t *buffer = dshot_burst_buffer[timer];
	const io_timers_channel_mapping_element_t *mapping = &io_timers_channel_mapping.element[timer];
	unsigned num_motors = mapping->channel_count_including_gaps;
	unsigned timer_channel_index = timer_io_channels[motor_number].timer_channel - mapping->lowest_timer_channel;

	for (unsigned motor_data_index = 0; motor_data_index < ONE_MOTOR_DATA_SIZE; motor_data_index++) {
		buffer[motor_data_index * num_motors + timer_channel_index] =
			(packet & 0x8000) ? MOTOR_PWM_BIT_1 : MOTOR_PWM_BIT_0;  // MSB first
		packet <<= 1;
	}
}

int up_dshot_arm(bool armed)
{
	return io_timer_set_enable(armed, IOTimerChanMode_Dshot, IO_TIMER_ALL_MODES_CHANNELS);
}

#endif
