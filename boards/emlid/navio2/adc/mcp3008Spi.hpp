/***********************************************************************
 * Author: Hussam Al-Hertani
 *
 * see https://github.com/halherta/RaspberryPi-mcp3008Spi
 *
 * This header file contains the mcp3008Spi class definition.
 * Its main purpose is to communicate with the MCP3008 chip using
 * the userspace spidev facility.
 * The class contains four variables:
 * mode        -> defines the SPI mode used. In our case it is SPI_MODE_0.
 * bitsPerWord -> defines the bit width of the data transmitted.
 *        This is normally 8. Experimentation with other values
 *        didn't work for me
 * speed       -> Bus speed or SPI clock frequency. According to
 *                https://projects.drogon.net/understanding-spi-on-the-raspberry-pi/
 *            It can be only 0.5, 1, 2, 4, 8, 16, 32 MHz.
 *                Will use 1MHz for now and test it further.
 * spifd       -> file descriptor for the SPI device
 *
 * The class contains two constructors that initialize the above
 * variables and then open the appropriate spidev device using spiOpen().
 * The class contains one destructor that automatically closes the spidev
 * device when object is destroyed by calling spiClose().
 * The spiWriteRead() function sends the data "data" of length "length"
 * to the spidevice and at the same time receives data of the same length.
 * Resulting data is stored in the "data" variable after the function call.
 * ****************************************************************************/

#pragma once

#include <drivers/drv_adc.h>
#include <drivers/drv_hrt.h>
#include <lib/cdev/CDev.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_adc.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/adc_report.h>

class mcp3008Spi : public device::SPI
{

public:
	mcp3008Spi(int bus_number, int device_id, spi_mode_e spi_mode, int bus_frequency);
	~mcp3008Spi() override;

	int init() override;
	int uninit();

	/**
	 * Check for the presence of the device on the bus.
	 */
	int	probe() override;

	int spiWriteRead(unsigned char *data, int length);

private:

};

