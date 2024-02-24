
#include "mcp3008Spi.hpp"

mcp3008Spi::mcp3008Spi(int bus_number, int device_id, spi_mode_e spi_mode, int bus_frequency) :
	SPI(DRV_ADC_DEVTYPE_ADS1115, MODULE_NAME, bus_number, device_id, spi_mode, bus_frequency)
{
	PX4_INFO("mcp3008Spi: bus_number=%d, device_id=%d, spi_mode=%d, bus_frequency=%d", bus_number, device_id, spi_mode,
		 bus_frequency);
}

mcp3008Spi::~mcp3008Spi()
{
}

int mcp3008Spi::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		return ret;
	}

	return PX4_OK;
}

int mcp3008Spi::uninit()
{
	//int ret = SPI::uninit();

	//if (ret != PX4_OK) {
	//        return ret;
	//}

	return PX4_OK;
}

/**
 * Check for the presence of the device on the bus.
 */
int mcp3008Spi::probe() { return SPI::probe(); }


/****************************************************************************
 * This function writes byte data "data" of length "len" to the spidev device.
 * Data received from the spidev device is saved back into * "data".
 * **************************************************************************/
int mcp3008Spi::spiWriteRead(uint8_t *data, int len)
{
	return SPI::transfer(data, data, len);
}
