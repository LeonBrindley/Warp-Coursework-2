#include <stdlib.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "errstrs.h"
#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"
#include "devINA219.h"

extern volatile WarpI2CDeviceState    deviceINA219State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;

void initINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts){
	deviceINA219State.i2cAddress			= i2cAddress;
	deviceINA219State.operatingVoltageMillivolts	= operatingVoltageMillivolts;

	// Set the configuration register to the POR value of 14751: https://www.vle.cam.ac.uk/pluginfile.php/13708422/mod_resource/content/1/ina219.pdf
	writeSensorRegisterINA219(kINA219RegConfiguration, 0b0011100110011111);
	// Set the calibration register to 4096 by default: https://www.vle.cam.ac.uk/pluginfile.php/13708422/mod_resource/content/1/ina219.pdf
	writeSensorRegisterINA219(kINA219RegCalibration, 0x1000);

	OSA_TimeDelay(20);
	
	return;
}

WarpStatus readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes){
	i2c_status_t status;
	
}

WarpStatus writeSensorRegisterINA219(uint8_t deviceRegister, uint8_t payload){
	i2c_status_t status;
}

uint16_t returnShunt(void){
	uint16_t Shunt;
	WarpStatus status;

	Shunt = readSensorRegisterINA219(kINA219RegShunt, 2);
	return Shunt;
}

uint16_t returnBus(void){
	uint16_t Bus;
	WarpStatus status;
	
	Bus = readSensorRegisterINA219(kINA219RegBus, 2);
	return Bus;
}

uint16_t returnCurrent(void){
	uint16_t Current;
	WarpStatus status;

	Current = readSensorRegisterINA219(kINA219RegCurrent, 2);
	return Current;
}

uint16_t returnPower(void){
	uint16_t Power;
	WarpStatus status;

	Power = readSensorRegisterINA219(kINA219RegPower, 2);
	return Power;
}
