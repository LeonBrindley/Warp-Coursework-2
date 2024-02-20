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
	// Current and power calibration are set by bits D15 to D1 of the Calibration Register. D0 is a void bit and will always be '0'.
	// Adjust the Calibration Register after taking your initial readings to achieve higher precision.
	// The Calibration Register (05h) is set in order to provide the device information about the current shunt resistor that was
	// used to create the measured shunt voltage. By knowing the value of the shunt resistor, the device can then calculate the amount of
	// current that created the measured shunt voltage drop.
	writeSensorRegisterINA219(kINA219RegCalibration, 0x1000);

	OSA_TimeDelay(20);
	
	return;
}

WarpStatus readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes){
	// See MMA8451Q.c for example: https://github.com/LeonBrindley/Warp-Coursework-2/blob/master/src/boot/ksdk1.1.0/devMMA8451Q.c
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t status;
	
	USED(numberOfBytes);
	switch (deviceRegister)
	{
		case 0x00: case 0x01: case 0x02: case 0x03:
		case 0x04: case 0x05:
		{
			/* OK */
			break;
		}

		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}
	
	i2c_device_t slave =
		{
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	cmdBuf[0] = deviceRegister;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterReceiveDataBlocking(
		0 /* I2C peripheral instance */,
		&slave,
		cmdBuf,
		1,
		(uint8_t *)deviceINA219State.i2cBuffer,
		numberOfBytes,
		gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus writeSensorRegisterINA219(uint8_t deviceRegister, uint8_t payload){
	uint8_t		payloadBytes[2], commandByte[1];
	i2c_status_t	status;

	switch (deviceRegister)
	{
		case 0x00: case 0x01: case 0x02: case 0x03:
		case 0x04: case 0x05:
		{
			/* OK */
			break;
		}

		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
		{
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	commandByte[0] = deviceRegister;
	payloadBytes[0] = payload >> 8; // MSB
	payloadBytes[1] = payload & 0xFF; // LSB
	warpEnableI2Cpins();

	status = I2C_DRV_MasterSendDataBlocking(
		0 /* I2C instance */,
		&slave,
		commandByte,
		1,
		payloadByte,
		2,
		gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

uint16_t returnShunt(void){
	uint16_t Shunt;
	WarpStatus i2cReadStatus;

	i2cReadStatus = readSensorRegisterINA219(kINA219RegShunt, 2);
	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
		return 0;
	}

	Shunt = (uint16_t *)deviceINA219State.i2cBuffer | (uint16_t *)deviceINA219State.i2cBuffer[0] << 8;
	
	return Shunt;
}

uint16_t returnBus(void){
	uint16_t Bus;
	WarpStatus i2cReadStatus;
	
	i2cReadStatus = readSensorRegisterINA219(kINA219RegBus, 2);
	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
		return 0;
	}

	Bus = (uint16_t *)deviceINA219State.i2cBuffer | (uint16_t *)deviceINA219State.i2cBuffer[0] << 8;
	
	return Bus;
}

// The device can measure bidirectional current; thus, the MSB of the Current Register is a sign bit that allows for the rest of the 15
// bits to be used for the Current Register value.
uint16_t returnCurrent(void){
	int16_t Current;
	WarpStatus i2cReadStatus;

	i2cReadStatus = readSensorRegisterINA219(kINA219RegCurrent, 2);
	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
		return 0;
	}

	Current = (int16_t *) (deviceINA219State.i2cBuffer | deviceINA219State.i2cBuffer[0] << 8);
	
	return Current;
}

uint16_t returnPower(void){
	uint16_t Power;
	WarpStatus i2cReadStatus;

	i2cReadStatus = readSensorRegisterINA219(kINA219RegPower, 2);
	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
		return 0;
	}

	Power = (uint16_t *)deviceINA219State.i2cBuffer | (uint16_t *)deviceINA219State.i2cBuffer[0] << 8;
	
	return Power;
}
