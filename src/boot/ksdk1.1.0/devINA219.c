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
	// See MMA8451Q.c for example: https://github.com/LeonBrindley/Warp-Coursework-2/blob/master/src/boot/ksdk1.1.0/devMMA8451Q.c
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t status;
	
	USED(numberOfBytes);
	switch (deviceRegister)
	{
		case 0x00: case 0x01: case 0x02: case 0x03:
		case 0x04: case 0x05: case 0x06: case 0x09:
		case 0x0a: case 0x0b: case 0x0c: case 0x0d:
		case 0x0e: case 0x0f: case 0x10: case 0x11:
		case 0x12: case 0x13: case 0x14: case 0x15:
		case 0x16: case 0x17: case 0x18: case 0x1d:
		case 0x1e: case 0x1f: case 0x20: case 0x21:
		case 0x22: case 0x23: case 0x24: case 0x25:
		case 0x26: case 0x27: case 0x28: case 0x29:
		case 0x2a: case 0x2b: case 0x2c: case 0x2d:
		case 0x2e: case 0x2f: case 0x30: case 0x31:
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
	uint8_t		payloadByte[1], commandByte[1];
	i2c_status_t	status;

	switch (deviceRegister)
	{
		case 0x09: case 0x0a: case 0x0e: case 0x0f:
		case 0x11: case 0x12: case 0x13: case 0x14:
		case 0x15: case 0x17: case 0x18: case 0x1d:
		case 0x1f: case 0x20: case 0x21: case 0x23:
		case 0x24: case 0x25: case 0x26: case 0x27:
		case 0x28: case 0x29: case 0x2a: case 0x2b:
		case 0x2c: case 0x2d: case 0x2e: case 0x2f:
		case 0x30: case 0x31:
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
	payloadByte[0] = payload;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterSendDataBlocking(
		0 /* I2C instance */,
		&slave,
		commandByte,
		1,
		payloadByte,
		1,
		gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
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
