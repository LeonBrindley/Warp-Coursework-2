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
	writeSensorRegisterINA219(kINA219RegConfiguration, (uint16_t) 0b0011100110011111);
	
	// Set the calibration register to 4096 by default: https://www.vle.cam.ac.uk/pluginfile.php/13708422/mod_resource/content/1/ina219.pdf
	// Current and power calibration are set by bits D15 to D1 of the Calibration Register. D0 is a void bit and will always be '0'.
	// Adjust the Calibration Register after taking your initial readings to achieve higher precision.
	// The Calibration Register (05h) is set in order to provide the device information about the current shunt resistor that was
	// used to create the measured shunt voltage. By knowing the value of the shunt resistor, the device can then calculate the amount of
	// current that created the measured shunt voltage drop.
	writeSensorRegisterINA219(kINA219RegCalibration, (uint16_t) 0x1000);

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
		(uint8_t *)deviceINA219State.i2cBuffer, // * refers to an array of pointers.
		numberOfBytes,
		gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus writeSensorRegisterINA219(uint8_t deviceRegister, uint16_t payload){
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
	payloadBytes[0] = (uint8_t)(payload >> 8); // MSB
	payloadBytes[1] = (uint8_t)(payload & 0xFF); // LSB
	warpEnableI2Cpins();

	status = I2C_DRV_MasterSendDataBlocking(
		0 /* I2C instance */,
		&slave,
		commandByte,
		1,
		(uint8_t *)payloadBytes, // * refers to an array of pointers.
		2,
		gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

// Style return functions after printSensorDataMMA8451Q() in devMMA8451Q.c.

int16_t returnShunt(void){
	int16_t Shunt;
	WarpStatus i2cReadStatus;

	i2cReadStatus = readSensorRegisterINA219(kINA219RegShunt, 2);
	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
		return 0;
	}

	// Combined value should be cast to a signed integer as in printSensorDataMMA8451Q().
	Shunt = (int16_t) (deviceINA219State.i2cBuffer[1] | deviceINA219State.i2cBuffer[0] << 8);

	// Convert this Shunt variable to real units by multiplying by the LSB (10 microvolts).
	return (Shunt * 10);
}

int16_t returnBus(void){
	int16_t Bus;
	WarpStatus i2cReadStatus;
	
	i2cReadStatus = readSensorRegisterINA219(kINA219RegBus, 2);
	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
		return 0;
	}

	// Combined value should be cast to a signed integer as in printSensorDataMMA8451Q().
	Bus = (int16_t) (deviceINA219State.i2cBuffer[1] | deviceINA219State.i2cBuffer[0] << 8);

	// Convert this Bus variable to real units by multiplying by the LSB (10 microvolts).
	return (Bus * 10);
}

// The device can measure bidirectional current; thus, the MSB of the Current Register is a sign bit that allows for the rest of the 15
// bits to be used for the Current Register value.
int16_t returnCurrent(void){
	int16_t Current;
	WarpStatus i2cReadStatus;

	i2cReadStatus = readSensorRegisterINA219(kINA219RegCurrent, 2);
	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
		return 0;
	}

	// Combined value should be cast to a signed integer as in printSensorDataMMA8451Q().
	Current = (int16_t) (deviceINA219State.i2cBuffer[1] | deviceINA219State.i2cBuffer[0] << 8);

	// Convert this Current variable to real units by multiplying by the LSB (10 microamps).
	return (Current * 10);
}

uint16_t returnPower(void){
	// Note that power is unsigned (in contrast to the voltages and currents above).
	uint16_t Power;
	WarpStatus i2cReadStatus;

	i2cReadStatus = readSensorRegisterINA219(kINA219RegPower, 2);
	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
		return 0;
	}

	// Combined value should be cast to an unsigned integer.
	Power = (uint16_t) (deviceINA219State.i2cBuffer[1] | deviceINA219State.i2cBuffer[0] << 8);

	// Convert this Power variable to real units by multiplying by the LSB (20 * Current LSB = 200 microwatts).
	return (Power * 200);
}
