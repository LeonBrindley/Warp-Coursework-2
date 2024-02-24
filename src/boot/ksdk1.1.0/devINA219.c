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
	
	// Set the configuration register to 415. The POR value equals 14751: https://www.vle.cam.ac.uk/pluginfile.php/13708422/mod_resource/content/1/ina219.pdf
	writeSensorRegisterINA219(kINA219RegConfiguration, (uint16_t) 0b0000000110011111);
	
	// Set the calibration register to 4096 by default: https://www.vle.cam.ac.uk/pluginfile.php/13708422/mod_resource/content/1/ina219.pdf
	// Current and power calibration are set by bits D15 to D1 of the Calibration Register. D0 is a void bit and will always be '0'.
	// Adjust the Calibration Register after taking your initial readings to achieve higher precision.
	// The Calibration Register (05h) is set in order to provide the device information about the current shunt resistor that was
	// used to create the measured shunt voltage. By knowing the value of the shunt resistor, the device can then calculate the amount of
	// current that created the measured shunt voltage drop.
	// writeSensorRegisterINA219(kINA219RegCalibration, (uint16_t) 0xA000); // 40,960 = 10uA LSB with 50mA maximum current and 0.1 Ohm resistor.
	writeSensorRegisterINA219(kINA219RegCalibration, (uint16_t) 0x5000); // 20,480 = 20uA LSB with 100mA maximum current and 0.1 Ohm resistor.

	OSA_TimeDelay(50);
	
	return;
}

WarpStatus readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes){
	// See MMA8451Q.c for example: https://github.com/LeonBrindley/Warp-Coursework-2/blob/master/src/boot/ksdk1.1.0/devMMA8451Q.c
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t status;

	warpPrint("Reading %d bytes from INA219 register %d.\n", numberOfBytes, deviceRegister);
	
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
			warpPrint("kWarpStatusBadDeviceCommand\n");
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
		1, // Send one command byte.
		(uint8_t *)deviceINA219State.i2cBuffer, // * refers to an array of pointers.
		numberOfBytes, // The number of payload bytes is parameterised.
		gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		warpPrint("kWarpStatusDeviceCommunicationFailed\n");
		return kWarpStatusDeviceCommunicationFailed;
	}

	warpPrint("Finished reading %d from INA219 register %d.\n", (deviceINA219State.i2cBuffer[0]  * 256) + deviceINA219State.i2cBuffer[1], deviceRegister);

	return kWarpStatusOK;
}

WarpStatus writeSensorRegisterINA219(uint8_t deviceRegister, uint16_t payload){
	uint8_t		payloadBytes[2], commandByte[1];
	i2c_status_t	status;

	switch (deviceRegister)
	{
		case 0x00: case 0x05:
		{
			/* OK */
			break;
		}

		default:
		{
			warpPrint("kWarpStatusBadDeviceCommand\n");
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

	warpPrint("Writing %d to INA219 register %d.\n", (payloadBytes[0] * 256) + payloadBytes[1], commandByte[0]);
	
	status = I2C_DRV_MasterSendDataBlocking(
		0 /* I2C instance */,
		&slave,
		commandByte,
		1, // Send one command byte.
		payloadBytes,
		2, // Send two payload bytes.
		gWarpI2cTimeoutMilliseconds);
	
	if (status != kStatus_I2C_Success)
	{
		warpPrint("kWarpStatusDeviceCommunicationFailed\n");
		return kWarpStatusDeviceCommunicationFailed;
	}

	warpPrint("Finished writing to INA219 register %d.\n", deviceRegister);

	return kWarpStatusOK;
}

// Style return functions after printSensorDataMMA8451Q() in devMMA8451Q.c.

int32_t returnShunt(void){
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	int16_t 	Shunt;
	WarpStatus 	i2cReadStatus;

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	
	i2cReadStatus = readSensorRegisterINA219(kINA219RegShunt, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = (readSensorRegisterValueLSB + (readSensorRegisterValueMSB << 8));

	warpPrint("readSensorRegisterValueMSB: %d\n", readSensorRegisterValueMSB);
	warpPrint("readSensorRegisterValueLSB: %d\n", readSensorRegisterValueLSB);
	warpPrint("readSensorRegisterValueCombined: %d\n", readSensorRegisterValueCombined);
	
	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
		return 0;
	}

	// Convert this Shunt variable to real units by multiplying by the LSB (10 microvolts).
	Shunt = readSensorRegisterValueCombined * kINA219ShuntLSB;
	return Shunt;
}

int32_t returnBus(void){
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	int16_t 	Bus;
	WarpStatus 	i2cReadStatus;

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	
	i2cReadStatus = readSensorRegisterINA219(kINA219RegBus, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = (readSensorRegisterValueLSB + (readSensorRegisterValueMSB << 8));

	warpPrint("readSensorRegisterValueMSB: %d\n", readSensorRegisterValueMSB);
	warpPrint("readSensorRegisterValueLSB: %d\n", readSensorRegisterValueLSB);
	warpPrint("readSensorRegisterValueCombined: %d\n", readSensorRegisterValueCombined);
	
	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
		return 0;
	}

	// Convert this Bus variable to real units by multiplying by the LSB (10 microvolts).
	Bus = readSensorRegisterValueCombined * kINA219BusLSB;
	return Bus;
}

// The device can measure bidirectional current; thus, the MSB of the Current Register is a sign bit that allows for the rest of the 15
// bits to be used for the Current Register value.
int32_t returnCurrent(void){
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	uint16_t	readSensorRegisterValueCombined;
	int32_t		Current;
	WarpStatus 	i2cReadStatus;

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

	i2cReadStatus = readSensorRegisterINA219(kINA219RegCurrent, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((uint16_t)(readSensorRegisterValueLSB) | (uint16_t)(readSensorRegisterValueMSB << 8));

	// warpPrint("readSensorRegisterValueMSB: %d\n", readSensorRegisterValueMSB);
	// warpPrint("readSensorRegisterValueLSB: %d\n", readSensorRegisterValueLSB);
	// warpPrint("readSensorRegisterValueCombined: %d\n", readSensorRegisterValueCombined);
	
	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
		return 0;
	}

	// Convert this Current variable to real units by multiplying by the LSB (10 microamps).
	Current = (int32_t)readSensorRegisterValueCombined * (int32_t)kINA219CurrentLSB;
	// warpPrint("Current: %d\n", Current);
	return Current;
}

uint32_t returnPower(void){
	// Note that power is unsigned (in contrast to the voltages and currents above).
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	uint16_t	readSensorRegisterValueCombined;
	uint16_t 	Power;
	WarpStatus 	i2cReadStatus;

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	
	i2cReadStatus = readSensorRegisterINA219(kINA219RegPower, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = (readSensorRegisterValueLSB + (readSensorRegisterValueMSB << 8));

	warpPrint("readSensorRegisterValueMSB: %d\n", readSensorRegisterValueMSB);
	warpPrint("readSensorRegisterValueLSB: %d\n", readSensorRegisterValueLSB);
	warpPrint("readSensorRegisterValueCombined: %d\n", readSensorRegisterValueCombined);
	
	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
		return 0;
	}

	// Convert this Power variable to real units by multiplying by the LSB (100 microwatts).
	Power = readSensorRegisterValueCombined * kINA219PowerLSB;
	return Power;
}

uint8_t appendSensorDataINA219(uint8_t* buf){
	uint8_t index = 0;
	uint16_t readSensorRegisterValueLSB;
	uint16_t readSensorRegisterValueMSB;
	int16_t readSensorRegisterValueCombined;
	WarpStatus i2cReadStatus;

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

	// Chosen to append the current measurements.
	i2cReadStatus                   = readSensorRegisterINA219(kINA219RegCurrent, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB      = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB      = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = (readSensorRegisterValueLSB | readSensorRegisterValueMSB << 8);

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}
	return index;
}

void printSensorDataINA219(bool hexModeFlag){
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;


	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

	i2cReadStatus = readSensorRegisterINA219(kINA219RegCurrent, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = (readSensorRegisterValueLSB | readSensorRegisterValueMSB << 8);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}
}
