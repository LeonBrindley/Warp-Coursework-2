#include <stdlib.h>
#include <math.h> // Required for sqrt() function.

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

#include "devMMA8451Q.h"
#include "devSSD1331.h"
#include "activityClassifier.h"

// See example implementation at https://www.vle.cam.ac.uk/pluginfile.php/27161189/mod_resource/content/1/chapter-02-measurements-and-uncertainty-and-cover.pdf.
// As I want to calculate the actual speed, the code below calculates the magnitude of the acceleration instead of simply detecting the maximal axis.
// Try a sampling rate of 50Hz, which matches the algorithm at https://www.ti.com/lit/pdf/slaa599. Decrease this if there is insufficient memory.
// The algorithm referenced above requires 1.2 kilobytes of code memory and 640 bytes of data memory. 

// Step 1: Combine all three axes using 3D Pythagoras' theorem.

void classifierAlgorithm(){
  // Set default maximumValue and minimumValue to guarantee that they are updated in the for loop below.
  maximumValue = 0;
  minimumValue = 65535;
	
  // Store the magnitude of the acceleration in AccelerationBuffer. 
  AccelerationBuffer[BUFFER_SIZE - 1] =  sqrt((XAcceleration*XAcceleration)
    + (YAcceleration*YAcceleration) + (ZAcceleration*ZAcceleration));
  warpPrint("1. Latest Magnitude: %d m/s.\n", AccelerationBuffer[BUFFER_SIZE - 1]);
  for (int i = 0; i < BUFFER_SIZE - 1; i++){
    LPFBuffer[i] = AccelerationBuffer[i] * LPFWeights[i];
    warpPrint("2. LPFBuffer[%d]: %d.\n", i, LPFBuffer[i]);
    if(LPFBuffer[i] > maximumValue){
      maximumValue = LPFBuffer[i];
    }
    if(LPFBuffer[i] < minimumValue){
      minimumValue = LPFBuffer[i];
    } 
  }
  LPFBufferMidpoint = (maximumValue - minimumValue) / 2;
  warpPrint("3. Maximum: %d, Minimum: %d, Midpoint: %d.\n", maximumValue, minimumValue, LPFBufferMidpoint);

  // See https://www.vle.cam.ac.uk/pluginfile.php/27161189/mod_resource/content/1/chapter-02-measurements-and-uncertainty-and-cover.pdf.
  numberOfCrossings = 0;
  for(int i = 1; i < BUFFER_SIZE - 1; i++){
    if(LPFBuffer[i - 1] > LPFBufferMidpoint){
      if(LPFBuffer[i] < LPFBufferMidpoint){
        numberOfCrossings++;
      }
    }
    else if(LPFBuffer[i - 1] < LPFBufferMidpoint){
      if(LPFBuffer[i] > LPFBufferMidpoint){
        numberOfCrossings++; 
      }
    }
    // Shift AccelerationBuffer and LPFBuffer left to free up space for new data.
    AccelerationBuffer[i - 1] = AccelerationBuffer[i];
    LPFBuffer[i - 1] = LPFBuffer[i];
  }
  // Average step length between men and women = 0.716m. https://marathonhandbook.com/average-stride-length
  numberOfSteps += (numberOfCrossings / 2); // Add (numberOfCrossings / 2) to the cumulative numbers of steps since booting the device.
  Speed = (360 / (1000 * 2))*(numberOfCrossings * 0.716); // 360 10-second periods in an hour. Divide by (1000*2) to convert to km/hr while accounting for both upward and downward crossings.
  warpPrint("4. Speed: %d.\n", Speed);
}

/*
void stepOneCombine(){
  // Shift AccelerationBuffer and LPFBuffer left to free up space for new data.
  for (int i = 1; i < BUFFER_SIZE - 1; i++){
    AccelerationBuffer[i - 1] = AccelerationBuffer[i];
    LPFBuffer[i - 1] = LPFBuffer[i];
  }
  // Store the magnitude of the acceleration in AccelerationBuffer. 
  AccelerationBuffer[BUFFER_SIZE - 1] =  sqrt((XAcceleration*XAcceleration)
    + (YAcceleration*YAcceleration) + (ZAcceleration*ZAcceleration));
  warpPrint("1. Latest Magnitude: %d m/s.\n", AccelerationBuffer[BUFFER_SIZE - 1]);
}

// Step 2: Low-pass filter the result to smooth out the signal.

void stepTwoFilter(){
  for (int i = 0; i < BUFFER_SIZE - 1; i++){
    LPFBuffer[i] = AccelerationBuffer[i] * LPFWeights[i];
    warpPrint("2. LPFBuffer[%d]: %d.\n", i, LPFBuffer[i]);
  }
}

// Step 3: Determine the midpoint of the results so the speed can be calculated in Step 4.

void stepThreeMidpoint(){
  // Set default maximumValue and minimumValue to guarantee that they are updated in the for loop below.
  maximumValue = 0;
  minimumValue = 65535;
  // Find maximum and minimum values in each time period.
  for(int i = 0; i < BUFFER_SIZE - 1; i++){
    if(LPFBuffer[i] > maximumValue){
      maximumValue = LPFBuffer[i];
    }
    if(LPFBuffer[i] < minimumValue){
      minimumValue = LPFBuffer[i];
    }    
  }
  LPFBufferMidpoint = (maximumValue - minimumValue) / 2;
  warpPrint("3. Maximum: %d, Minimum: %d, Midpoint: %d.\n", maximumValue, minimumValue, LPFBufferMidpoint);
}

// Step 4: Calculate the speed over the last 10 seconds.

void stepFourSpeed(){
  // Count the number of crossings of the midpoint of the maximum and minimum values.
  // See https://www.vle.cam.ac.uk/pluginfile.php/27161189/mod_resource/content/1/chapter-02-measurements-and-uncertainty-and-cover.pdf.
  numberOfCrossings = 0;
  for(int i = 0; i < BUFFER_SIZE - 2; i++){
    if(LPFBuffer[i] > LPFBufferMidpoint){
      if(LPFBuffer[i + 1] < LPFBufferMidpoint){
        numberOfCrossings++;
      }
    }
    else if(LPFBuffer[i] < LPFBufferMidpoint){
      if(LPFBuffer[i + 1] > LPFBufferMidpoint){
        numberOfCrossings++; 
      }
    }
  }
  // Average step length between men and women = 0.716m. https://marathonhandbook.com/average-stride-length
  numberOfSteps += (numberOfCrossings / 2); // Add (numberOfCrossings / 2) to the cumulative numbers of steps since booting the device.
  Speed = (360 / (1000 * 2))*(numberOfCrossings * 0.716); // 360 10-second periods in an hour. Divide by (1000*2) to convert to km/hr while accounting for both upward and downward crossings.
  warpPrint("4. Speed: %d.\n", Speed);
}
*/
WarpStatus updateAccelerations(){
	uint16_t XLSB, YLSB, ZLSB;
	uint16_t XMSB, YMSB, ZMSB;
	WarpStatus i2cReadStatus;

	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);
	
	// Firstly, update the x-axis acceleration.	
	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 2 /* numberOfBytes */);
	if (i2cReadStatus != kWarpStatusOK){
		warpPrint("Failed to read x-axis acceleration.\n");
		return i2cReadStatus;
	}
	XMSB	      = deviceMMA8451QState.i2cBuffer[0];
	XLSB          = deviceMMA8451QState.i2cBuffer[1];
	XAcceleration = ((XMSB & 0xFF) << 6) | (XLSB >> 2);
	XAcceleration = (XAcceleration ^ (1 << 13)) - (1 << 13);
	warpPrint("XAcceleration: %d.\n", XAcceleration);

	// Secondly, update the y-axis acceleration.	
	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Y_MSB, 2 /* numberOfBytes */);
	if (i2cReadStatus != kWarpStatusOK){
		warpPrint("Failed to read y-axis acceleration.\n");
		return i2cReadStatus;
	}
	YMSB	      = deviceMMA8451QState.i2cBuffer[0];
	YLSB          = deviceMMA8451QState.i2cBuffer[1];
	YAcceleration = ((YMSB & 0xFF) << 6) | (YLSB >> 2);
	YAcceleration = (YAcceleration ^ (1 << 13)) - (1 << 13);
	warpPrint("YAcceleration: %d.\n", YAcceleration);

	// Finally, update the y-axis acceleration.	
	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Z_MSB, 2 /* numberOfBytes */);
	if (i2cReadStatus != kWarpStatusOK){
		warpPrint("Failed to read z-axis acceleration.\n");
		return i2cReadStatus;
	}
	ZMSB	      = deviceMMA8451QState.i2cBuffer[0];
	ZLSB          = deviceMMA8451QState.i2cBuffer[1];
	ZAcceleration = ((ZMSB & 0xFF) << 6) | (ZLSB >> 2);
	ZAcceleration = (ZAcceleration ^ (1 << 13)) - (1 << 13);
	warpPrint("ZAcceleration: %d.\n", ZAcceleration);

	return 0;
}

void printGUI(){
	/*
	 *	Clear Screen
	 */
	clearDisplay();
}

void printNumber(uint8_t column, uint8_t row, uint8_t number){
	switch(number){
		case 0:
		{
			printLine(column + 1, row - 1, column + 1, row - 7, 0xFF, 0x00, 0x00);
			printLine(column + 4, row - 1, column + 4, row - 7, 0xFF, 0x00, 0x00);
			printLine(column + 1, row - 1, column + 4, row - 1, 0xFF, 0x00, 0x00);
			printLine(column + 1, row - 7, column + 4, row - 7, 0xFF, 0x00, 0x00);	
			break;
		}
		case 1:
		{
			break;
		}
		case 2:
		{
			break;
		}
		case 3:
		{
			break;
		}
		case 4:
		{
			break;
		}
		case 5:
		{
			break;
		}
		case 6:
		{
			break;
		}
		case 7:
		{
			break;
		}
		case 8:
		{
			break;
		}
		case 9:
		{
			break;
		}
	}
}
