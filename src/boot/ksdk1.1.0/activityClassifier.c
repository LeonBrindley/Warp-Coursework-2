#include <stdlib.h>

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
// As I want to calculate the actual speed, the code below calculates the magnitude of the acceleration instead of simply detecting the maximal activity axis.
// Try a sampling rate of 50Hz, which matches the algorithm found at https://www.ti.com/lit/pdf/slaa599. Decrease this if there is insufficient memory.
// The algorithm referenced above requires 1.2 kilobytes of code memory and 640 bytes of data memory, but makes use of efficient fixed-point computations. 

// Combine all steps in classifierAlgorithm().

void classifierAlgorithm(){

  warpPrint("Running updateAccelerations() now.\n");
  updateAccelerations();
  warpPrint("Finished running updateAccelerations().\n");

  // Set default maximumValue and minimumValue to guarantee that they are updated in the for loop below.
  maximumValue = 0;
  minimumValue = 4294967295;
  warpPrint("LPFBuffer[%d] Before Update: %d.\n", BUFFER_SIZE - 1, LPFBuffer[BUFFER_SIZE - 1]);
	
  for (int i = 0; i < BUFFER_SIZE; i++){
    LPFBuffer[BUFFER_SIZE - 1] += AccelerationBuffer[i] * LPFWeights[i];
    warpPrint("2. AccelerationBuffer[%d] = %d, LPFWeights[%d] = %d, LPFBuffer[%d] = %d.\n", i, AccelerationBuffer[i], i, LPFWeights[i], i, LPFBuffer[i]);
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
  for(int i = 1; i < BUFFER_SIZE; i++){
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
  }
	
  // Average step length between men and women = 0.716m. https://marathonhandbook.com/average-stride-length
  numberOfSteps += (numberOfCrossings / 2); // Add (numberOfCrossings / 2) to the cumulative numbers of steps since booting the device.
  speed = (360 / (1000 * 2))*(numberOfCrossings * 0.716); // 360 10-second periods in an hour. Divide by (1000*2) to convert to km/hr while accounting for both upward and downward crossings.
  warpPrint("4. Number of Steps: %d, Speed: %d.\n", numberOfSteps, speed);

}

WarpStatus updateAccelerations(){
  uint16_t XLSB, YLSB, ZLSB;
  uint16_t XMSB, YMSB, ZMSB;
  WarpStatus i2cReadStatus;

  warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);
	
  i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 6 /* numberOfBytes */);
  warpPrint("Reading acceleration measurements from MMA8451Q registers %d to %d.\n", kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, kWarpSensorOutputRegisterMMA8451QOUT_X_MSB + 5);
  if (i2cReadStatus != kWarpStatusOK){
    warpPrint("Failed to read acceleration measurements.\n");
    return i2cReadStatus;
  }
	
  XMSB = deviceMMA8451QState.i2cBuffer[0];
  XLSB = deviceMMA8451QState.i2cBuffer[1];
  XAcceleration = ((XMSB & 0xFF) << 6) | (XLSB >> 2);
  XAcceleration = (XAcceleration ^ (1 << 13)) - (1 << 13);
  warpPrint("XMSB: %d.\n", XMSB);
  warpPrint("XLSB: %d.\n", XLSB);
  warpPrint("XAcceleration: %d.\n", XAcceleration);
  YMSB = deviceMMA8451QState.i2cBuffer[2];
  YLSB = deviceMMA8451QState.i2cBuffer[3];
  YAcceleration = ((YMSB & 0xFF) << 6) | (YLSB >> 2);
  YAcceleration = (YAcceleration ^ (1 << 13)) - (1 << 13);
  warpPrint("YMSB: %d.\n", YMSB);
  warpPrint("YLSB: %d.\n", YLSB);
  warpPrint("YAcceleration: %d.\n", YAcceleration);
  ZMSB = deviceMMA8451QState.i2cBuffer[4];
  ZLSB = deviceMMA8451QState.i2cBuffer[5];
  ZAcceleration = ((ZMSB & 0xFF) << 6) | (ZLSB >> 2);
  ZAcceleration = (ZAcceleration ^ (1 << 13)) - (1 << 13);
  warpPrint("ZMSB: %d.\n", ZMSB);
  warpPrint("ZLSB: %d.\n", ZLSB);
  warpPrint("ZAcceleration: %d.\n", ZAcceleration);

  // Shift AccelerationBuffer and LPFBuffer left to free up space for new data.
  for (int i = 1; i < BUFFER_SIZE; i++){
    AccelerationBuffer[i - 1] = AccelerationBuffer[i];
    LPFBuffer[i - 1] = LPFBuffer[i];
  }
  
  // Identify the maximal activity axis as sqrt() is too big for the FRDM-KL03Z's memory.
  absXAcceleration = abs(XAcceleration);
  absYAcceleration = abs(YAcceleration);
  absZAcceleration = abs(ZAcceleration);
		
  if(absXAcceleration > absYAcceleration){
    maximalAcceleration = absXAcceleration;
  }
  else{
    maximalAcceleration = absYAcceleration;
  }
  if(absZAcceleration > maximalAcceleration){
    maximalAcceleration = absZAcceleration;
  }
  
  AccelerationBuffer[BUFFER_SIZE - 1] =  maximalAcceleration;
  warpPrint("1. Maximal Acceleration: %d m/s.\n", AccelerationBuffer[BUFFER_SIZE - 1]);
	
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
			printLine(column, row, column, row + 40, 0xFF, 0x00, 0x00);
			printLine(column, row, column + 20, row, 0xFF, 0x00, 0x00);
			printLine(column + 20, row + 40, column, row + 40, 0xFF, 0x00, 0x00);
			printLine(column + 20, row + 40, column + 20, row, 0xFF, 0x00, 0x00);	
			break;
		}
		case 1:
		{
			printLine(column + 20, row + 40, column + 20, row, 0xFF, 0x00, 0x00);	
			break;
		}
		case 2:
		{
			printLine(column, row, column + 20, row, 0xFF, 0x00, 0x00);
			printLine(column + 20, row, column + 20, row + 20, 0xFF, 0x00, 0x00);
			printLine(column + 20, row + 20, column, row + 20, 0xFF, 0x00, 0x00);
			printLine(column, row + 20, column, row + 40, 0xFF, 0x00, 0x00);
			printLine(column, row + 40, column + 20, row + 40, 0xFF, 0x00, 0x00);
			break;
		}
		case 3:
		{
			printLine(column, row, column + 20, row, 0xFF, 0x00, 0x00);
			printLine(column + 20, row, column + 20, row + 40, 0xFF, 0x00, 0x00);
			printLine(column + 20, row + 20, column, row + 20, 0xFF, 0x00, 0x00);
			printLine(column, row + 40, column + 20, row + 40, 0xFF, 0x00, 0x00);
			break;
		}
		case 4:
		{
			printLine(column, row, column, row + 20, 0xFF, 0x00, 0x00);
			printLine(column, row + 20, column + 20, row + 20, 0xFF, 0x00, 0x00);
			printLine(column + 20, row, column + 20, row + 40, 0xFF, 0x00, 0x00);
			break;
		}
		case 5:
		{
			printLine(column + 20, row, column, row, 0xFF, 0x00, 0x00);
			printLine(column, row, column, row + 20, 0xFF, 0x00, 0x00);
			printLine(column, row + 20, column + 20, row + 20, 0xFF, 0x00, 0x00);
			printLine(column + 20, row + 20, column + 20, row + 40, 0xFF, 0x00, 0x00);
			printLine(column + 20, row + 40, column, row + 40, 0xFF, 0x00, 0x00);
			break;
		}
		case 6:
		{
			printLine(column + 20, row, column, row, 0xFF, 0x00, 0x00);
			printLine(column, row, column, row + 40, 0xFF, 0x00, 0x00);
			printLine(column, row + 40, column + 20, row + 40, 0xFF, 0x00, 0x00);
			printLine(column + 20, row + 40, column + 20, row + 20, 0xFF, 0x00, 0x00);
			printLine(column + 20, row + 20, column, row + 20, 0xFF, 0x00, 0x00);
			break;
		}
		case 7:
		{
			printLine(column, row, column + 20, row, 0xFF, 0x00, 0x00);
			printLine(column + 20, row, column + 20, row + 40, 0xFF, 0x00, 0x00);
			break;
		}
		case 8:
		{
			printLine(column, row, column + 20, row, 0xFF, 0x00, 0x00);
			printLine(column, row, column, row + 40, 0xFF, 0x00, 0x00);
			printLine(column + 20, row, column + 20, row + 40, 0xFF, 0x00, 0x00);
			printLine(column, row + 20, column + 20, row + 20, 0xFF, 0x00, 0x00);
			printLine(column, row + 40, column + 20, row + 40, 0xFF, 0x00, 0x00);
			break;
		}
		case 9:
		{
			printLine(column, row, column + 20, row, 0xFF, 0x00, 0x00);
			printLine(column, row, column, row + 20, 0xFF, 0x00, 0x00);
			printLine(column + 20, row, column + 20, row + 40, 0xFF, 0x00, 0x00);
			printLine(column, row + 20, column + 20, row + 20, 0xFF, 0x00, 0x00);
			printLine(column, row + 40, column + 20, row + 40, 0xFF, 0x00, 0x00);
			break;
		}
	}
}
