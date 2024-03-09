#include <stdlib.h>
#include <math.h> // Required for "sin" function to generate synthetic acceleration data.

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

/*

void generateData(){ // Function to generate synthetic acceleration data for testing purposes. To save on memory, the below code can be substituted for pre-computed values.
  float exampleTime; // exampleTime should be of the floating point type in the sinusoid below.
  warpPrint("\nGenerating synthetic acceleration data.\n");
  for (int i = 0; i < BUFFER_SIZE; i++){
    exampleTime = i * SAMPLE_PERIOD;
    AccelerationBuffer[i] = 500 + (500 * sin(2 * exampleTime)); // Make the output acceleration data a standard sinusoid for testing the algorithm.
    // The maximum value of 1000 is comparable to accelerationMagnitude below, and the offset of 500 is added so all results are positive (as the data type is uint32_t).
    warpPrint("%d, ", AccelerationBuffer[i]);
  }
  warpPrint("\nFinished generating synthetic acceleration data.\n");
}

*/

// To identify inflection points, look at the points either side of the current data point.
// This method works when changing rapidly from stationary to running, as the midpoint detection option may be inaccurate in this case.
void simpleDiff(){
  numberOfInflectionPoints = 0; // Reset numberOfInflectionPoints. Includes both maxima and minima with the implementation below.
  for(int i = 1; i < BUFFER_SIZE - 1; i++){
    if((LPFBuffer[i] > LPFBuffer[i-1]) && (LPFBuffer[i] > LPFBuffer[i+1])){ // A concave inflection point (maximum) has been reached.
      numberOfInflectionPoints = numberOfInflectionPoints + 1;
      warpPrint("%d > %d and %d > %d - MAXIMUM detected. numberOfInflectionPoints = %d.\n", LPFBuffer[i], LPFBuffer[i-1], LPFBuffer[i], LPFBuffer[i+1], numberOfInflectionPoints);
    }
    else if((LPFBuffer[i] < LPFBuffer[i-1]) && (LPFBuffer[i] < LPFBuffer[i+1])){ // A convex inflection point (minimum) has been reached.
      numberOfInflectionPoints = numberOfInflectionPoints + 1;
      warpPrint("%d < %d and %d < %d - MINIMUM detected. numberOfInflectionPoints = %d.\n", LPFBuffer[i], LPFBuffer[i-1], LPFBuffer[i], LPFBuffer[i+1], numberOfInflectionPoints);
    }
  }
  warpPrint("Final numberOfInflectionPoints inside loop = %d.\n", numberOfInflectionPoints);
  if(cycleCounter == 19){
    cycleCounter = 0;
    cumulativeNumberOfInflectionPoints += numberOfInflectionPoints;
    warpPrint("Final cumulativeNumberOfInflectionPoints inside loop = %d.\n", cumulativeNumberOfInflectionPoints);
  }
  cycleCounter++;
}

uint32_t sqrtInt(uint32_t base){
  if(base == 0 || base == 1){ // If the number equals 0 or 1, the root equals the base.
    return base;
  }
  else{
    uint32_t root = base / 8; // Guess the square root at first.
    // warpPrint("Square rooting the number %d.\n", base);
    while(1){ // Perform this iterative result until the square root is calculated.
      uint32_t oldRoot = root; // Save the old root to compare the new one to.
      root = (root / 2) + (base / (2 * root));
      if(abs(root - oldRoot) <= 1){ // <= 1 to prevent the result hopping between two adjacent numbers and failing to converge.
        return (root + 1); // Add 1 to round up, so the final square root result is accurate.
      }
      else{
        // warpPrint("Guessed the number %d.\n", root);
        // warpPrint("%d != %d.\n", root, oldRoot);  
      }
    }
  }
}

void classifierAlgorithm(){

  // warpPrint("\nDeclaring LSB, MSB and acceleration variables now.\n");
  uint16_t XLSB, YLSB, ZLSB; // Least significant byte of each acceleration measurement.
  uint16_t XMSB, YMSB, ZMSB; // Most significant byte of each acceleration measurement.
  int32_t XAcceleration, YAcceleration, ZAcceleration; // Actual acceleration values for checking their accuracy.

  // warpPrint("\nDeclaring i2cReadStatus variable now.\n");	
  WarpStatus i2cReadStatus;

  // warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);

  i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 6 /* numberOfBytes */);
  // warpPrint("\nReading acceleration measurements from MMA8451Q registers %d to %d.\n", kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, kWarpSensorOutputRegisterMMA8451QOUT_X_MSB + 5);
  if (i2cReadStatus != kWarpStatusOK){
    warpPrint("\nFailed to read acceleration measurements.\n");
    return; // Return from the classifierAlgorithm() function if the MMA8451Q can't be read from.
  }

  // LSB of acceleration readings in 14-bit mode with a full-scale range of +/-4g = 8g/16384 = 0.488mg.
  // Therefore, to convert to the acceleration to ums^-2, multiply by (488 * 9.81) = 4787 to the nearest integer.
  // Note that the %f (float) format specifier does not work with SEGGER_RTT_printf, instead use %d (decimal).
  // Details of bit manipulation with the MMA8451Q can be found at https://www.nxp.com/docs/en/application-note/AN4076.pdf.

  // warpPrint("\nParsing the bytes received from MMA8451Q's registers now.\n");	

  XMSB = deviceMMA8451QState.i2cBuffer[0];
  XLSB = deviceMMA8451QState.i2cBuffer[1];
  // warpPrint("Calculating XCombined now.\n");
  XCombined = ((XMSB & 0xFF) << 6) | (XLSB >> 2);
  XCombined = (XCombined ^ (1 << 13)) - (1 << 13);
  // warpPrint("Calculating XAcceleration now.\n");
  XAcceleration = (int32_t)(XCombined) * 4787; // Convert the acceleration to ums^-2.
  XAcceleration = XAcceleration / 1000; // Convert the acceleration to mms^-2.
  // warpPrint("XMSB: %d.\n", XMSB);
  // warpPrint("XLSB: %d.\n", XLSB);
  // warpPrint("XCombined - Decimal: %d, Hexadecimal: %x.\n", XCombined, XCombined);
  // warpPrint("XAcceleration (mms^-2) - Decimal: %d, Hexadecimal: %x.\n", XAcceleration, XAcceleration);

  YMSB = deviceMMA8451QState.i2cBuffer[2];
  YLSB = deviceMMA8451QState.i2cBuffer[3];
  // warpPrint("Calculating YCombined now.\n");
  YCombined = ((YMSB & 0xFF) << 6) | (YLSB >> 2);
  YCombined = (YCombined ^ (1 << 13)) - (1 << 13);
  // warpPrint("Calculating YAcceleration now.\n");
  YAcceleration = (int32_t)(YCombined) * 4787; // Convert the acceleration to ums^-2.
  YAcceleration = YAcceleration / 1000; // Convert the acceleration to mms^-2.
  // warpPrint("YMSB: %d.\n", YMSB);
  // warpPrint("YLSB: %d.\n", YLSB);
  // warpPrint("YCombined - Decimal: %d, Hexadecimal: %x.\n", YCombined, YCombined);
  // warpPrint("YAcceleration (mms^-2) - Decimal: %d, Hexadecimal: %x.\n", YAcceleration, YAcceleration);

  ZMSB = deviceMMA8451QState.i2cBuffer[4];
  ZLSB = deviceMMA8451QState.i2cBuffer[5];
  // warpPrint("Calculating ZCombined now.\n");
  ZCombined = ((ZMSB & 0xFF) << 6) | (ZLSB >> 2);
  ZCombined = (ZCombined ^ (1 << 13)) - (1 << 13);
  // warpPrint("Calculating ZAcceleration now.\n");
  ZAcceleration = (int32_t)(ZCombined) * 4787; // Convert the acceleration to ums^-2.
  ZAcceleration = ZAcceleration / 1000; // Convert the acceleration to mms^-2.
  // warpPrint("ZMSB: %d.\n", ZMSB);
  // warpPrint("ZLSB: %d.\n", ZLSB);
  // warpPrint("ZCombined - Decimal: %d, Hexadecimal: %x.\n", ZCombined, ZCombined);
  // warpPrint("ZAcceleration (mms^-2) - Decimal: %d, Hexadecimal: %x.\n", ZAcceleration, ZAcceleration);

  accelerationMagnitude = sqrtInt((uint32_t)(XAcceleration*XAcceleration) + (uint32_t)(YAcceleration*YAcceleration) + (uint32_t)(ZAcceleration*ZAcceleration));

  // Shift AccelerationBuffer and LPFBuffer left to free up space for new data.
  for (int i = 1; i < BUFFER_SIZE; i++){
    AccelerationBuffer[i - 1] = AccelerationBuffer[i];
    LPFBuffer[i - 1] = LPFBuffer[i];
  }

  // Reset the final element in each buffer to 0.
  AccelerationBuffer[BUFFER_SIZE - 1] = 0;
  LPFBuffer[BUFFER_SIZE - 1] = 0;

  if(SYNTHETIC_DATA == 1){
    if(exampleDataCounter < 19){	  
      AccelerationBuffer[BUFFER_SIZE - 1] = exampleData[exampleDataCounter];
      exampleDataCounter++;
    }
    else{
      exampleDataCounter = 0;
      AccelerationBuffer[BUFFER_SIZE - 1] = exampleData[exampleDataCounter];
      exampleDataCounter++;    
    }
  }
  else{
    AccelerationBuffer[BUFFER_SIZE - 1] = accelerationMagnitude;	
  }
  warpPrint("1. Acceleration Magnitude (mms^-2): %d.\n", AccelerationBuffer[BUFFER_SIZE - 1]);

  // warpPrint("LPFBuffer[%d] Before Update: %d.\n", BUFFER_SIZE - 1, LPFBuffer[BUFFER_SIZE - 1]);
	
  for (int i = 0; i < BUFFER_SIZE; i++){
    LPFBuffer[BUFFER_SIZE - 1] += ((uint32_t)AccelerationBuffer[i] * (uint32_t)LPFWeights[i]) / 1000; // Divide by 1000 to avoid 32-bit overflow when the values are summed.
    warpPrint("2. AccelerationBuffer[%d] = %d, LPFWeights[%d] = %d, LPFBuffer[%d] = %d.\n", i, AccelerationBuffer[i], i, LPFWeights[i], i, LPFBuffer[i]);
  }

  // See https://www.vle.cam.ac.uk/pluginfile.php/27161189/mod_resource/content/1/chapter-02-measurements-and-uncertainty-and-cover.pdf.
  simpleDiff(); // Identify the maxima and minima of the low-pass filtered waveform.
  warpPrint("3. numberOfInflectionPoints: %d.\n", numberOfInflectionPoints);
	
  // Average step length between men and women = 0.716m. https://marathonhandbook.com/average-stride-length
  // Dividing this by 2 (to account for both maxima and minima) gives the figure 358mm.
  distance = numberOfInflectionPoints * 358; // Calculate distance travelled over the previous 10-second period (in mm).
  speed = (distance * 36) / 100; // Calculate speed over the previous 10-second period (in m/hr).
  warpPrint("4. Distance (mm): %d, Speed (mm/s): %d, Speed (m/hr): %d.\n", distance, (speed * 10) / 36, speed); // Print speed in m/hr as warpPrint() can only display integers (so km/hr would be too imprecise).

  // "The average speed with equal amounts of walking and running (running fraction = 0.5) is about 2.2 m/s."
  // https://www.ncbi.nlm.nih.gov/pmc/articles/PMC3627106
  // Therefore, set the threshold to distinguish running from walking to 2.2 m/s (7.92 km/hr).
  if(speed > 7920){ // 7.92 km/hr = 7920 m/hr.
    activityReading = ActivityRunning; // Equals 0x2.
    warpPrint("5. Activity = Running.\n");
  }
  // "Mean walking speeds of 0.50 and 0.23 m/s have been reported for older adults in hospital and geriatric rehabilitation settings, respectively."
  // https://www.ncbi.nlm.nih.gov/pmc/articles/PMC2967707
  // Therefore, set the threshold to distinguish walking from stationary to 0.23 m/s (0.828 km/hr).
  else if(speed > 828){ // 0.828 km/hr = 828 m/hr.
    activityReading = ActivityWalking; // Equals 0x1.
    warpPrint("5. Activity = Walking.\n");
  }
  // Finally, if the speed is below 0.23 m/s, set activityReading to ActivityStationary.
  else{
    activityReading = ActivityStationary; // Equals 0x0.
    warpPrint("5. Activity = Stationary.\n");
    clearDisplay();
    printCharacter(23);
  }
}

void printCharacter(uint8_t column, uint8_t row, uint8_t number){
	switch(number){
		case 0: // Number 0.
		{
			printLine(column, row, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row, column + 5, row, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 10, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 10, column + 5, row, 0xFF, 0x00, 0x00);	
			break;
		}
		case 1: // Number 1.
		{
			printLine(column + 5, row + 10, column + 5, row, 0xFF, 0x00, 0x00);	
			break;
		}
		case 2: // Number 2.
		{
			printLine(column, row, column + 5, row, 0xFF, 0x00, 0x00);
			printLine(column + 5, row, column + 5, row + 5, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 5, column, row + 5, 0xFF, 0x00, 0x00);
			printLine(column, row + 5, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row + 10, column + 5, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 3: // Number 3.
		{
			printLine(column, row, column + 5, row, 0xFF, 0x00, 0x00);
			printLine(column + 5, row, column + 5, row + 10, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 5, column, row + 5, 0xFF, 0x00, 0x00);
			printLine(column, row + 10, column + 5, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 4: // Number 4.
		{
			printLine(column, row, column, row + 5, 0xFF, 0x00, 0x00);
			printLine(column, row + 5, column + 5, row + 5, 0xFF, 0x00, 0x00);
			printLine(column + 5, row, column + 5, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 5: // Number 5.
		{
			printLine(column + 5, row, column, row, 0xFF, 0x00, 0x00);
			printLine(column, row, column, row + 5, 0xFF, 0x00, 0x00);
			printLine(column, row + 5, column + 5, row + 5, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 5, column + 5, row + 10, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 10, column, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 6: // Number 6.
		{
			printLine(column + 5, row, column, row, 0xFF, 0x00, 0x00);
			printLine(column, row, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row + 10, column + 5, row + 10, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 10, column + 5, row + 5, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 5, column, row + 5, 0xFF, 0x00, 0x00);
			break;
		}
		case 7: // Number 7.
		{
			printLine(column, row, column + 5, row, 0xFF, 0x00, 0x00);
			printLine(column + 5, row, column + 5, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 8: // Number 8.
		{
			printLine(column, row, column + 5, row, 0xFF, 0x00, 0x00);
			printLine(column, row, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column + 5, row, column + 5, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row + 5, column + 5, row + 5, 0xFF, 0x00, 0x00);
			printLine(column, row + 10, column + 5, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 9: // Number 9.
		{
			printLine(column, row, column + 5, row, 0xFF, 0x00, 0x00);
			printLine(column, row, column, row + 5, 0xFF, 0x00, 0x00);
			printLine(column + 5, row, column + 5, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row + 5, column + 5, row + 5, 0xFF, 0x00, 0x00);
			printLine(column, row + 10, column + 5, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 10: // Letter 'K'.
		{
			printLine(column, row, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row + 5, column + 5, row, 0xFF, 0x00, 0x00);
			printLine(column, row + 5, column + 5, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 11: // Letter 'M'.
		{
			printLine(column, row + 5, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row + 5, column + 10, row + 5, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 5, column + 5, row + 10, 0xFF, 0x00, 0x00);
			printLine(column + 10, row + 5, column + 10, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 12: // Character '/'.
		{
			printLine(column, row + 10, column + 5, row, 0xFF, 0x00, 0x00);
			break;
		}
		case 13: // Letter 'H'.
		{
			printLine(column, row, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row + 5, column + 5, row + 5, 0xFF, 0x00, 0x00);
			printLine(column + 5, row, column + 5, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 14: // Character 'R'.
		{
			printLine(column, row, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row, column + 5, row, 0xFF, 0x00, 0x00);
			printLine(column + 5, row, column + 5, row + 5, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 5, column, row + 5, 0xFF, 0x00, 0x00);
			printLine(column, row + 5, column + 5, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 15: // Character '.'.
		{
			printLine(column + 2, row + 10, column + 2, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 16: // Character 'W'.
		{
			printLine(column, row, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row + 10, column + 2, row + 8, 0xFF, 0x00, 0x00);
			printLine(column + 3, row + 8, column + 5, row + 10, 0xFF, 0x00, 0x00);
			printLine(column + 10, row, column + 10, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 17: // Character 'A'.
		{
			printLine(column, row + 5, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row + 5, column + 2, row, 0xFF, 0x00, 0x00);
			printLine(column + 3, row, column + 5, row + 5, 0xFF, 0x00, 0x00);
			printLine(column, row + 5, column + 5, row + 5, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 5, column + 5, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 18: // Character 'L'.
		{
			printLine(column, row, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row + 10, column + 5, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 19: // Character 'I'.
		{
			printLine(column + 2, row + 10, column + 2, row, 0xFF, 0x00, 0x00);	
			break;
		}
		case 20: // Character 'N'.
		{
			printLine(column, row, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row, column + 5, row + 10, 0xFF, 0x00, 0x00);
			printLine(column + 5, row, column + 5, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 21: // Character 'G'.
		{
			printLine(column + 5, row, column, row, 0xFF, 0x00, 0x00);
			printLine(column, row, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row + 10, column + 5, row + 10, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 10, column + 5, row + 5, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 5, column + 2, row + 5, 0xFF, 0x00, 0x00);
			break;
		}
		case 22: // Character 'U'.
		{
			printLine(column, row, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row + 10, column + 5, row + 10, 0xFF, 0x00, 0x00);
			printLine(column + 5, row, column + 5, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 23: // Character 'S'.
		{
			printLine(column + 5, row, column + 1, row, 0xFF, 0x00, 0x00);
			printLine(column + 1, row, column, row + 1, 0xFF, 0x00, 0x00);
			printLine(column, row + 1, column, row + 4, 0xFF, 0x00, 0x00);
			printLine(column, row + 4, column + 1, row + 5, 0xFF, 0x00, 0x00);
			printLine(column + 1, row + 5, column + 4, row + 5, 0xFF, 0x00, 0x00);
			printLine(column + 4, row + 5, column + 5, row + 6, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 6, column + 5, row + 9, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 9, column + 4, row + 10, 0xFF, 0x00, 0x00);
			printLine(column + 4, row + 10, column, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 24: // Character 'T'.
		{
			printLine(column, row, column + 5, row, 0xFF, 0x00, 0x00);
			printLine(column + 2, row, column + 2, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 25: // Character 'B'.
		{
			printLine(column, row, column + 4, row, 0xFF, 0x00, 0x00);
			printLine(column + 4, row, column + 5, row + 1, 0xFF, 0x00, 0x00);
			printLine(column, row, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 1, column + 5, row + 4, 0xFF, 0x00, 0x00);
			printLine(column, row + 5, column + 4, row + 5, 0xFF, 0x00, 0x00);
			printLine(column + 4, row + 5, column + 5, row + 4, 0xFF, 0x00, 0x00);
			printLine(column + 4, row + 5, column + 5, row + 6, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 6, column + 5, row + 9, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 9, column + 4, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row + 10, column + 4, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
	}
}

/*

void printGUI(){
	clearDisplay();

	printCharacter(2, 2, 4);  // 4
	printCharacter(9, 2, 25); // B
	printCharacter(16, 2, 2); // 2
	printCharacter(23, 2, 5); // 5
}

*/
