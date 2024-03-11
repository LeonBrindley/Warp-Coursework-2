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
// Note that the %f (float) format specifier does not work with SEGGER_RTT_printf, instead use %d (decimal).
// Details of bit manipulation with the MMA8451Q can be found at https://www.nxp.com/docs/en/application-note/AN4076.pdf.

int32_t convertAcceleration(int16_t number){ // Convert the acceleration from multiples of (1/1024)g to mms^-2. 
  // Acceleration is given in multiples of (1/1024)g with the chosen +/- 8g range and 14-bit resolution of the MMA8451Q readings.
  // Hence, multiply by 9810 and then divide by 1024 to convert to mms^-2. Therefore, there is an implicit scaling factor of 1,000.
  int32_t result = ((int32_t)(number) * 9810) / 1024;
  return result;
}

uint32_t sqrtInt(uint32_t base){ // Step 1: calculate the magnitude of the acceleration using Pythagoras' theorem across three cartesian axes.
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
	warpPrint("sqrt(%d) = %d mms^-2.\n", base, root + 1);
        return (root + 1); // Add 1 to round up, so the final square root result is accurate.
      }
      else{
        // warpPrint("Guessed the number %d.\n", root);
        // warpPrint("%d != %d.\n", root, oldRoot);  
      }
    }
  }
}

void shiftBuffer(){ // Shift the AccelerationBuffer and LPFBuffer left and set the final element to 0.
  // Shift AccelerationBuffer and LPFBuffer left to free up space for new data.
  for (int i = 1; i < BUFFER_SIZE; i++){
    AccelerationBuffer[i - 1] = AccelerationBuffer[i];
    LPFBuffer[i - 1] = LPFBuffer[i];
  }
  // Reset the final element in each buffer to 0.
  AccelerationBuffer[BUFFER_SIZE - 1] = 0;
  LPFBuffer[BUFFER_SIZE - 1] = 0;
}

void applyLPF(){ // Step 2: apply a low-pass filter to the data.
  for (int i = 0; i < BUFFER_SIZE; i++){
    // Note that the LPF coefficients have all been multiplied by 1,000,000.
    LPFBuffer[BUFFER_SIZE - 1] += (AccelerationBuffer[i] * (uint32_t)LPFWeights[i]) / 1000; // Denominator of 1,000 is less than the factor of 1,000,000 mentioned above, so the results are effectively scaled by 1,000 to give ums^-2.
    // warpPrint("AccelerationBuffer[%d] = %d, LPFWeights[%d] = %d, LPFBuffer[%d] = %d.\n", i, AccelerationBuffer[i], i, LPFWeights[i], i, LPFBuffer[i]);
    // warpPrint("%d, %d\n", AccelerationBuffer[i], LPFBuffer[i]); // Use this for extracting raw data for checking the validity of the algorithm.
  }
  warpPrint("2. AccelerationBuffer[%d] = %d, LPFBuffer[%d] = %d.\n", BUFFER_SIZE - 1, AccelerationBuffer[BUFFER_SIZE - 1], BUFFER_SIZE - 1, LPFBuffer[BUFFER_SIZE - 1]);
}

void simpleDiff(){ // Step 3: search for points of inflection by considering the values either side of each data point.
  // This method works when changing rapidly from stationary to running, as the midpoint detection option may be inaccurate in this case.
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
}

void calculateSpeed(){ // Step 4: calculate the speed (in m/hr).
  cumulativeInflectionPoints += numberOfInflectionPoints; // Calculate the number of inflection points since starting the program.
  cumulativeSteps = cumulativeInflectionPoints / 2; // This line can introduce significant rounding error early on, so consider cumulativeInflectionPoints instead.
  warpPrint("3. numberOfInflectionPoints: %d, cumulativeInflectionPoints: %d, cumulativeSteps: %d.\n", numberOfInflectionPoints, cumulativeInflectionPoints, cumulativeSteps);
  
  // On average, step length = height * 0.415 in males and height * 0.413 in females: https://www.verywellfit.com/set-pedometer-better-accuracy-3432895.
  // One stride equals two steps, so the average stride length is 2 * (0.415 + 0.413) = 0.828m.
  // Dividing this figure by 2 (to account for both maxima and minima) gives the average stride length of HEIGHT * 0.414.
  distance = (cumulativeInflectionPoints * (HEIGHT * 414)) / 100; // Calculate distance travelled so far (in mm). For this formula to work, the HEIGHT must be in cm.
  speed = (distance * 3600) / (numberOfCycles * (SAMPLE_PERIOD * BUFFER_SIZE)); // Calculate speed (distance over time) so far (in m/hr). Takes SAMPLE_PERIOD * BUFFER_SIZE per cycle.
  warpPrint("4. Distance (mm): %d / Time (ms): %d = Speed (mm/s): %d, Speed (m/hr): %d.\n", distance, numberOfCycles * SAMPLE_PERIOD * BUFFER_SIZE, (speed * 10) / 36, speed); // Print time and speed in ms and m/hr, respectively, as warpPrint() can only display integers (so km/hr would be too imprecise).
}

void identifyActivity(){ // Step 5: identify the activity as running, walking or stationary.
  // "The average speed with equal amounts of walking and running (running fraction = 0.5) is about 2.2 m/s."
  // https://www.ncbi.nlm.nih.gov/pmc/articles/PMC3627106
  // Therefore, set the threshold to distinguish running from walking to 2.2 m/s (7.92 km/hr).
  if(speed > 7920){ // 7.92 km/hr = 7920 m/hr.
    activityReading = ActivityRunning; // Equals 0x2 (see enumerated type).
    warpPrint("5. Activity = Running.\n"); // "RUNNING" on OLED display.
  }
  // "Mean walking speeds of 0.50 and 0.23 m/s have been reported for older adults in hospital and geriatric rehabilitation settings, respectively."
  // https://www.ncbi.nlm.nih.gov/pmc/articles/PMC2967707
  // Therefore, set the threshold to distinguish walking from stationary to 0.23 m/s (0.828 km/hr).
  else if(speed > 828){ // 0.828 km/hr = 828 m/hr.
    activityReading = ActivityWalking; // Equals 0x1 (see enumerated type).
    warpPrint("5. Activity = Walking.\n"); // "WALKING" on OLED display.
  }
  // Finally, if the speed is below 0.23 m/s, set activityReading to ActivityStationary.
  else{
    activityReading = ActivityStationary; // Equals 0x0 (see enumerated type).
    warpPrint("5. Activity = Stationary.\n"); // "STILL" on OLED display.
    // clearDisplay();
  }
}

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

void classifierAlgorithm(){
  uint16_t XLSB, YLSB, ZLSB; // Least significant byte of each acceleration measurement.
  uint16_t XMSB, YMSB, ZMSB; // Most significant byte of each acceleration measurement.
  int32_t XAcceleration, YAcceleration, ZAcceleration; // Actual acceleration values for checking their accuracy.
  WarpStatus i2cReadStatus;

  i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 6 /* numberOfBytes */); // Read 6 bytes consecutively to get 14-bit acceleration measurements from all three axes.
  // warpPrint("\nReading acceleration measurements from MMA8451Q registers %d to %d.\n", kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, kWarpSensorOutputRegisterMMA8451QOUT_X_MSB + 5);
  if (i2cReadStatus != kWarpStatusOK){
    warpPrint("\nFailed to read acceleration measurements.\n");
    return; // Return from the classifierAlgorithm() function if the MMA8451Q can't be read from.
  }

  // warpPrint("\nParsing the bytes received from MMA8451Q's registers now.\n");	

  XMSB = deviceMMA8451QState.i2cBuffer[0];
  XLSB = deviceMMA8451QState.i2cBuffer[1];
  XCombined = ((XMSB & 0xFF) << 6) | (XLSB >> 2);
  XCombined = (XCombined ^ (1 << 13)) - (1 << 13);
  // warpPrint("XMSB: %d, XMSB: %d, XCombined - Decimal: %d, Hexadecimal: %x.\n", XMSB, XLSB, XCombined, XCombined);
  XAcceleration = convertAcceleration(XCombined);
  // warpPrint("XAcceleration (mms^-2) - Decimal: %d, Hexadecimal: %x.\n", XAcceleration, XAcceleration);

  YMSB = deviceMMA8451QState.i2cBuffer[2];
  YLSB = deviceMMA8451QState.i2cBuffer[3];
  YCombined = ((YMSB & 0xFF) << 6) | (YLSB >> 2);
  YCombined = (YCombined ^ (1 << 13)) - (1 << 13);
  // warpPrint("YMSB: %d, YMSB: %d, YCombined - Decimal: %d, Hexadecimal: %x.\n", YMSB, YLSB, YCombined, YCombined);
  YAcceleration = convertAcceleration(YCombined);
  // warpPrint("YAcceleration (mms^-2) - Decimal: %d, Hexadecimal: %x.\n", YAcceleration, YAcceleration);
	
  ZMSB = deviceMMA8451QState.i2cBuffer[4];
  ZLSB = deviceMMA8451QState.i2cBuffer[5];
  ZCombined = ((ZMSB & 0xFF) << 6) | (ZLSB >> 2);
  ZCombined = (ZCombined ^ (1 << 13)) - (1 << 13);
  // warpPrint("ZMSB: %d, ZMSB: %d, ZCombined - Decimal: %d, Hexadecimal: %x.\n", ZMSB, ZLSB, ZCombined, ZCombined);
  ZAcceleration = convertAcceleration(ZCombined);
  // warpPrint("ZAcceleration (mms^-2) - Decimal: %d, Hexadecimal: %x.\n", ZAcceleration, ZAcceleration);

  // warpPrint("Calculating the square root of %d + %d + %d.\n", XAcceleration*XAcceleration, YAcceleration*YAcceleration, ZAcceleration*ZAcceleration);
  accelerationMagnitude = sqrtInt((uint32_t)(XAcceleration*XAcceleration) + (uint32_t)(YAcceleration*YAcceleration) + (uint32_t)(ZAcceleration*ZAcceleration));
  	
  shiftBuffer();

  AccelerationBuffer[BUFFER_SIZE - 1] = accelerationMagnitude;
  warpPrint("1. Acceleration Magnitude: %dmms^-2.\n", AccelerationBuffer[BUFFER_SIZE - 1]);
	
  // if(SYNTHETIC_DATA == 1){
  //   if(exampleDataCounter < 19){	  
  //     AccelerationBuffer[BUFFER_SIZE - 1] = exampleData[exampleDataCounter];
  //     exampleDataCounter++;
  //   }
  //   else{
  //     exampleDataCounter = 0;
  //     AccelerationBuffer[BUFFER_SIZE - 1] = exampleData[exampleDataCounter];
  //     exampleDataCounter++;    
  //   }
  // }
  // else{
  //   AccelerationBuffer[BUFFER_SIZE - 1] = accelerationMagnitude;
  // }

  applyLPF(); // Step 2.
	
  if((cycleCounter == BUFFER_SIZE) && dataValid){ // The data is not valid if the AccelerationBuffer has yet to be filled, so make sure that this condition is met.
    // See https://www.vle.cam.ac.uk/pluginfile.php/27161189/mod_resource/content/1/chapter-02-measurements-and-uncertainty-and-cover.pdf.
    numberOfCycles += 1;
    warpPrint("numberOfCycles: %d.\n", numberOfCycles);
    simpleDiff(); // Step 3.
    calculateSpeed(); // Step 4.
    identifyActivity(); // Step 5.
    cycleCounter = 0;
  }
  else if((cycleCounter == BUFFER_SIZE) && !dataValid){ // When the buffer fills for the first time, reset cycleCounter but don't undertake the next steps of the algorithm.
    warpPrint("First cycle, so not performing any additional steps.\n");
    cycleCounter = 0;
    dataValid = 1;
  }
  cycleCounter++;
}

/*

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

void printWalking(){ // 7 letters = width of 35, with spacing of 2 between each letter.
  clearDisplay(); // 128 - 35 - (6 * 2) = 81, so start at trunc(81 / 2) = 40.
  printCharacter(40, 10, 16); // W = 16.
  printCharacter(47, 10, 17); // A = 17.
  printCharacter(54, 10, 18); // L = 18.
  printCharacter(61, 10, 10); // K = 10.
  printCharacter(68, 10, 19); // I = 19.
  printCharacter(75, 10, 20); // N = 20.
  printCharacter(82, 10, 21); // G = 21.
}  

void printRunning(){ // 7 letters = width of 35, with spacing of 2 between each letter.
  clearDisplay(); // 128 - 35 - (6 * 2) = 81, so start at trunc(81 / 2) = 40.
  printCharacter(40, 10, 14); // R = 14.
  printCharacter(47, 10, 22); // U = 22.
  printCharacter(54, 10, 20); // N = 20.
  printCharacter(61, 10, 20); // N = 20.
  printCharacter(68, 10, 19); // I = 19.
  printCharacter(75, 10, 20); // N = 20.
  printCharacter(82, 10, 21); // G = 21.
}

void printStill(){ // 5 letters = width of 25, with spacing of 2 between each letter.
  clearDisplay(); // 128 - 25 - (4 * 2) = 95, so start at trunc(95 / 2) = 47.
  printCharacter(47, 10, 23); // S = 23.
  printCharacter(54, 10, 24); // T = 24.
  printCharacter(61, 10, 19); // I = 19.
  printCharacter(68, 10, 18); // L = 18.
  printCharacter(75, 10, 18); // L = 18.
}

*/

/*

void printGUI(){
	clearDisplay();

	printCharacter(2, 2, 4);  // 4
	printCharacter(9, 2, 25); // B
	printCharacter(16, 2, 2); // 2
	printCharacter(23, 2, 5); // 5
}

*/
