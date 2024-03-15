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

// void generateData(){ // Function to generate synthetic acceleration data for testing purposes. To save on memory, the below code can be substituted for pre-computed values.
//   float exampleTime; // exampleTime should be of the floating point type in the sinusoid below.
//   warpPrint("\nGenerating synthetic acceleration data.\n");
//   for (int i = 0; i < BUFFER_SIZE; i++){
//     exampleTime = ((generateDataCycle * 39) + i) * 10; // Multiple of 10ms used in the example in 4B25 lecture 2.
//     AccelerationBuffer[i] = 500 + (500 * sin(40 * exampleTime)); // Make the output acceleration data a standard sinusoid for testing the algorithm.
//     // The maximum value of 1000 is comparable to accelerationMagnitude below, and the offset of 500 is added so all results are positive (as the data type is uint32_t).
//     warpPrint("%d, ", AccelerationBuffer[i]);
//   }
//   warpPrint("\nFinished generating synthetic acceleration data.\n");
//   generateDateCycle++;
// }

int32_t convertAcceleration(int16_t number){ // Convert the acceleration from multiples of (1/1024)g to mms^-2. 
  // Acceleration is given in multiples of (1/1024)g with the chosen +/- 8g range and 14-bit resolution of the MMA8451Q readings.
  // Hence, multiply by 9810 and then divide by 1024 to convert to mms^-2. Therefore, there is an implicit scaling factor of 1,000.
  int32_t result = ((int32_t)(number) * 9810) / 1024;
  // warpPrint("convertAcceleration(): %d * (9810/1024) = %d.\n", number, result);
  return result;
}

uint32_t sqrtInt(uint32_t base){ // Step 1: calculate the magnitude of the acceleration using Pythagoras' theorem across three cartesian axes.
  if(base == 0 || base == 1){ // If the number equals 0 or 1, the root equals the base.
    return base;
  }
  else{
    uint32_t root = base / 8; // Guess the square root at first.
    // warpPrint("sqrtInt(): Square rooting the number %d.\n", base);
    while(1){ // Perform this iterative result until the square root is calculated.
      uint32_t oldRoot = root; // Save the old root to compare the new one to.
      root = (root / 2) + (base / (2 * root));
      if(abs(root - oldRoot) <= 1){ // <= 1 to prevent the result hopping between two adjacent numbers and failing to converge.
	// warpPrint("sqrtInt(): Square root of %d = %d mms^-2.\n", base, root + 1);
        return (root + 1); // Add 1 to round up, so the final square root result is accurate.
      }
      else{
        // warpPrint("sqrtInt(): Guessed the number %d.\n", root);
        // warpPrint("sqrtInt(): %d != %d.\n", root, oldRoot);  
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
  // warpPrint("shiftBuffer(): AccelerationBuffer[BUFFER_SIZE - 1] = %d.\n", AccelerationBuffer[BUFFER_SIZE - 1]);
  LPFBuffer[BUFFER_SIZE - 1] = 0;
  // warpPrint("shiftBuffer(): LPFBuffer[BUFFER_SIZE - 1] = %d.\n", LPFBuffer[BUFFER_SIZE - 1]);
}

void applyLPF(){ // Step 2: apply a low-pass filter to the data.
  for (int i = 0; i < BUFFER_SIZE; i++){
    // Note that the LPF coefficients have all been multiplied by 1,000,000 (so floating-point multiplication is not required).
    LPFBuffer[BUFFER_SIZE - 1] += (AccelerationBuffer[i] * (uint32_t)LPFWeights[i]) / 1000; // Denominator of 1,000 is less than the factor of 1,000,000 mentioned above, so the results are effectively scaled by 1,000 to give ums^-2.
    // warpPrint("applyLPF(): AccelerationBuffer[%d] = %d, LPFWeights[%d] = %d, LPFBuffer[%d] = %d.\n", i, AccelerationBuffer[i], i, LPFWeights[i], i, LPFBuffer[i]);
    // warpPrint("applyLPF(), %d, %d\n", AccelerationBuffer[i], LPFBuffer[i]); // Use this for extracting raw data into a CSV for checking the validity of the LPF.
  }
  warpPrint("2. AccelerationBuffer[%d] = %d, LPFBuffer[%d] = %d.\n", BUFFER_SIZE - 1, AccelerationBuffer[BUFFER_SIZE - 1], BUFFER_SIZE - 1, LPFBuffer[BUFFER_SIZE - 1]);
  // warpPrint("%d, %d\n", AccelerationBuffer[BUFFER_SIZE - 1], LPFBuffer[BUFFER_SIZE - 1]); // For copying to a CSV file.
}

void simpleDiff(){ // Step 3: search for points of inflection by considering the values either side of each data point.
  // This method works when changing rapidly from stationary to running, as the midpoint detection option may be inaccurate in this case.
  numberOfInflectionPoints = 0; // Reset numberOfInflectionPoints. Includes both maxima and minima with the implementation below.

  // Firstly, check if the last element of the previous LPFBuffer was an inflection point.
  if((lastElement > secondToLastElement) && (lastElement > LPFBuffer[0]) && (minimumSamples > 10)){ // A concave inflection point (maximum) has been reached.
    numberOfInflectionPoints = numberOfInflectionPoints + 1;
    minimumSamples = 0;
    // warpPrint("simpleDiff(): %d > %d and %d > %d - MAXIMUM detected in LPFBuffer[-1].\n", lastElement, secondToLastElement, lastElement, LPFBuffer[0]);
    if(firstExcessTest){ // Runs if this is the first inflection point of the experiment.
      firstExcessTest = 0;
      firstExcessTime = (numberOfCycles - 1) * (SAMPLE_PERIOD * BUFFER_SIZE);
      // warpPrint("simpleDiff(): firstExcessTime: %d.\n", firstExcessTime);
    }
    finalInflectionTime = (numberOfCycles - 1) * (SAMPLE_PERIOD * BUFFER_SIZE);
    // warpPrint("simpleDiff(): finalInflectionTime: %d.\n", finalInflectionTime);
  }
  else if((lastElement < secondToLastElement) && (lastElement < LPFBuffer[0]) && (minimumSamples > 10)){ // A convex inflection point (minimum) has been reached.
    numberOfInflectionPoints = numberOfInflectionPoints + 1;
    minimumSamples = 0;
    warpPrint("simpleDiff(): %d < %d and %d < %d - MINIMUM detected in LPFBuffer[-1].\n", lastElement, secondToLastElement, lastElement, LPFBuffer[0]);
    if(firstExcessTest){ // Runs if this is the first inflection point of the experiment.
      firstExcessTest = 0;
      firstExcessTime = (numberOfCycles - 1) * (SAMPLE_PERIOD * BUFFER_SIZE);
      // warpPrint("simpleDiff(): firstExcessTime: %d.\n", firstExcessTime);
    }
    finalInflectionTime = (numberOfCycles - 1) * (SAMPLE_PERIOD * BUFFER_SIZE);
    // warpPrint("simpleDiff(): finalInflectionTime: %d.\n", finalInflectionTime);
  }
  minimumSamples++;

  // Secondly, check if the first element of the current LPFBuffer is an inflection point.
  if((LPFBuffer[0] > lastElement) && (LPFBuffer[0] > LPFBuffer[1])  && (minimumSamples > 10)){ // A concave inflection point (maximum) has been reached.
    numberOfInflectionPoints = numberOfInflectionPoints + 1;
    minimumSamples = 0;
    warpPrint("simpleDiff(): %d > %d and %d > %d - MAXIMUM detected in LPFBuffer[0].\n", LPFBuffer[0], lastElement, LPFBuffer[0], LPFBuffer[1]);
    if(firstExcessTest){ // Runs if this is the first inflection point of the experiment.
      firstExcessTest = 0;
      firstExcessTime = ((numberOfCycles - 1) * (SAMPLE_PERIOD * BUFFER_SIZE)) + SAMPLE_PERIOD;
      // warpPrint("simpleDiff(): firstExcessTime: %d.\n", firstExcessTime);
    }
    finalInflectionTime = ((numberOfCycles - 1) * (SAMPLE_PERIOD * BUFFER_SIZE)) + SAMPLE_PERIOD;
    // warpPrint("simpleDiff(): finalInflectionTime: %d.\n", finalInflectionTime);
  }
  else if((LPFBuffer[0] < lastElement) && (LPFBuffer[0] < LPFBuffer[1])  && (minimumSamples > 10)){ // A convex inflection point (minimum) has been reached.
    numberOfInflectionPoints = numberOfInflectionPoints + 1;
    minimumSamples = 0;
    warpPrint("simpleDiff(): %d < %d and %d < %d - MINIMUM detected in LPFBuffer[0].\n", LPFBuffer[0], lastElement, LPFBuffer[0], LPFBuffer[1]);
    if(firstExcessTest){ // Runs if this is the first inflection point of the experiment.
      firstExcessTest = 0;
      firstExcessTime = ((numberOfCycles - 1) * (SAMPLE_PERIOD * BUFFER_SIZE)) + SAMPLE_PERIOD;
      warpPrint("simpleDiff(): firstExcessTime: %d.\n", firstExcessTime);
    }
    finalInflectionTime = ((numberOfCycles - 1) * (SAMPLE_PERIOD * BUFFER_SIZE)) + SAMPLE_PERIOD;
    // warpPrint("simpleDiff(): finalInflectionTime: %d.\n", finalInflectionTime);
  }
  minimumSamples++;

  // Thirdly, check the middle 37 points of the buffer in a for loop.
  for(int i = 1; i < BUFFER_SIZE - 1; i++){
    if((LPFBuffer[i] > LPFBuffer[i-1]) && (LPFBuffer[i] > LPFBuffer[i+1])  && (minimumSamples > 10)){ // A concave inflection point (maximum) has been reached.
      numberOfInflectionPoints = numberOfInflectionPoints + 1;
      minimumSamples = 0;
      warpPrint("simpleDiff(): %d > %d and %d > %d - MAXIMUM detected in LPFBuffer[%d].\n", LPFBuffer[i], LPFBuffer[i-1], LPFBuffer[i], LPFBuffer[i+1], i);
      if(firstExcessTest){ // Runs if this is the first inflection point of the experiment.
        firstExcessTest = 0;
	firstExcessTime = ((numberOfCycles - 1) * (SAMPLE_PERIOD * BUFFER_SIZE)) + (SAMPLE_PERIOD * (i + 1));
	// warpPrint("simpleDiff(): firstExcessTime: %d.\n", firstExcessTime);
      }
      finalInflectionTime = ((numberOfCycles - 1) * (SAMPLE_PERIOD * BUFFER_SIZE)) + (SAMPLE_PERIOD * (i + 1));
      // warpPrint("simpleDiff(): finalInflectionTime: %d.\n", finalInflectionTime);
    }
    else if((LPFBuffer[i] < LPFBuffer[i-1]) && (LPFBuffer[i] < LPFBuffer[i+1]) && (minimumSamples > 10)){ // A convex inflection point (minimum) has been reached.
      numberOfInflectionPoints = numberOfInflectionPoints + 1;
      minimumSamples = 0;
      warpPrint("simpleDiff(): %d < %d and %d < %d - MINIMUM detected in LPFBuffer[%d].\n", LPFBuffer[i], LPFBuffer[i-1], LPFBuffer[i], LPFBuffer[i+1], i);
      if(firstExcessTest){ // Runs if this is the first inflection point of the experiment.
        firstExcessTest = 0;
	firstExcessTime = ((numberOfCycles - 1) * (SAMPLE_PERIOD * BUFFER_SIZE)) + (SAMPLE_PERIOD * (i + 1));
	// warpPrint("simpleDiff(): firstExcessTime: %d.\n", firstExcessTime);
      }
      finalInflectionTime = ((numberOfCycles - 1) * (SAMPLE_PERIOD * BUFFER_SIZE)) + (SAMPLE_PERIOD * (i + 1));
      // warpPrint("simpleDiff(): finalInflectionTime: %d.\n", finalInflectionTime);
    }
    minimumSamples++;
  }
  // Save the last and second-to-last elements in the range - this is required to avoid missing inflection points which occur in the 0th and 38th elements of the buffer.
  lastElement = LPFBuffer[BUFFER_SIZE - 1];
  secondToLastElement = LPFBuffer[BUFFER_SIZE - 2];
}

void calculateSpeed(){ // Step 4: calculate the speed (in m/hr).
  cumulativeInflectionPoints += numberOfInflectionPoints; // Calculate the total number of inflection points since starting the program.
  warpPrint("3. numberOfInflectionPoints: %d, cumulativeInflectionPoints: %d.\n", numberOfInflectionPoints, cumulativeInflectionPoints);

  // To account for the excess time either side of the first/final inflection point, find out the fraction of the total program runtime this represents.
  // The greater the fraction, the greater the uncertainty introduced by this effect. To account for this, subtract (firstExcessTime + finalExcessTime) from the result and subtract 1 from cumulativeInflectionPoints.
  finalExcessTime = numberOfCycles * (SAMPLE_PERIOD * BUFFER_SIZE) - finalInflectionTime; // Additional time to the right of the final inflectionPoint;
  cumulativeInflectionPoints -= 1; // Subtract 1 from the cumulativeInflectionPoints, as with an inflection point on both ends of the program runtime (as enforced above), the frequency will be slightly overestimated.
	
  // On average, step length = height * 0.415 in males and height * 0.413 in females: https://www.verywellfit.com/set-pedometer-better-accuracy-3432895.
  // Note that one stride equals two steps, so the average stride length is 0.415 + 0.413 = 0.828 times an individual's height.
  // As both maxima and minima are accounted for, the average step length of HEIGHT * 0.414 is used.
  distance = (cumulativeInflectionPoints * (HEIGHT * 414)) / 100; // Calculate distance travelled so far (in mm). For this formula to work, the HEIGHT must be in cm.
  time = (numberOfCycles * SAMPLE_PERIOD * BUFFER_SIZE) - (firstExcessTime + finalExcessTime);
  // warpPrint("calculateSpeed(): time = (%d * %d * %d) - (%d + %d) = %d ms.\n", numberOfCycles, SAMPLE_PERIOD, BUFFER_SIZE, firstExcessTime, finalExcessTime, time); // Simpler code when not debugging.
  // warpPrint("calculateSpeed(): time = (numberOfCycles (%d) * SAMPLE_PERIOD (%d) * BUFFER_SIZE (%d)) - (firstExcessTime (%d) + finalExcessTime (%d)) = %d ms.\n", numberOfCycles, SAMPLE_PERIOD, BUFFER_SIZE, firstExcessTime, finalExcessTime, time);
  speed = (distance * 3600) / time; // Calculate speed (distance over time) so far (in m/hr). Takes SAMPLE_PERIOD * BUFFER_SIZE per cycle.
  warpPrint("4. Distance (mm): %d / Time (ms): %d = Speed (mm/s): %d, Speed (m/hr): %d.\n", distance, time, (speed * 10) / 36, speed); // Print time and speed in ms and m/hr, respectively, as warpPrint() can only display integers (so km/hr would be too imprecise).
}

void identifyActivity(){ // Step 5: identify the activity as running, walking or stationary.
  // "The average speed with equal amounts of walking and running (running fraction = 0.5) is about 2.2 m/s."
  // https://www.ncbi.nlm.nih.gov/pmc/articles/PMC3627106
  // "The speed at which the transition from walking to running naturally occurs is not well defined: different observations have reported values ranging from 1.80 to 2.50 m/s.
  // https://link.springer.com/article/10.1007/s00421-002-0654-9
  // Therefore, to calculate uncertainty, consider range from 1.80 m/s (6.48 km/hr) to 2.50m/s (9 km/hr).
  if(speed > 9000){ // 9 km/hr = 9000 m/hr.
    activityReading = ActivityRunning; // Equals 0x2 (see enumerated type). Represented by "RUNNING" on OLED display.
    characteristicUncertainty = 0;
    // warpPrint("5. Activity = Running (Confidence Level = %d Percent).\n", 100 - characteristicUncertainty);
  }
    // clearDisplay();
    // printRunning(); // See devSSD1331.c for printRunning() and printCharacter().
  else if(speed > 6480){ // 6.48 km/hr = 6480 m/hr.
    activityReading = ActivityRunning; // Equals 0x2 (see enumerated type).
    characteristicUncertainty = (100 * (9000 - speed)) / (9000 - 6480);
    // warpPrint("5. Activity = Running (Confidence Level = %d Percent), Walking (Confidence Level = %d Percent).\n", 100 - characteristicUncertainty, characteristicUncertainty);
  }
  // "Mean walking speeds of 0.50 and 0.23 m/s have been reported for older adults in hospital and geriatric rehabilitation settings, respectively."
  // https://www.ncbi.nlm.nih.gov/pmc/articles/PMC2967707
  // "The mean (95% CI) walking speed for slow pace (in apparently healthy adults) was 0.82 (0.77–0.86) m/s.
  // https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7806575
  // Therefore, to calculate uncertainty, consider range from 0.23 m/s (0.828 km/hr) to 0.82m/s (2.952 km/hr).
  else if(speed > 2952){ // 0.82 m/s = 2.952 km/hr = 2952 m/hr.
    activityReading = ActivityWalking; // Equals 0x1 (see enumerated type). Represented by "WALKING" on OLED display.
    characteristicUncertainty = 0;
    // warpPrint("5. Activity = Walking (Confidence Level = %d Percent).\n", 100 - characteristicUncertainty);
    // clearDisplay();
    // printWalking(); // See devSSD1331.c for printWalking() and printCharacter().
  }
  else if(speed > 828){ // 0.23 m/s = 0.828 km/hr = 828 m/hr.
    activityReading = ActivityWalking; // Equals 0x1 (see enumerated type).
    characteristicUncertainty = (100 * (2952 - speed)) / (2952 - 828);
    // warpPrint("5. Activity = Walking (Confidence Level = %d Percent), Stationary (Confidence Level = %d Percent).\n", 100 - characteristicUncertainty, characteristicUncertainty);
  }
  // Finally, if the speed is below 0.23 m/s, set activityReading to ActivityStationary.
  else{
    activityReading = ActivityStationary; // Equals 0x0 (see enumerated type). Represented by "STILL" on OLED display.
    characteristicUncertainty = 0;
    warpPrint("5. Activity = Stationary (Confidence Level = %d Percent).\n", 100 - characteristicUncertainty);
    // clearDisplay();
    // printStill();  // See devSSD1331.c for printStill() and printCharacter().
  }
  // warpPrint("\n------------------------------------------------------------------------------------------\n"); // Print a divide to make it easier to study the output.
}

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

  // warpPrint("\nParsing the bytes received from the MMA8451Q's registers now.\n");	

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
  uint16_t timeBeforeFunc = OSA_TimeGetMsec();
  accelerationMagnitude = sqrtInt((uint32_t)(XAcceleration*XAcceleration) + (uint32_t)(YAcceleration*YAcceleration) + (uint32_t)(ZAcceleration*ZAcceleration));
  uint16_t timeAfterFunc = OSA_TimeGetMsec();
  uint16_t timeDifferenceFunc = timeAfterFunc - timeBeforeFunc; 
  // warpPrint("EXECUTION TIME of sqrtInt() = %d - %d = %dms.\n", timeAfterFunc, timeBeforeFunc, timeDifferenceFunc);
	
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

  timeBeforeFunc = OSA_TimeGetMsec();
  applyLPF(); // Step 2.
  timeAfterFunc = OSA_TimeGetMsec();
  timeDifferenceFunc = timeAfterFunc - timeBeforeFunc; 
  // warpPrint("EXECUTION TIME of applyLPF() = %d - %d = %dms.\n", timeAfterFunc, timeBeforeFunc, timeDifferenceFunc);
	
  if((cycleCounter == BUFFER_SIZE) && dataValid){ // The data is not valid if the AccelerationBuffer has yet to be filled, so make sure that this condition is met.
    // See https://www.vle.cam.ac.uk/pluginfile.php/27161189/mod_resource/content/1/chapter-02-measurements-and-uncertainty-and-cover.pdf for ideas.
    numberOfCycles += 1;
    if(firstTimeRunning){
      lastElement = LPFBuffer[0]; // Make sure the first element tested by the algorithm isn't erroneously detected as an inflection point. This ensures the > and < conditions cannot be met for the first element.
      secondToLastElement = LPFBuffer[0]; // Make sure the first element tested by the algorithm isn't erroneously detected as an inflection point. This ensures the > and < conditions cannot be met for the first element.
      // warpPrint("\nfirstTimeRunning = %d, numberOfCycles = %d, LPFBuffer[0] = %d, lastElement = %d, secondToLastElement = %d.\n",  firstTimeRunning, numberOfCycles, LPFBuffer[0], lastElement, secondToLastElement); // Print the work of this if statement to check that it functions correctly.
      firstTimeRunning = 0;
    }
    else{
      // warpPrint("\nnumberOfCycles = %d, LPFBuffer[0] = %d, lastElement = %d, secondToLastElement = %d.\n", numberOfCycles, LPFBuffer[0], lastElement, secondToLastElement); // Print the work of this if statement to check that it functions correctly.    
    }

    timeBeforeFunc = OSA_TimeGetMsec();
    simpleDiff(); // Step 3.
    timeAfterFunc = OSA_TimeGetMsec();
    timeDifferenceFunc = timeAfterFunc - timeBeforeFunc; 
    // warpPrint("EXECUTION TIME of simpleDiff() = %d - %d = %dms.\n", timeAfterFunc, timeBeforeFunc, timeDifferenceFunc);
	  
    timeBeforeFunc = OSA_TimeGetMsec();
    calculateSpeed(); // Step 4.
    timeAfterFunc = OSA_TimeGetMsec();
    timeDifferenceFunc = timeAfterFunc - timeBeforeFunc; 
    // warpPrint("EXECUTION TIME of calculateSpeed() = %d - %d = %dms.\n", timeAfterFunc, timeBeforeFunc, timeDifferenceFunc);

    timeBeforeFunc = OSA_TimeGetMsec();	
    identifyActivity(); // Step 5.
    timeAfterFunc = OSA_TimeGetMsec();
    timeDifferenceFunc = timeAfterFunc - timeBeforeFunc; 
    // warpPrint("EXECUTION TIME of identifyActivity() = %d - %d = %dms.\n", timeAfterFunc, timeBeforeFunc, timeDifferenceFunc);

    // warpPrint("FINAL: Distance (%d mm) / Time (%d ms) = Speed (%d m/hr).\n", distance, time, speed); 
	    
    cycleCounter = 0;
  }
  else if((cycleCounter == BUFFER_SIZE) && !dataValid){ // When the buffer fills for the first time, reset cycleCounter but don't undertake the next steps of the algorithm.
    warpPrint("\nFirst cycle, so not performing any additional steps.\n");
    cycleCounter = 0;
    dataValid = 1;
  }
  cycleCounter++;
}
