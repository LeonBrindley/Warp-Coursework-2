#include <stdlib.h>
#include <math.h> // Required for sqrt() function.

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

void stepOneCombine(){
  // Shift AccelerationBuffer and LPFBuffer left to free up space for new data.
  for (int i = 0; i < BUFFER_SIZE - 2; i++){
    AccelerationBuffer[i] = AccelerationBuffer[i + 1];
    LPFBuffer[i] = LPFBuffer[i + 1];
  }
  // Store the magnitude of the acceleration in AccelerationBuffer. 
  AccelerationBuffer[BUFFER_SIZE - 1] =  sqrt((XAcceleration*XAcceleration)
    + (YAcceleration*YAcceleration) + (ZAcceleration*ZAcceleration));
  warpPrint("Latest Magnitude: %d m/s.\n", AccelerationBuffer[BUFFER_SIZE - 1]);
}

// Step 2: Low-pass filter the result to smooth out the signal.

void stepTwoFilter(){
  for (int i = 0; i < BUFFER_SIZE - 1; i++){
    LPFBuffer[i] = AccelerationBuffer[i] * LPFWeights[i];
    warpPrint("LPFBuffer[%d]: %d.\n", i, LPFBuffer[i]);
  }
}

// Step 3: Perform extremal value marking by differentiating the data.

void stepThreeExtremal(){
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
  Speed = (360/1000)*(numberOfCrossings * 0.716); // 360 10-second periods in an hour. Divide by 1000 to convert to km/hr.
  warpPrint("Speed: %d.\n", Speed);
}
