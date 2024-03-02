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
  // Shift AccelerationBuffer left to free up space for new data.
  for (int i = 0; i < BUFFER_SIZE - 1; i++){
    AccelerationBuffer[i] = AccelerationBuffer[i + 1];
  }
  // Store the magnitude of the acceleration in AccelerationBuffer. 
  AccelerationBuffer[BUFFER_SIZE - 1] =  sqrt((XAcceleration*XAcceleration)
    + (YAcceleration*YAcceleration) + (ZAcceleration*ZAcceleration));
}

// Step 2: Low-pass filter the result to smooth out the signal.

void stepTwoFilter(){

}

// Step 3: Perform extremal value marking by differentiating the data.

void stepThreeExtremal(){

}

// Step 4: Calculate the speed over the last 10 seconds.

void stepFourSpeed(){

}

