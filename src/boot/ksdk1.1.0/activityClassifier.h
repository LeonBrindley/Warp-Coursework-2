#pragma once

/*
typedef enum
{
	kMMA8451QXAxis			= 0x0,
	kMMA8451QYAxis		        = 0x1,
	kMMA8451QZAxis		        = 0x2,
} MMA8451QConstants;
*/

typedef enum
{
	ActivityStationary		= 0x0,
	ActivityWalking		        = 0x1,
	ActivityRunning		        = 0x2,
} detectedActivity;

void classifierAlgorithm();

/*
void stepOneCombine();
void stepTwoFilter();
void stepThreeMidpoint();
void stepFourSpeed();
*/

WarpStatus updateAccelerations();

#define SAMPLE_PERIOD 0.01 // Take a sample every 10ms across all three cartesian axes.
#define BUFFER_SIZE 19 // Size of AccelerationBuffer and LPFBuffer. LPF must have an odd number of taps.

// Define variables for the X, Y and Z acceleration measurements (16-bit 2's complement integers). Initialise to 0.
int32_t XCombined = 0;
int32_t YCombined = 0;
int32_t ZCombined = 0;

uint32_t accelerationMagnitude = 0;

// Declare activityReading as the enumerated type detectedActivity defined above.
detectedActivity activityReading;

// Define buffers to store the magnitudes of the acceleration measurements before and after low-pass filtering.
uint16_t AccelerationBuffer[BUFFER_SIZE] = {0}; // Initialised to 0.
uint32_t LPFBuffer[BUFFER_SIZE] = {0}; // Initialised to 0.
uint16_t absXCombined, absYCombined, absZCombined; // Absolute values of the X, Y and Z acceleration measurements.
uint16_t maximalAcceleration; // Maximum value among the X, Y and Z acceleration measurements.
uint32_t maximumValue, minimumValue; // Not set until the classification algorithm runs.
uint32_t LPFBufferMidpoint; // Not set until the classification algorithm runs.
uint16_t numberOfCrossings; // The number of times the midpoint of the low-pass filtered signal is crossed.

uint16_t numberOfSteps = 0; // Cumulative number of steps since booting the device. Initialised to 0.
float speed = 0; // Estimated speed in km/hr. Initialised to 0.

// The LPF uses a finite impulse response (FIR) structure.
// The FIR coefficients h(n) are defined below.
// These have been calculated using https://rfcalculator.com/FIR-Filters and then multiplied by 1,000.
uint16_t LPFWeights[BUFFER_SIZE] = {34, 459, 1942, 5428, 11757, 21089, 32428, 43607, 51876, 54932, 51876, 43607, 32428, 21089, 11757, 5428, 1942, 459, 34};

void printGUI();
void printNumber(uint8_t column, uint8_t row, uint8_t number);
