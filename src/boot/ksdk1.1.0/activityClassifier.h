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

#define SAMPLE_PERIOD 0.01 // Take a sample every 10ms.
#define BUFFER_SIZE 19 // Size of AccelerationBuffer and LPFBuffer. LPF must have an odd number of taps.

// Define variables for the X, Y and Z acceleration measurements. Initialise to 0.
float XAcceleration = 0;
float YAcceleration = 0;
float ZAcceleration = 0;

// Declare activityReading as the enumerated type detectedActivity defined above.
detectedActivity activityReading;

// Define buffers to store the magnitudes of the acceleration measurements before and after low-pass filtering.
float AccelerationBuffer[BUFFER_SIZE] = {0}; // Initialised to 0.
float LPFBuffer[BUFFER_SIZE] = {0}; // Initialised to 0.
float absXAcceleration, absYAcceleration, absZAcceleration; // Absolute values of the X, Y and Z acceleration measurements.
float maximalAcceleration; // Maximum value among the X, Y and Z acceleration measurements.
float maximumValue, minimumValue, LPFBufferMidpoint, numberOfCrossings; // Not set until the classification algorithm runs.

uint16_t numberOfSteps = 0; // Cumulative number of steps since booting the device. Initialised to 0.
float speed = 0; // Estimated speed in km/hr. Initialised to 0.

// The LPF uses a finite impulse response (FIR) structure.
// The FIR coefficients h(n) are defined below.
// These have been calculated using https://rfcalculator.com/FIR-Filters and then multiplied by 1,000.
float LPFWeights[BUFFER_SIZE] = {3.4, 45.9, 194.2, 542.8, 1175.7, 2108.9, 3242.8, 4360.7, 5187.6, 5493.2, 5187.6, 4360.7, 3242.8, 2108.9, 1175.7, 542.8, 194.2, 45.9, 3.4};

void printGUI();
void printNumber(uint8_t column, uint8_t row, uint8_t number);
