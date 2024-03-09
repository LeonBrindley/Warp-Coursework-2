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

/*
void stepOneCombine();
void stepTwoFilter();
void stepThreeMidpoint();
void stepFourSpeed();
*/

WarpStatus updateAccelerations();
void classifierAlgorithm();

#define SAMPLE_PERIOD 0.01 // Take a sample every 10ms across all three cartesian axes.
#define BUFFER_SIZE 19 // Size of AccelerationBuffer and LPFBuffer. LPF must have an odd number of taps.
#define SYNTHETIC_DATA 1 // Set this to 1 to call generateData() and overwrite the raw MMA8451Q data for testing purposes.

// Define variables for the X, Y and Z acceleration measurements (16-bit 2's complement integers). Initialise to 0.
int16_t XCombined = 0;
int16_t YCombined = 0;
int16_t ZCombined = 0;

uint32_t accelerationMagnitude = 0;
uint32_t exampleData[BUFFER_SIZE] = {500, 739, 920, 998, 954, 799, 570, 324, 121, 11, 20, 147, 360, 607, 828, 968, 994, 899, 706}; // Sine wave with known frequency of 0.5Hz for testing.
uint8_t exampleDataCounter = 0; // For selecting the correct array elements in exampleData in sequence.
uint8_t cycleCounter = 0; // This ensures the cumulative number of steps is only added to when the buffer has been fully refreshed to avoid double-counting.

// Declare activityReading as the enumerated type detectedActivity defined above.
detectedActivity activityReading;

// Define buffers to store the magnitudes of the acceleration measurements before and after low-pass filtering.
uint32_t AccelerationBuffer[BUFFER_SIZE] = {0}; // Initialised to 0.
uint32_t LPFBuffer[BUFFER_SIZE] = {0}; // Initialised to 0.
uint32_t maximumValue, minimumValue; // Not set until the classification algorithm runs.
uint32_t LPFBufferMidpoint; // Not set until the classification algorithm runs.
uint16_t numberOfCrossings = 0; // The number of times the midpoint of the low-pass filtered signal is crossed.
uint16_t numberOfInflectionPoints = 0; // The number of local maxima and minima in the array. Use numberOfInflectionPoints / 2 instead of the number of steps to avoid rounding errors.
uint16_t cumulativeNumberOfInflectionPoints = 0; // The cumulative number of inflection points since starting the program.
uint16_t cumulativeNumberOfSteps = 0; // Cumulative number of steps since booting the device. Initialised to 0.

uint32_t distance = 0; // Initialised to 0.
uint32_t speed = 0; // Initialised to 0.

// The LPF uses a finite impulse response (FIR) structure.
// The FIR coefficients h(n) are defined below.
// These have been calculated using https://rfcalculator.com/FIR-Filters and then multiplied (e.g. by 10,000) to provide integer results.
// uint16_t LPFWeights[BUFFER_SIZE] = {34, 459, 1942, 5428, 11757, 21089, 32428, 43607, 51876, 54932, 51876, 43607, 32428, 21089, 11757, 5428, 1942, 459, 34}; // OLD 19-element array.
uint16_t LPFWeights[BUFFER_SIZE] = {0, 3, 40, 238, 875, 2303, 4664, 7555, 10014, 10986, 10014, 7555, 4664, 2303, 875, 238, 40, 3, 0}; // NEW 19-element array.
// uint16_t LPFWeights[BUFFER_SIZE] = {0, 0, 4, 21, 82, 243, 604, 1316, 2578, 4618, 7658, 11853, 17232, 23649, 30751, 37991, 44691, 50136, 53694, 54932, 53694, 50136, 44691, 37991, 30751, 23649, 17232, 11853, 7658, 4618, 2578, 1316, 604, 243, 82, 21, 4, 0, 0}; // 39-element array.

uint32_t sqrtInt(uint32_t base);
void simpleDiff(); // Function to identify maxima and minima by considering the gradient on either side of each data point.

// void printGUI();
// void printNumber(uint8_t column, uint8_t row, uint8_t number);
