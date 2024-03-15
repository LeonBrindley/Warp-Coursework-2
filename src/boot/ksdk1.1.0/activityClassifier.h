#pragma once

typedef enum
{
	ActivityStationary		= 0x0,
	ActivityWalking		        = 0x1,
	ActivityRunning		        = 0x2,
} detectedActivity;

WarpStatus updateAccelerations();
void classifierAlgorithm();

#define HEIGHT 180 // Height in cm. If this unit is incorrect, then the calculated speed will be also.
#define SAMPLE_PERIOD 25 // Take a sample every 25ms (40 times per second) across all three cartesian axes.
#define BUFFER_SIZE 39 // Size of AccelerationBuffer and LPFBuffer. LPF must have an odd number of taps.
// #define SYNTHETIC_DATA 0 // Set this to 1 to call generateData() and overwrite the raw MMA8451Q data for testing purposes.

bool dataValid = 0; // Data is not valid until the AccelerationBuffer is filled, so initialise dataValid to 0.
bool firstTimeRunning = 1; // Used to overwrite the lastElement and secondToLastElement variables during the first cycle, as these must be set such that a peak cannot be detected in the first element.

// Define variables for the X, Y and Z acceleration measurements (16-bit 2's complement integers). Initialise to 0.
int16_t XCombined = 0;
int16_t YCombined = 0;
int16_t ZCombined = 0;

uint32_t accelerationMagnitude = 0;
// uint32_t exampleData[BUFFER_SIZE] = {500, 739, 920, 998, 954, 799, 570, 324, 121, 11, 20, 147, 360, 607, 828, 968, 994, 899, 706}; // Sine wave with known frequency of 0.5Hz for testing.
// uint8_t exampleDataCounter = 0; // For selecting the correct array elements in exampleData in sequence.
uint8_t cycleCounter = 0; // This ensures that cumulativeInflectionPoints is only added to when the buffer has been fully refreshed (i.e. cycleCounter = BUFFER_SIZE) to avoid double-counting.
uint8_t numberOfCycles = 0; // This is incremented whenever cycleCounter resets, so the time expended across multiple cycles can be accounted for when calculating the speed.

// Declare activityReading as the enumerated type detectedActivity defined above.
detectedActivity activityReading;

// Define buffers to store the magnitudes of the acceleration measurements before and after low-pass filtering.
uint32_t AccelerationBuffer[BUFFER_SIZE] = {0}; // Initialised to 0.
uint32_t LPFBuffer[BUFFER_SIZE] = {0}; // Initialised to 0.
uint32_t lastElement, secondToLastElement = {0}; // Initialised to 0.
uint16_t numberOfInflectionPoints = 0; // The number of local maxima and minima in the array. Use numberOfInflectionPoints / 2 instead of the number of steps to avoid rounding errors.
uint16_t cumulativeInflectionPoints = 0; // The cumulative number of inflection points since starting the program.
uint16_t generateDataCycle = 0; // Count the number of times generateData() has run so the sine wave can continue in subsequent cycles. Initialised to 0.

uint32_t distance = 0; // Initialised to 0.
uint32_t time = 0; // Initialised to 0.
uint32_t speed = 0; // Initialised to 0.

uint16_t characteristicUncertainty; // Uncertainty due to the characteristic velocity being between 1.80 and 2.50ms^-1.

// The LPF uses a finite impulse response (FIR) structure.
// The FIR coefficients h(n) are defined below.
// These have been calculated using https://rfcalculator.com/FIR-Filters and then multiplied (e.g. by 10,000) to provide integer results.
// uint16_t LPFWeights[BUFFER_SIZE] = {34, 459, 1942, 5428, 11757, 21089, 32428, 43607, 51876, 54932, 51876, 43607, 32428, 21089, 11757, 5428, 1942, 459, 34}; // OLD 19-element array.
// uint16_t LPFWeights[BUFFER_SIZE] = {0, 3, 40, 238, 875, 2303, 4664, 7555, 10014, 10986, 10014, 7555, 4664, 2303, 875, 238, 40, 3, 0}; // NEW 19-element array.
uint16_t LPFWeights[BUFFER_SIZE] = {0, 0, 4, 21, 82, 243, 604, 1316, 2578, 4618, 7658, 11853, 17232, 23649, 30751, 37991, 44691, 50136, 53694, 54932, 53694, 50136, 44691, 37991, 30751, 23649, 17232, 11853, 7658, 4618, 2578, 1316, 604, 243, 82, 21, 4, 0, 0}; // 39-element array.
// uint16_t LPFWeights[BUFFER_SIZE] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000}; // Dummy coefficients for testing the cut-off frequency using a moving average (movag) filter. This gives a phase-modulated sinc() function.

int32_t convertAcceleration(int16_t number);
uint32_t sqrtInt(uint32_t base); // Return the square root of an integer using the Newton-Raphson method
void shiftBuffer();
void applyLPF(); // Step 2.
void simpleDiff(); // Step 3.
void calculateSpeed(); // Step 4.
void identifyActivity(); // Step 5.

// Variables for dealing with uncertainty in the result.
bool firstExcessTest = 1; // Set to 0 when the first inflection point is detected.
uint16_t firstExcessTime, finalInflectionTime, finalExcessTime;

// Usain Bolt ran at 4.4 steps per second during the fastest 20m segment of his run in the Berlin World Championships: https://posemethod.com/usain-bolts-running-technique.
// Therefore, if the number of steps is greater than 5 steps per second, there must be a secondary maximum or minimum (which must be discarded).
uint16_t minimumSamples = 0; // If the number of samples since the last inflection point is less than 10 (i.e. 200ms), discard any inflection points. Initialised to 0.

uint16_t timeBefore = 0; // Initialising timeBefore variable to 0.
uint16_t timeAfter = 0; // Initialising timeAfter variable to 0.
uint16_t timeDifference = 0; // Initialising timeDifference variable to 0.
