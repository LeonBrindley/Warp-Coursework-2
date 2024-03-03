/*
typedef enum
{
	kMMA8451QXAxis			= 0x0,
	kMMA8451QYAxis		        = 0x1,
	kMMA8451QZAxis		        = 0x2,
} MMA8451QConstants;

typedef enum
{
	ActivityStationary		= 0x0,
	ActivityWalking		        = 0x1,
	ActivityRunning		        = 0x2,
} ActivityBeingPerformed;
*/

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

// Define variables for the X, Y and Z acceleration measurements.
int16_t XAcceleration, YAcceleration, ZAcceleration;

// Define buffers to store the magnitudes of the acceleration measurements before and after low-pass filtering.
uint16_t AccelerationBuffer[BUFFER_SIZE] = {0}; // Initialised to 0.
uint16_t LPFBuffer[BUFFER_SIZE] = {0}; // Initialised to 0.
uint16_t maximumValue, minimumValue, LPFBufferMidpoint, numberOfCrossings; // Not set until the classification algorithm runs.

uint16_t numberOfSteps = 0; // Cumulative number of steps since booting the device. Initialised to 0.
float speed = 0; // Estimated speed in km/hr. Initialised to 0.

// The LPF uses a finite impulse response (FIR) structure.
// The FIR coefficients h(n) are defined below.
// These have been calculated using https://rfcalculator.com/FIR-Filters.
int16_t LPFWeights[BUFFER_SIZE] = {0, 3, 13, 43, 102, 198, 323, 451, 549, 586, 549, 451, 323, 198, 102, 43, 13, 3, 0};

void printGUI();
void printNumber(uint8_t column, uint8_t row, uint8_t number);
