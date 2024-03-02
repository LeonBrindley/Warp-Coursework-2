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

void stepOneCombine();
void stepTwoFilter();
void stepThreeExtremal();
void stepFourSpeed();

// const uint16_t circularBufferSize = 8000;
// uint8_t circularBufferHead = 0;
// uint8_t circularBufferTail = 0;
// uint8_t MMA8451QBuffer[circularBufferSize];
