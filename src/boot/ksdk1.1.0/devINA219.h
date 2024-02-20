// TI INA219 Datasheet: https://www.vle.cam.ac.uk/mod/resource/view.php?id=9835022
// Adafruit INA219 User Guide: https://www.vle.cam.ac.uk/mod/resource/view.php?id=9835032

// 0x0: Configuration, 0x1: Shunt, 0x2: Bus, 0x3: Power, 0x4: Current, 0x5: Calibration
typedef enum
{
	kINA219RegConfiguration		= 0x0,
	kINA219RegShunt		        = 0x1,
	kINA219RegBus		        = 0x2,
	kINA219RegPower		        = 0x3,
	kINA219RegCurrent		= 0x4,
	kINA219RegCalibration		= 0x5,
} INA219Constants;

void		initINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);

// Function to read from the INA219 registers.
WarpStatus	readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes);

// Function to write to the INA219 registers.
WarpStatus	writeSensorRegisterINA219(uint8_t deviceRegister, uint8_t payload);

// Additional functions to return the shunt voltage, bus voltage, current or power.
int16_t returnShunt(void);
int16_t returnBus(void);
int16_t returnCurrent(void);
uint16_t returnPower(void);
