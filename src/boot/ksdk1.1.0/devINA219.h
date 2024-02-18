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

// Functions to read from the INA219 registers.
WarpStatus	readSensorRegisterINA219Configuration(uint8_t deviceRegister, int numberOfBytes);
WarpStatus	readSensorRegisterINA219Shunt(uint8_t deviceRegister, int numberOfBytes);
WarpStatus	readSensorRegisterINA219Bus(uint8_t deviceRegister, int numberOfBytes);
WarpStatus	readSensorRegisterINA219Power(uint8_t deviceRegister, int numberOfBytes);
WarpStatus	readSensorRegisterINA219Current(uint8_t deviceRegister, int numberOfBytes);
WarpStatus	readSensorRegisterINA219Calibration(uint8_t deviceRegister, int numberOfBytes);

// Functions to write to the INA219 registers.
WarpStatus	writeSensorRegisterINA219Configuration(uint8_t deviceRegister, uint8_t payload);
WarpStatus	writeSensorRegisterINA219Shunt(uint8_t deviceRegister, uint8_t payload);
WarpStatus	writeSensorRegisterINA219Bus(uint8_t deviceRegister, uint8_t payload);
WarpStatus	writeSensorRegisterINA219Power(uint8_t deviceRegister, uint8_t payload);
WarpStatus	writeSensorRegisterINA219Current(uint8_t deviceRegister, uint8_t payload);
WarpStatus	writeSensorRegisterINA219Calibration(uint8_t deviceRegister, uint8_t payload);
