#include <stdint.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devSSD1331.h"

volatile uint8_t	inBuffer[32];
volatile uint8_t	payloadBytes[32];

/*
 *	Override Warp firmware's use of these pins and define new aliases.
 */
// enum
// {
// 	kSSD1331PinMOSI		= GPIO_MAKE_PIN(HW_GPIOA, 8),
// 	kSSD1331PinSCK		= GPIO_MAKE_PIN(HW_GPIOA, 9),
// 	kSSD1331PinCSn		= GPIO_MAKE_PIN(HW_GPIOB, 13),
// 	kSSD1331PinDC		= GPIO_MAKE_PIN(HW_GPIOA, 12),
// 	kSSD1331PinRST		= GPIO_MAKE_PIN(HW_GPIOB, 0),
// };

static int
writeCommand(uint8_t commandByte)
{
	spi_status_t status;

	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

	/*
	 *	Drive DC low (command).
	 */
	GPIO_DRV_ClearPinOutput(kSSD1331PinDC);

	payloadBytes[0] = commandByte;
	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)&payloadBytes[0],
					(uint8_t * restrict)&inBuffer[0],
					1		/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);

	/*
	 *	Drive /CS high
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

	return status;
}

int
devSSD1331init(void)
{
	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Re-configure SPI to be on PTA8 and PTA9 for MOSI and SCK respectively.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt3);

	warpEnableSPIpins();

	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Reconfigure to use as GPIO.
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 13u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 12u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 0u, kPortMuxAsGpio);


	/*
	 *	RST high->low->high.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_ClearPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);

	/*
	 *	Initialization sequence, borrowed from https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino
	 */
	writeCommand(kSSD1331CommandDISPLAYOFF);	// 0xAE
	writeCommand(kSSD1331CommandSETREMAP);		// 0xA0
	writeCommand(0x72);				// RGB Color
	writeCommand(kSSD1331CommandSTARTLINE);		// 0xA1
	writeCommand(0x0);
	writeCommand(kSSD1331CommandDISPLAYOFFSET);	// 0xA2
	writeCommand(0x0);
	writeCommand(kSSD1331CommandNORMALDISPLAY);	// 0xA4
	writeCommand(kSSD1331CommandSETMULTIPLEX);	// 0xA8
	writeCommand(0x3F);				// 0x3F 1/64 duty
	writeCommand(kSSD1331CommandSETMASTER);		// 0xAD
	writeCommand(0x8E);
	writeCommand(kSSD1331CommandPOWERMODE);		// 0xB0
	writeCommand(0x0B);
	writeCommand(kSSD1331CommandPRECHARGE);		// 0xB1
	writeCommand(0x31);
	writeCommand(kSSD1331CommandCLOCKDIV);		// 0xB3
	writeCommand(0xF0);				// 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8A
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGEB);	// 0x8B
	writeCommand(0x78);
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8C
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGELEVEL);	// 0xBB
	writeCommand(0x3A);
	writeCommand(kSSD1331CommandVCOMH);		// 0xBE
	writeCommand(0x3E);
	writeCommand(kSSD1331CommandMASTERCURRENT);	// 0x87
	writeCommand(0x06);
	writeCommand(kSSD1331CommandCONTRASTA);		// 0x81
	writeCommand(0x91);
	writeCommand(kSSD1331CommandCONTRASTB);		// 0x82
	writeCommand(0x50);
	writeCommand(kSSD1331CommandCONTRASTC);		// 0x83
	writeCommand(0x7D);
	writeCommand(kSSD1331CommandDISPLAYON);		// Turn on oled panel
	SEGGER_RTT_WriteString(0, "\r\n\tDone with initialization sequence...\n");
	
	/*
	 *	To use fill commands, you will have to issue a command to the display to enable them. See the manual.
	 */
	writeCommand(kSSD1331CommandFILL);
	writeCommand(0x01);
	SEGGER_RTT_WriteString(0, "\r\n\tDone with enabling fill...\n");

	/*
	 *	Clear Screen
	 */
	writeCommand(kSSD1331CommandCLEAR);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);
	SEGGER_RTT_WriteString(0, "\r\n\tDone with screen clear...\n");

	/*
	 *	Any post-initialization drawing commands (i.e. turning the screen bright green) go here.
	 */
	//...

	/*
	 *	Find the maximum values of each setting in the SSD1331 manual:
	 *	https://www.vle.cam.ac.uk/pluginfile.php/13561382/mod_resource/content/1/SSD1331_1.2.pdf
	 */
	
	/*
	 *	Set the precharge factor of channels A, B and C to their maximum value of 0xFF (0b11111111).
	 */	
	// writeCommand(kSSD1331CommandPRECHARGEA);
	// writeCommand(0xFF);
	// writeCommand(kSSD1331CommandPRECHARGEB);
	// writeCommand(0xFF);
	// writeCommand(kSSD1331CommandPRECHARGEC);
	// writeCommand(0xFF);

	/*
	 *	Set the precharge voltage to its maximum value of 0x1F (0b11111).
	 */
	// writeCommand(kSSD1331CommandPRECHARGELEVEL);
	// writeCommand(0x3E);
	
	/*
	 *	Set the master current to its maximum value of 0x0F (0b1111).
	 */
	// writeCommand(kSSD1331CommandMASTERCURRENT);
	// writeCommand(0x0F);
	
	/*
	 *	Set the contrast of channels A, B and C to their maximum value of 0xFF (0b11111111).
	 */	
	// writeCommand(kSSD1331CommandCONTRASTA);
	// writeCommand(0xFF);
	// writeCommand(kSSD1331CommandCONTRASTB);
	// writeCommand(0xFF);
	// writeCommand(kSSD1331CommandCONTRASTC);
	// writeCommand(0xFF);

	/*
	 *	Ensure the OLED display is turned on.
	 */	
	// writeCommand(kSSD1331CommandDISPLAYON);

	/*
	 *	See devSSD1331.h for drawing commands.
	 *	See the SSD1331 manual for explanations.  	
	 */	

	/*
	 *	writeCommand(kSSD1331CommandDISPLAYALLON);
	 */
	
	// writeCommand(kSSD1331CommandDRAWRECT);
	// writeCommand(0x00); // A[6:0]:  Column Address of Start 
	// writeCommand(0x00); // B[5:0]:  Row Address of Start
	// writeCommand(0x5F); // C[6:0]:  Column Address of End
	// writeCommand(0x3F); // D[5:0]:  Row Address of End
	// writeCommand(0x00); // E[5:1]:  Color C of the line (BLUE)
	// writeCommand(0x3F); // F[5:0]:  Color B of the line (GREEN)
	// writeCommand(0x00); // G[5:1]:  Color A of the line (RED)
	// writeCommand(0x00); // H[5:1]:  Color C of the fill area (BLUE)
	// writeCommand(0x3F); // I[5:0]:  Color B of the fill area (GREEN)
	// writeCommand(0x00); // J[5:1]:  Color A of the fill area (RED)
	// SEGGER_RTT_WriteString(0, "\r\n\tDone with draw rectangle...\n");

	return 0;
}

void clearDisplay(){
	writeCommand(kSSD1331CommandCLEAR);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);
	SEGGER_RTT_WriteString(0, "\r\n\tDone with screen clear...\n");
}

void printLine(uint8_t colStart, uint8_t rowStart, uint8_t colEnd, uint8_t rowEnd, uint8_t blue, uint8_t green, uint8_t red){
	writeCommand(kSSD1331CommandDRAWLINE);
	writeCommand(colStart); // A[6:0]:  Column Address of Start 
	writeCommand(rowStart); // B[5:0]:  Row Address of Start
	writeCommand(colEnd); // C[6:0]:  Column Address of End
	writeCommand(rowEnd); // D[5:0]:  Row Address of End
	writeCommand(blue); // E[5:1]:  Color C of the line (BLUE)
	writeCommand(green); // F[5:0]:  Color B of the line (GREEN)
	writeCommand(red); // G[5:1]:  Color A of the line (RED)
}

void printRect(uint8_t colStart, uint8_t rowStart, uint8_t colEnd, uint8_t rowEnd, uint8_t blueLine, uint8_t greenLine, uint8_t redLine, uint8_t blueFill, uint8_t greenFill, uint8_t redFill){
	writeCommand(kSSD1331CommandDRAWRECT);
	writeCommand(colStart); // A[6:0]:  Column Address of Start 
	writeCommand(rowStart); // B[5:0]:  Row Address of Start
	writeCommand(colEnd); // C[6:0]:  Column Address of End
	writeCommand(rowEnd); // D[5:0]:  Row Address of End
	writeCommand(blueLine); // E[5:1]:  Color C of the line (BLUE)
	writeCommand(greenLine); // F[5:0]:  Color B of the line (GREEN)
	writeCommand(redLine); // G[5:1]:  Color A of the line (RED)
	writeCommand(blueFill); // H[5:1]:  Color C of the fill area (BLUE)
	writeCommand(greenFill); // I[5:0]:  Color B of the fill area (GREEN)
	writeCommand(redFill); // J[5:1]:  Color A of the fill area (RED)
}

void printCharacter(uint8_t column, uint8_t row, uint8_t number){
	switch(number){
		case 0: // Number 0.
		{
			printLine(column, row, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row, column + 5, row, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 10, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 10, column + 5, row, 0xFF, 0x00, 0x00);	
			break;
		}
		case 1: // Number 1.
		{
			printLine(column + 5, row + 10, column + 5, row, 0xFF, 0x00, 0x00);	
			break;
		}
		case 2: // Number 2.
		{
			printLine(column, row, column + 5, row, 0xFF, 0x00, 0x00);
			printLine(column + 5, row, column + 5, row + 5, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 5, column, row + 5, 0xFF, 0x00, 0x00);
			printLine(column, row + 5, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row + 10, column + 5, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 3: // Number 3.
		{
			printLine(column, row, column + 5, row, 0xFF, 0x00, 0x00);
			printLine(column + 5, row, column + 5, row + 10, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 5, column, row + 5, 0xFF, 0x00, 0x00);
			printLine(column, row + 10, column + 5, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 4: // Number 4.
		{
			printLine(column, row, column, row + 5, 0xFF, 0x00, 0x00);
			printLine(column, row + 5, column + 5, row + 5, 0xFF, 0x00, 0x00);
			printLine(column + 5, row, column + 5, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 5: // Number 5.
		{
			printLine(column + 5, row, column, row, 0xFF, 0x00, 0x00);
			printLine(column, row, column, row + 5, 0xFF, 0x00, 0x00);
			printLine(column, row + 5, column + 5, row + 5, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 5, column + 5, row + 10, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 10, column, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 6: // Number 6.
		{
			printLine(column + 5, row, column, row, 0xFF, 0x00, 0x00);
			printLine(column, row, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row + 10, column + 5, row + 10, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 10, column + 5, row + 5, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 5, column, row + 5, 0xFF, 0x00, 0x00);
			break;
		}
		case 7: // Number 7.
		{
			printLine(column, row, column + 5, row, 0xFF, 0x00, 0x00);
			printLine(column + 5, row, column + 5, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 8: // Number 8.
		{
			printLine(column, row, column + 5, row, 0xFF, 0x00, 0x00);
			printLine(column, row, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column + 5, row, column + 5, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row + 5, column + 5, row + 5, 0xFF, 0x00, 0x00);
			printLine(column, row + 10, column + 5, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 9: // Number 9.
		{
			printLine(column, row, column + 5, row, 0xFF, 0x00, 0x00);
			printLine(column, row, column, row + 5, 0xFF, 0x00, 0x00);
			printLine(column + 5, row, column + 5, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row + 5, column + 5, row + 5, 0xFF, 0x00, 0x00);
			printLine(column, row + 10, column + 5, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 10: // Letter 'K'.
		{
			printLine(column, row, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row + 5, column + 5, row, 0xFF, 0x00, 0x00);
			printLine(column, row + 5, column + 5, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 11: // Letter 'M'.
		{
			printLine(column, row + 5, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row + 5, column + 10, row + 5, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 5, column + 5, row + 10, 0xFF, 0x00, 0x00);
			printLine(column + 10, row + 5, column + 10, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 12: // Character '/'.
		{
			printLine(column, row + 10, column + 5, row, 0xFF, 0x00, 0x00);
			break;
		}
		case 13: // Letter 'H'.
		{
			printLine(column, row, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row + 5, column + 5, row + 5, 0xFF, 0x00, 0x00);
			printLine(column + 5, row, column + 5, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 14: // Character 'R'.
		{
			printLine(column, row, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row, column + 5, row, 0xFF, 0x00, 0x00);
			printLine(column + 5, row, column + 5, row + 5, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 5, column, row + 5, 0xFF, 0x00, 0x00);
			printLine(column, row + 5, column + 5, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 15: // Character '.'.
		{
			printLine(column + 2, row + 10, column + 2, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 16: // Character 'W'.
		{
			printLine(column, row, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row + 10, column + 2, row + 8, 0xFF, 0x00, 0x00);
			printLine(column + 3, row + 8, column + 5, row + 10, 0xFF, 0x00, 0x00);
			printLine(column + 10, row, column + 10, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 17: // Character 'A'.
		{
			printLine(column, row + 5, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row + 5, column + 2, row, 0xFF, 0x00, 0x00);
			printLine(column + 3, row, column + 5, row + 5, 0xFF, 0x00, 0x00);
			printLine(column, row + 5, column + 5, row + 5, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 5, column + 5, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 18: // Character 'L'.
		{
			printLine(column, row, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row + 10, column + 5, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 19: // Character 'I'.
		{
			printLine(column + 2, row + 10, column + 2, row, 0xFF, 0x00, 0x00);	
			break;
		}
		case 20: // Character 'N'.
		{
			printLine(column, row, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row, column + 5, row + 10, 0xFF, 0x00, 0x00);
			printLine(column + 5, row, column + 5, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 21: // Character 'G'.
		{
			printLine(column + 5, row, column, row, 0xFF, 0x00, 0x00);
			printLine(column, row, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row + 10, column + 5, row + 10, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 10, column + 5, row + 5, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 5, column + 2, row + 5, 0xFF, 0x00, 0x00);
			break;
		}
		case 22: // Character 'U'.
		{
			printLine(column, row, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row + 10, column + 5, row + 10, 0xFF, 0x00, 0x00);
			printLine(column + 5, row, column + 5, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 23: // Character 'S'.
		{
			printLine(column + 5, row, column + 1, row, 0xFF, 0x00, 0x00);
			printLine(column + 1, row, column, row + 1, 0xFF, 0x00, 0x00);
			printLine(column, row + 1, column, row + 4, 0xFF, 0x00, 0x00);
			printLine(column, row + 4, column + 1, row + 5, 0xFF, 0x00, 0x00);
			printLine(column + 1, row + 5, column + 4, row + 5, 0xFF, 0x00, 0x00);
			printLine(column + 4, row + 5, column + 5, row + 6, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 6, column + 5, row + 9, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 9, column + 4, row + 10, 0xFF, 0x00, 0x00);
			printLine(column + 4, row + 10, column, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 24: // Character 'T'.
		{
			printLine(column, row, column + 5, row, 0xFF, 0x00, 0x00);
			printLine(column + 2, row, column + 2, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
		case 25: // Character 'B'.
		{
			printLine(column, row, column + 4, row, 0xFF, 0x00, 0x00);
			printLine(column + 4, row, column + 5, row + 1, 0xFF, 0x00, 0x00);
			printLine(column, row, column, row + 10, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 1, column + 5, row + 4, 0xFF, 0x00, 0x00);
			printLine(column, row + 5, column + 4, row + 5, 0xFF, 0x00, 0x00);
			printLine(column + 4, row + 5, column + 5, row + 4, 0xFF, 0x00, 0x00);
			printLine(column + 4, row + 5, column + 5, row + 6, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 6, column + 5, row + 9, 0xFF, 0x00, 0x00);
			printLine(column + 5, row + 9, column + 4, row + 10, 0xFF, 0x00, 0x00);
			printLine(column, row + 10, column + 4, row + 10, 0xFF, 0x00, 0x00);
			break;
		}
	}
} 

void printWalking(){ // 7 letters = width of 35, with spacing of 2 between each letter.
  clearDisplay(); // 128 - 35 - (6 * 2) = 81, so start at trunc(81 / 2) = 40.
  printCharacter(40, 10, 16); // W = 16.
  printCharacter(47, 10, 17); // A = 17.
  printCharacter(54, 10, 18); // L = 18.
  printCharacter(61, 10, 10); // K = 10.
  printCharacter(68, 10, 19); // I = 19.
  printCharacter(75, 10, 20); // N = 20.
  printCharacter(82, 10, 21); // G = 21.
}  

void printRunning(){ // 7 letters = width of 35, with spacing of 2 between each letter.
  clearDisplay(); // 128 - 35 - (6 * 2) = 81, so start at trunc(81 / 2) = 40.
  printCharacter(40, 10, 14); // R = 14.
  printCharacter(47, 10, 22); // U = 22.
  printCharacter(54, 10, 20); // N = 20.
  printCharacter(61, 10, 20); // N = 20.
  printCharacter(68, 10, 19); // I = 19.
  printCharacter(75, 10, 20); // N = 20.
  printCharacter(82, 10, 21); // G = 21.
}

void printStill(){ // 5 letters = width of 25, with spacing of 2 between each letter.
  clearDisplay(); // 128 - 25 - (4 * 2) = 95, so start at trunc(95 / 2) = 47.
  printCharacter(47, 10, 23); // S = 23.
  printCharacter(54, 10, 24); // T = 24.
  printCharacter(61, 10, 19); // I = 19.
  printCharacter(68, 10, 18); // L = 18.
  printCharacter(75, 10, 18); // L = 18.
}

void printModuleCode(){
	clearDisplay();

	printCharacter(2, 2, 4);  // 4
	printCharacter(9, 2, 25); // B
	printCharacter(16, 2, 2); // 2
	printCharacter(23, 2, 5); // 5
}
