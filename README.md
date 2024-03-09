# 4B25 Coursework 4 - Accelerometer-Based Activity Classifier with Uncertainty Estimates

**Name: Leon Brindley**

**College: Gonville and Caius**

**CRSid: lpb32**

<ins>**Summary**</ins>

This project determines the number of steps taken by an individual over a **10-second** period. It then infers whether they are **stationary**, **walking** or **running** from this. The classification is determined using a five-step algorithm and a **Freescale MMA8451Q** accelerometer. The design was implemented using a **Freescale FRDM-KL03Z** development platform, which contains an integrated MMA8451Q accelerometer by default.

<ins>**Step 1: Magnitude Calculation**</ins>

Firstly, the **14-bit** acceleration measurements from the X, Y and Z axes are parsed using bit shift operations. The magnitude of these three readings is calculated using Pythagoras' theorem across three cartesian axes. The **sqrt()** function in **math.h** is too large relative to the FRDM-KL03Z's **2 KB** of SRAM, so an integer-based implementation using the Newton-Raphson method is executed in **sqrtInt()** instead. By default, **19** results are stored in the **AccelerationBuffer**.

<ins>**Step 2: Low-Pass Filter**</ins>

Secondly, the AccelerationBuffer is multiplied by an array of coefficients (**LPFBuffer**) to low-pass filter the signal. This removes high-frequency noise, so an accurate speed can be calculated. By default, **19** results are stored in the LPFBuffer.

<ins>**Step 3: Frequency Calculation**</ins>

Thirdly, the frequency of the signal is extracted by counting the number of **inflection points** (**maxima** and **minima**). The function **simpleDiff()** compares the readings on either side of each array element to identify these inflection points. The **numberOfInflectionPoints** is divided by **2** to give the **numberOfSteps**, as both the maxima and minima are included in the calculation.

<ins>**Step 4: Step Counting and Speed Calculation**</ins>

Fourthly, the number of steps in a particular period of time is extracted from the data. By accounting for the length of each step, the algorithm also estimates the speed at which the device is moving. Please note that the speed results will only become valid once the AccelerationBuffer and LPFBuffer have been filled.

<ins>**Step 5: Activity Classification**</ins>

Finally, the aforementioned speed calculation is converted to an activity. [Long and Srinivasan](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC3627106) have shown that the average speed with equal amounts of walking and running (running fraction = 0.5) is about 2.2 m/s. Therefore, the **activityReading** variable is set to **ActivityRunning** if the speed exceeds 2.2 m/s (7.92 km/hr). Meanwhile, [Graham et al.](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC2967707) have shown that mean walking speeds of 0.23 m/s have been reported for older adults in geriatric rehabilitation settings. Therefore, the **activityReading** variable is set to **ActivityWalking** if the speed is between 0.23 m/s (0.828 km/hr) and 2.2 m/s (7.92 km/hr). Similarly, if the speed is below 0.23 m/s (0.828 km/hr), the **activityReading** variable is set to **ActivityStationary**.

<ins>**Configuration: MMA8451Q Registers**</ins>

The MMA8451Q accelerometer is configured by writing to the registers **F_SETUP**, **CTRL_REG1**, **HP_FILTER_CUTOFF** and **XYZ_DATA_CFG**. The structures of each register are explained in [Freescale's MMA8451Q datasheet](https://pdf1.alldatasheet.com/datasheet-pdf/download/460022/FREESCALE/MMA8451Q.html).

Firstly, F_SETUP is set to **0x00** to disable the internal first-in, first-out (FIFO) buffer.

Secondly, CTRL_REG1 is set to **0x05** to select a **14-bit** resolution and activate the **reduced noise mode**. In contrast, if the **F_READ** bit is set to **1**, then an **8-bit** resolution can be used for faster data transfer.

Thirdly, HP_FILTER_CUTOFF is set to **0x00** to enable the MMA8451Q's high-pass filter with its default cut-off frequency of **16 Hz** and its default output data rate (ODR) of **800 Hz**. This high-pass filter removes the acceleration due to **gravity** (g), which forms a DC offset.

Finally, XYZ_DATA_CFG is set to **0x11** so the high-pass filter is not bypassed and the accelerometer's full-scale range equals **4g**.

<ins>**Configuration: Low-Pass Filter Cut-Off Frequency**</ins>

To set the low-pass filter frequency, you can use a program such as [WinRFCalc](https://rfcalculator.com). You must make sure that the number of taps is **odd** and that the sampling frequency is **at least twice** the cut-off frequency (to fulfil the Nyquist Criterion). The default LPFWeights array gives **19** taps, a maximum attenuation of **120dB** and a cut-off frequency and sampling frequency of **450 Hz** and **8,192 Hz**, respectively. These parameters were chosen as the resulting coefficients were all positive and there were minimal rounding errors (such as in comparison to an attenuation of 130dB).

![WinRFCalc19](https://github.com/LeonBrindley/Warp-Coursework-2/assets/68070085/f3751657-6c32-4f95-8307-0c1e51b3f1d3)

In contrast, the alternative LPFWeights array gives **39** taps, a maximum attenuation of **130dB** and a cut-off frequency and sampling frequency of **450 Hz** and **4,096 Hz**, respectively.

![WinRFCalc](https://github.com/LeonBrindley/Warp-Coursework-2/assets/68070085/a63f13e9-0b34-45bc-ab6a-496e3c0f1f94)

<ins>**Low-Pass Filter Effect**</ins>

The LPF significantly smoothes the acceleration magnitudes provided.

<img width="1145" alt="MMA8451Q Acceleration After LPF" src="https://github.com/LeonBrindley/Warp-Coursework-2/assets/68070085/40857ab5-3bd0-4c23-9f49-77abd432da8e">

<ins>**OLED Display**</ins>

The function **printCharacter()** in **activityClassifier.c** is used to display the numbers and units of the measurements. This calls the function **printLine()** in **devSSD1331.c** to display the numbers **0 to 9**, the letters **K**, **M**, **H**, **R**, **W**, **A**, **L**, **I**, **N**, **G**, **U**, **S** and **T**, a full stop (**.**) or a forward slash (**/**). This allows for the classifications **RUNNING**, **WALKING** and **STILL** to be printed, as well as the unit **KM/HR**. Alternatively, for bolder text with a width of greater than one pixel, the function **printRect()** in **devSSD1331.c** can be used instead.

<ins>**File Structure**</ins>

`src/boot/ksdk1.1.0/activityClassifier.c` - Implements the four steps of the activity classifier algorithm. This source file also includes the function **printCharacter()** so the final results can be printed on an SSD1331 OLED display. These numbers are implemented by calling the function **printLine()** to mimic multiple seven-segment displays alongside each other.

`src/boot/ksdk1.1.0/devMMA8451Q.c` - Driver for communicating with MMA8451Q accelerometers. For example, the function **configureSensorMMA8451Q()** will write to the MMA8451Q configuration registers explained above. The I2C address of the MMA8451Q must be set to **0x1D** when calling initMMA8451Q in boot.c with the FRDM-KL03Z. When using the FRDM-KL03Z, as opposed to the more complicated Warp platform, the MMA8451Q's operating voltage cannot be dynamically altered, so the **operatingVoltageMillivolts** member variable has no effect.

`src/boot/ksdk1.1.0/devSSD1331.c` - Driver for communicating with SSD1331 OLED displays. For example, the function **printLine()** will print a 2D line between two coordinates in a colour defined by its blue, green and red content. Meanwhile, the function **printRect()** will print a solid rectangle bounded by two coordinates in a colour defined by its fill and its line (border).

`src/boot/ksdk1.1.0/boot.c` - Contains the **main()** function that runs when the program boots. For example, the function **classifierAlgorithm()** is repeatedly called here using a for loop to execute the activity classifier algorithm.

`src/boot/ksdk1.1.0/config.h` - Configures the Warp firmware for the platform in question (in this case, the FRDM-KL03Z). For the activity classifier algorithm to work, **WARP_BUILD_ENABLE_DEVMMA8451Q** must be set to **1**. Furthermore, as two bytes are required for each axis (X, Y and Z), **kWarpSizesI2cBufferBytes** must be set to **at least 6**.

# Baseline firmware for the [Warp](https://github.com/physical-computation/Warp-hardware) family of hardware platforms
This is the firmware for the [Warp hardware](https://github.com/physical-computation/Warp-hardware) and its publicly available and unpublished derivatives. This firmware also runs on the Freescale/NXP FRDM KL03 evaluation board which we use for teaching at the University of Cambridge. When running on platforms other than Warp, only the sensors available in the corresponding hardware platform are accessible.

**Prerequisites:** You need an arm cross-compiler such as `arm-none-eabi-gcc` installed as well as a working `cmake` (installed, e.g., via `apt-get` on Linux or via [MacPorts](https://www.macports.org) on macOS). On Ubuntu, the package you need is `gcc-arm-none-eabi`. You will also need an installed copy of the SEGGER [JLink commander](https://www.segger.com/downloads/jlink/), `JlinkExe`, which is available for Linux, macOS, and Windows (here are direct links for downloading it for [macOS](https://www.segger.com/downloads/jlink/JLink_MacOSX.pkg), and [Linux tgz 64-bit](https://www.segger.com/downloads/jlink/JLink_Linux_x86_64.tgz)).

## 1.  Compiling the Warp firmware
First, edit [setup.conf](setup.conf) to set the variable `ARMGCC_DIR` and `JLINKPATH`. If your `arm-none-eabi-gcc` is in `/usr/local/bin/arm-none-eabi-gcc`, then you want to set  `ARMGCC_DIR` to `/usr/local`. In the following, this `README.md` will refer to the top of the repository as `$TREEROOT`. On some platforms, you might need to also, in addition, set the `ARMGCC_DIR` environment variable in your shell (using `setenv` or `export` as appropriate), to point to the same path as you set in [setup.conf](setup.conf).

Second, edit [`tools/scripts/glaux.jlink.commands`](tools/scripts/glaux.jlink.commands) and [`tools/scripts/warp.jlink.commands`](tools/scripts/warp.jlink.commands) to replace `<full-path-to-warp-firmware>` with the full path to your Warp firmware directory.

Third, build the Warp firmware by

	make warp

Fourth, load the Warp firmware to hardware by

	make load-warp

To build for the Glaux variant, use `make glaux` and `make load-glaux` in steps three and four instead.

The build process copies files from `src/boot/ksdk1.1.0/` into the `build/`, builds, and converts the binary to SREC. See `Warp/src/boot/ksdk1.1.0/README.md` for more. _When editing source, edit the files in `src/boot/ksdk1.1.0/`, not the files in `build` location, since the latter are overwritten during each build._

To connect to the running hardware to see output, you will need two terminal windows. In a separate shell window from the one in which you ran `make load-warp` (or its variants), launch the JLink RTT client<sup>&nbsp;<a href="#Notes">See note 1 below</a></sup>:

	JLinkRTTClient

## 2. Using the Warp firmware on the Freescale FRDMKL03 Board
The SEGGER firmware allows you to use SEGGER’s JLink software to load your own firmware to the board, even without using their specialized JLink programming cables. You can find the SEGGER firmware at the SEGGER Page for [OpenSDA firmware](https://www.segger.com/products/debug-probes/j-link/models/other-j-links/opensda-sda-v2/).

To build the Warp firmware for the FRDM KL03, you will need to modify [this line in `src/boot/ksdk1.1.0/config.h`](https://github.com/physical-computation/Warp-firmware/blob/9e7f9e5e3f3c039cc98cbd1e6dfeb6b8fd78c86a/src/boot/ksdk1.1.0/config.h#L55).


## 3.  Editing the firmware
The firmware is currently all in `src/boot/ksdk1.1.0/`, in particular, see `src/boot/ksdk1.1.0/warp-kl03-ksdk1.1-boot.c` and the per-sensor drivers in `src/boot/ksdk1.1.0/dev*.[c,h]`.

The firmware builds on the Kinetis SDK. You can find more documentation on the Kinetis SDK in the document [doc/Kinetis SDK v.1.1 API Reference Manual.pdf](https://github.com/physical-computation/Warp-firmware/blob/master/doc/Kinetis%20SDK%20v.1.1%20API%20Reference%20Manual.pdf).

The firmware is designed for the Warp and Glaux hardware platforms, but will also run on the Freescale FRDM KL03 development board. In that case, the only sensor driver which is relevant is the one for the MMA8451Q. For more details about the structure of the firmware, see [src/boot/ksdk1.1.0/README.md](src/boot/ksdk1.1.0/README.md).

## 4.  Interacting with the boot menu
When the firmware boots, you will be dropped into a menu with a rich set of commands. The Warp boot menu allows you to conduct most of the experiments you will likely need without modifying the firmware:
````
[ *				W	a	r	p	(rev. b)			* ]
[  				      Cambridge / Physcomplab   				  ]

	Supply=0mV,	Default Target Read Register=0x00
	I2C=200kb/s,	SPI=200kb/s,	UART=1kb/s,	I2C Pull-Up=32768

	SIM->SCGC6=0x20000001		RTC->SR=0x10		RTC->TSR=0x5687132B
	MCG_C1=0x42			MCG_C2=0x00		MCG_S=0x06
	MCG_SC=0x00			MCG_MC=0x00		OSC_CR=0x00
	SMC_PMPROT=0x22			SMC_PMCTRL=0x40		SCB->SCR=0x00
	PMC_REGSC=0x00			SIM_SCGC4=0xF0000030	RTC->TPR=0xEE9

	0s in RTC Handler to-date,	0 Pmgr Errors
Select:
- 'a': set default sensor.
- 'b': set I2C baud rate.
- 'c': set SPI baud rate.
- 'd': set UART baud rate.
- 'e': set default register address.
- 'f': write byte to sensor.
- 'g': set default SSSUPPLY.
- 'h': powerdown command to all sensors.
- 'i': set pull-up enable value.
- 'j': repeat read reg 0x00 on sensor #3.
- 'k': sleep until reset.
- 'l': send repeated byte on I2C.
- 'm': send repeated byte on SPI.
- 'n': enable SSSUPPLY.
- 'o': disable SSSUPPLY.
- 'p': switch to VLPR mode.
- 'r': switch to RUN mode.
- 's': power up all sensors.
- 't': dump processor state.
- 'u': set I2C address.
- 'x': disable SWD and spin for 10 secs.
- 'z': dump all sensors data.
Enter selection>
````
### Double echo characters
By default on Unix, you will likely see characters you enter shown twice. To avoid this, do the following:
- Make sure you are running `bash` (and not `csh`)
- Execute `stty -echo` at the command line in the terminal window in which you will run the `JLinkRTTClient`.

### Introduction to using the menu
You can probe around the menu to figure out what to do. In brief, you will likely want:

1. Menu item `b` to set the I2C baud rate.

2. Menu item `r` to switch the processor from low-power mode (2MHz) to "run" mode (48MHz).

3. Menu item `g` to set sensor supply voltage.

4. Menu item `n` to turn on the voltage regulators.

5. Menu item `z` to repeatedly read from all the sensors whose drivers are compiled into the build.

*NOTE: In many cases, the menu expects you to type a fixed number of characters (e.g., 0000 or 0009 for zero and nine)<sup>&nbsp;<a href="#Notes">See note 1 below</a></sup>. If using the `JLinkRTTClient`, the menu interface eats your characters as you type them, and you should not hit RETURN after typing in text. On the other hand, if using `telnet` you have to hit return.*

If you see repeated characters, you can set your terminal to not echo typed characters using `stty -echo`.

### Example 1: Dump all registers for a single sensor
-	`b` (set the I2C baud rate to `0300` for 300 kb/s).
-	`g` (set sensor supply voltage to `3000` for 3000mV sensor supply voltage).
-	`n` (turn on the sensor supply regulators).
-	`j` (submenu for initiating a fixed number of repeated reads from a sensor):
````
Enter selection> j

    Auto-increment from base address 0x01? ['0' | '1']> 0
    Chunk reads per address (e.g., '1')> 1
    Chatty? ['0' | '1']> 1
    Inter-operation spin delay in milliseconds (e.g., '0000')> 0000
    Repetitions per address (e.g., '0000')> 0000
    Maximum voltage for adaptive supply (e.g., '0000')> 2500
    Reference byte for comparisons (e.g., '3e')> 00
````

### Example 2: Stream data from all sensors
This will perpetually stream data from the 90+ sensor dimensions at a rate of about 90-tuples per second. Use the following command sequence:
-	`b` (set the I2C baud rate to `0300` for 300 kb/s).
-	`r` (enable 48MHz "run" mode for the processor).
-	`g` (set sensor supply voltage to `3000` for 3000mV sensor supply voltage).
-	`n` (turn on the sensor supply regulators).
-	`z` (start to stream data from all sensors that can run at the chosen voltage and baud rate).

## 5.  To update your fork
From your local clone:

	git remote add upstream https://github.com/physical-computation/Warp-firmware.git
	git fetch upstream
	git pull upstream master

----

### If you use Warp in your research, please cite it as:
Phillip Stanley-Marbell and Martin Rinard. “A Hardware Platform for Efficient Multi-Modal Sensing with Adaptive Approximation”. ArXiv e-prints (2018). arXiv:1804.09241.

**BibTeX:**
```
@ARTICLE{1804.09241,
	author = {Stanley-Marbell, Phillip and Rinard, Martin},
	title = {A Hardware Platform for Efficient Multi-Modal 
	Sensing with Adaptive Approximation},
	journal = {ArXiv e-prints},
	archivePrefix = {arXiv},
	eprint = {1804.09241},
	year = 2018,
}
```
Phillip Stanley-Marbell and Martin Rinard. “Warp: A Hardware Platform for Efficient Multi-Modal Sensing with Adaptive Approximation”. IEEE Micro, Volume 40 , Issue 1 , Jan.-Feb. 2020.

**BibTeX:**
```
@ARTICLE{8959350,
	author = {P. {Stanley-Marbell} and M. {Rinard}},
	title = {Warp: A Hardware Platform for Efficient Multi-Modal
	Sensing with Adaptive Approximation},
	journal = {IEEE Micro},
	year = {2020},
	volume = {40},
	number = {1},
	pages = {57-66},
	ISSN = {1937-4143},
	month = {Jan},
}
```
### Acknowledgements
This research is supported by an Alan Turing Institute award TU/B/000096 under EPSRC grant EP/N510129/1, by Royal Society grant RG170136, and by EPSRC grants EP/P001246/1 and EP/R022534/1.

----
### Notes
<sup>1</sup>&nbsp; On some Unix platforms, the `JLinkRTTClient` has a double echo of characters you type in. You can prevent this by configuring your terminal program to not echo the characters you type. To achieve this on `bash`, use `stty -echo` from the terminal. Alternatively, rather than using the `JLinkRTTClient`, you can use a `telnet` program: `telnet localhost 19021`. This avoids the JLink RTT Client's "double echo" behavior but you will then need a carriage return (&crarr;) for your input to be sent to the board. Also see [Python SEGGER RTT library from Square, Inc.](https://github.com/square/pylink/blob/master/examples/rtt.py) (thanks to [Thomas Garry](https://github.com/tidge27) for the pointer).
