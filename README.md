# Overview 
A digital thermometer and temperature logger that runs on the Raspberry Pi Pico W microcontroller. Can be configured to capture readings of the temperature and humidity at a steady interval, and can produce graphs of the collected data against time.

The I2C commands are currently adapted for the SHT31 digital thermometer, but these can easily be swapped for different devices.
The SPI commands are likewise adapted for a particular 96x64 RGB565 display, and adapting the graphics for a different size display would require much larger overhauls. 

Note: There are some work-in-progress wireless connectivity options and functionality present in the code and in the product, but these are not fully functional and are not intended to be used yet.

# Features
Temperature & humidity graphs mapped against time.
	- Graph can be zoomed arbitrarily
	- Can view precise readings at a given time
	- Data samples' timestamps adapt to system time changes by computing the relative time on the fly
Live configuration settings
	- System time
	- Display brightness
	- Display sleep time
	- Adjustable polling intervals
Graphic memory inspector
	- Visualizes the total system RAM through the display
	- Values of individual bytes can be read for debugging

# Architecture

`Temp_logger.c` contains the main loop and program.
`poll_log.c` and `poll_log.h` contain the definitions for simple linked list used by the temperature log.
`font.h` contain the definitions for the pixel font and other hardcoded graphics.

The main program runs inside of a continuously running loop that:
	- Checks for user input, and stores any in the device state
	- Polls the thermometer for the current temperature (only if the interval has passed)
	- Updates device state variables based on user input
	- Renders the screen based on the changes, if any
