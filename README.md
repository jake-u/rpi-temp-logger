# Overview 

![example physical implementation](https://github.com/jake-u/rpi-temp-logger/blob/master/readme_assets/exinterface.png?raw=true)
A temperature and humidity logger that runs on the Raspberry Pi Pico W microcontroller. Can be configured to capture readings of the temperature and humidity at a steady interval, and can produce graphs of the collected data against time.
![example temperature graph](https://github.com/jake-u/rpi-temp-logger/blob/master/readme_assets/extemp2.png?raw=true)
![example humidity graph](https://github.com/jake-u/rpi-temp-logger/blob/master/readme_assets/exhum.png?raw=true)

The I2C commands are currently adapted for the SHT31 digital thermometer, but these can easily be swapped for different devices.
The SPI commands are likewise adapted for a particular 96x64 RGB565 display, and adapting the graphics for a different size display would require much larger overhauls. 

Note: There are some work-in-progress wireless connectivity options and functionality present in the code and in the product, but these are not fully functional and are not intended to be used yet.

# Features
![example temperature graph](https://github.com/jake-u/rpi-temp-logger/blob/master/readme_assets/extemp1.png?raw=true)
- Temperature & humidity graphs mapped against time; can be zoomed arbitrarily.
	- Can view precise readings at a given time
	- Data samples' timestamps adapt to system time changes by computing the relative time on the fly
![configuration screen](https://github.com/jake-u/rpi-temp-logger/blob/master/readme_assets/exconf.png?raw=true)
- Live configuration settings
	- System time
	- Display brightness
	- Display sleep time
	- Adjustable polling intervals
![memory inspector](https://github.com/jake-u/rpi-temp-logger/blob/master/readme_assets/exmeminspec.png?raw=true)
- Graphic memory inspector
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
