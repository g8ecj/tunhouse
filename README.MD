Tunnel House Window Controller

Three 1-wire sensors read the temperature at the bottom, the top and outside the tunnel house. 

The bottom and top sensors control motors to open/close vents.
The temperature at which the vents are opened and closed is set from the UI and are independent.

Date and time are set from the UI and displayed on the main page.
The CPU clock variation can be accounted for by setting the 'Timesync' value to allow for fast or slow clocks.

The supply voltage (I run from a car battery with a 5W solar panel on it) can also be monitored.

When a key is pressed, the LCD backlight it turned on for a duration that can also be set in the UI.

An image of the LCD is sent to a remote station via the NRF2401. This consists of 4 lines of 20 characters each along with backlight and cursor information.
Keypress data can also be sent fro mthe remote startion back to the controller for full remote operation.

The code uses 3 state machines:
   1. the opening and closing of the vents, handling timouts, manual override etc
   2. the measuring of the temperatures, keeping track of which sensors are busy
   3. the UI which has 4 modes
      a. Monitor
         several screens that allow details to be displayed such as current vent position, temperature, battery voltage
      b. Setup
         top level configuration with screens for setting temperature limits, time
      c. Page Edit
         Once within a setup screen, allows for the movement between fields
      d. Field Edit
         Having selected a field to change, allows values to be adjusted acording to the type of field it is (decimal, boolean etc) with wraparound.


Build instructions

clone my BeRTOS git repository and switch to the 'g8ecj/all' branch that merges my 1-wire and terminal emulator branches.
Use the BeRTOS wizard in the resulting branch to create a project.
wizard/bertos.py
Hit <next>
Set project name to tunhouse
Hit <next>
Select BeRTOS version
Hit <next>
Select 'Arduino Duemilanove'
Hit <next>
Select toolchain
Hit <next>
Make sure the 'Automatically fix dependencies' is checked
Select the following modules

drv/kbd
drv/lcd_pcf8574
drv/ow_ds18x20
drv/ser
drv/spi_bitbang
drv/term
net/nrf24l01

Hit <next>
Hit <create>
Hit <finish>

cd into this tunhouse directory of the new project, delete the tunhouse sub-directory already in it and clone this repo into it using

git clone https://github.com/g8ecj/tunhouse.git tunhouse

A quick patch may now have to be applied. This is to remove the eeprom data from the hex file output, otherwise avrdude chokes on it.
That is done by changing at around line 320 in bertos/rules.mk
from
	$$($(1)_OBJCOPY) -O ihex $$< $$@
to
	$$($(1)_OBJCOPY) -R .eeprom -O ihex $$< $$@

Move up one level and 'make' will generate the 'tunhouse.hex' firmware in the 'images' directory.


Building the remote

Follow the instructions above using the project name 'remote', no need for the 'drv'ow_ds18x20' module.
When the wizard has completed, copy remote.c from the tunhouse directory into the new remote/remote directory and either
   1. rename it to main.c or
   2. edit the remote_user.mk and change main.c to remote.c
Again, move up one level and 'make' will generate the 'remote.hex' firmware in the 'images' directory.


