#
# Wizard autogenerated makefile.
# DO NOT EDIT, use the tunhouse_user.mk file instead.
#

# Constants automatically defined by the selected modules
tunhouse_DEBUG = 0

# Our target application
TRG += tunhouse

tunhouse_PREFIX = "/usr/bin/avr-"

tunhouse_SUFFIX = ""

tunhouse_SRC_PATH = tunhouse

tunhouse_HW_PATH = tunhouse

# Files automatically generated by the wizard. DO NOT EDIT, USE tunhouse_USER_CSRC INSTEAD!
tunhouse_WIZARD_CSRC = \
	bertos/algo/crc8.c \
	bertos/cpu/avr/drv/ser_mega.c \
	bertos/cpu/avr/drv/i2c_mega.c \
	bertos/cpu/avr/drv/timer_mega.c \
	bertos/drv/pcf8574.c \
	bertos/drv/lcd_hd44780.c \
	bertos/drv/ow_1wire.c \
	bertos/drv/ow_ds18x20.c \
	bertos/drv/ow_ds2413.c \
	bertos/drv/ser.c \
	bertos/drv/i2c.c \
	bertos/drv/kbd.c \
	bertos/drv/term.c \
	bertos/drv/timer.c \
	bertos/drv/spi_bitbang.c \
	bertos/net/nrf24l01.c \
	bertos/io/kfile.c \
	bertos/mware/event.c \
	bertos/mware/formatwr.c \
	bertos/mware/hex.c \
	bertos/struct/heap.c \
	#

# Files automatically generated by the wizard. DO NOT EDIT, USE tunhouse_USER_PCSRC INSTEAD!
tunhouse_WIZARD_PCSRC = \
	bertos/mware/formatwr.c \
	#

# Files automatically generated by the wizard. DO NOT EDIT, USE tunhouse_USER_CPPASRC INSTEAD!
tunhouse_WIZARD_CPPASRC = \
	 \
	#

# Files automatically generated by the wizard. DO NOT EDIT, USE tunhouse_USER_CXXSRC INSTEAD!
tunhouse_WIZARD_CXXSRC = \
	 \
	#

# Files automatically generated by the wizard. DO NOT EDIT, USE tunhouse_USER_ASRC INSTEAD!
tunhouse_WIZARD_ASRC = \
	 \
	#

tunhouse_CPPFLAGS = -D'CPU_FREQ=(16000000UL)' -D'ARCH=(ARCH_DEFAULT)' -D'WIZ_AUTOGEN' -I$(tunhouse_HW_PATH) -I$(tunhouse_SRC_PATH) $(tunhouse_CPU_CPPFLAGS) $(tunhouse_USER_CPPFLAGS)

# Automatically generated by the wizard. PLEASE DO NOT EDIT!
tunhouse_LDFLAGS = $(tunhouse_CPU_LDFLAGS) $(tunhouse_WIZARD_LDFLAGS) $(tunhouse_USER_LDFLAGS)

# Automatically generated by the wizard. PLEASE DO NOT EDIT!
tunhouse_CPPAFLAGS = $(tunhouse_CPU_CPPAFLAGS) $(tunhouse_WIZARD_CPPAFLAGS) $(tunhouse_USER_CPPAFLAGS)

# Automatically generated by the wizard. PLEASE DO NOT EDIT!
tunhouse_CSRC = $(tunhouse_CPU_CSRC) $(tunhouse_WIZARD_CSRC) $(tunhouse_USER_CSRC)

# Automatically generated by the wizard. PLEASE DO NOT EDIT!
tunhouse_PCSRC = $(tunhouse_CPU_PCSRC) $(tunhouse_WIZARD_PCSRC) $(tunhouse_USER_PCSRC)

# Automatically generated by the wizard. PLEASE DO NOT EDIT!
tunhouse_CPPASRC = $(tunhouse_CPU_CPPASRC) $(tunhouse_WIZARD_CPPASRC) $(tunhouse_USER_CPPASRC)

# Automatically generated by the wizard. PLEASE DO NOT EDIT!
tunhouse_CXXSRC = $(tunhouse_CPU_CXXSRC) $(tunhouse_WIZARD_CXXSRC) $(tunhouse_USER_CXXSRC)

# Automatically generated by the wizard. PLEASE DO NOT EDIT!
tunhouse_ASRC = $(tunhouse_CPU_ASRC) $(tunhouse_WIZARD_ASRC) $(tunhouse_USER_ASRC)

# CPU specific flags and options, defined in the CPU definition files.
# Automatically generated by the wizard. PLEASE DO NOT EDIT!
tunhouse_MCU = atmega328p
tunhouse_CPU_CPPFLAGS = -Os -Ibertos/cpu/avr/
tunhouse_PROGRAMMER_CPU = atmega328p
tunhouse_STOPFLASH_SCRIPT = bertos/prg_scripts/avr/stopflash.sh
tunhouse_STOPDEBUG_SCRIPT = bertos/prg_scripts/none.sh
tunhouse_DEBUG_SCRIPT = bertos/prg_scripts/nodebug.sh
tunhouse_FLASH_SCRIPT = bertos/prg_scripts/avr/flash.sh

include $(tunhouse_SRC_PATH)/tunhouse_user.mk
