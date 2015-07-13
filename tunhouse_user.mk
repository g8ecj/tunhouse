#
# User makefile.
# Edit this file to change compiler options and related stuff.
#
# trim the commit hash from the version but keep the subversion beyond the tag
#GIT_VERSION := $(shell cd $(ardmega-turbine_SRC_PATH); git describe --dirty --always | sed 's/-g.*//'; cd ..)
GIT_VERSION := 1.0

# Programmer interface configuration, see http://dev.bertos.org/wiki/ProgrammerInterface for help
tunhouse_PROGRAMMER_TYPE = none
tunhouse_PROGRAMMER_PORT = none

# Files included by the user.
tunhouse_USER_CSRC = \
	$(tunhouse_SRC_PATH)/main.c \
	$(tunhouse_SRC_PATH)/minmax.c \
	$(tunhouse_SRC_PATH)/rtc.c \
	$(tunhouse_SRC_PATH)/eeprommap.c \
	$(tunhouse_SRC_PATH)/measure.c \
	$(tunhouse_SRC_PATH)/analog.c \
	$(tunhouse_SRC_PATH)/window.c \
	$(tunhouse_SRC_PATH)/ui.c \
	#

# Files included by the user.
tunhouse_USER_PCSRC = \
	#

# Files included by the user.
tunhouse_USER_CPPASRC = \
	#

# Files included by the user.
tunhouse_USER_CXXSRC = \
	#

# Files included by the user.
tunhouse_USER_ASRC = \
	#

# Flags included by the user.
tunhouse_USER_LDFLAGS = \
	#

# Flags included by the user.
tunhouse_USER_CPPAFLAGS = \
	#

# Flags included by the user.
tunhouse_USER_CPPFLAGS = \
	-fno-strict-aliasing \
	-fwrapv \
	-DVERSION=\"$(GIT_VERSION)\" \
	#
