#  Part of Grbl Simulator
#
#  Copyright (c) 2012 Jens Geisler
#  Copyright (c) 2014-2015 Adam Shelly
#
#  2020 - modified for grblHAL by Terje Io
#
#  Grbl is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  Grbl is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

# PLATFORM   = WINDOWS
# PLATFORM   = OSX
PLATFORM   = LINUX

#The original grbl code, except those files overriden by sim
GRBL_BASE_OBJECTS = grbl/grbllib.o grbl/protocol.o grbl/planner.o grbl/settings.o grbl/nuts_bolts.o grbl/stepper.o grbl/gcode.o grbl/spindle_control.o grbl/motion_control.o grbl/limits.o grbl/coolant_control.o grbl/system.o grbl/report.o grbl/state_machine.o grbl/override.o grbl/nvs_buffer.o grbl/sleep.o grbl/tool_change.o grbl/my_plugin.o

# Simulator Only Objects
SIM_OBJECTS = main.o simulator.o driver.o eeprom.o grbl_eeprom_extensions.o mcu.o serial.o platform_$(PLATFORM).o

GRBL_SIM_OBJECTS = grbl_interface.o  $(GRBL_BASE_OBJECTS) $(SIM_OBJECTS)
GRBL_VAL_OBJECTS = validator.o validator_driver.o $(GRBL_BASE_OBJECTS)

CLOCK      = 16000000
SIM_EXE_NAME   = grbl_sim.exe
VALIDATOR_NAME = gvalidate.exe
FLAGS = -g -O3
COMPILE    = $(CC) -Wall $(FLAGS) -DF_CPU=$(CLOCK) -I. -DPLAT_$(PLATFORM)
LINUX_LIBRARIES = -lrt -pthread
OSX_LIBRARIES =
WINDOWS_LIBRARIES =

# symbolic targets:
all:	main gvalidate

new: clean main gvalidate

clean:
	rm -f $(SIM_EXE_NAME) $(GRBL_SIM_OBJECTS) $(VALIDATOR_NAME) $(GRBL_VAL_OBJECTS)

# file targets:
main: $(GRBL_SIM_OBJECTS) 
	$(COMPILE) -o $(SIM_EXE_NAME) $(GRBL_SIM_OBJECTS) -lm $($(PLATFORM)_LIBRARIES)


gvalidate: $(GRBL_VAL_OBJECTS) 
	$(COMPILE)  -o $(VALIDATOR_NAME) $(GRBL_VAL_OBJECTS) -lm  $($(PLATFORM)_LIBRARIES)


%.o: %.c
	$(COMPILE) -c $< -o $@

grbl/planner.o: grbl/planner.c
	$(COMPILE) -include planner_inject_accessors.c -c $< -o $@
