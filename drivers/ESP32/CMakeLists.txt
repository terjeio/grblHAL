# The following four lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# The following plugins are supported:
# I2C keypad
# SD Card
# Trinamic TMC2130 stepper drivers via I2C <> SPI bridge
# ---
# Move the commented lines below to MAIN_SRCS list
# if enabled in driver.h:
# keypad/keypad.c
# trinamic/trinamic2130.c
# trinamic/TMC2130_I2C_map.c
# tmc2130/trinamic.c
# sdcard/sdcard.c

set(MAIN_SRCS main.c driver.c nvs.c esp32-hal-uart.c bluetooth.c wifi.c TCPStream.c
 i2c.c
 ioexpand.c
 eeprom.c
 GRBL/grbllib.c
 GRBL/coolant_control.c
 GRBL/eeprom_emulate.c
 GRBL/gcode.c
 GRBL/limits.c
 GRBL/motion_control.c
 GRBL/nuts_bolts.c
 GRBL/override.c
 GRBL/planner.c
 GRBL/protocol.c
 GRBL/report.c
 GRBL/settings.c
 GRBL/sleep.c
 GRBL/spindle_control.c
 GRBL/state_machine.c
 GRBL/stepper.c
 GRBL/system.c
)

#include_directories("$ENV{IDF_PATH}/components/freertos/include" "D:\Projects\esp\GrblDriver")
#include_directories(BEFORE "D:/Projects/esp/GrblDriver")

set(INCLUDE_DIRS "D:/Projects/esp/GrblDriver")

#file(GLOB GRBL_SOURCE "GRBL/*.c")
    
#list (APPEND GRBL_SOURCE MAIN_SRCS)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(grbl)
include_directories(BEFORE "D:/Projects/esp/GrblDriver")
#target_compile_options(grbl PRIVATE -Werror -Wall -Wextra)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#add_subdirectory (GRBL)
