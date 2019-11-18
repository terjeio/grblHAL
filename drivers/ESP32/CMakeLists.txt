# CMake for GrblHAL driver for ESP32
# 2019-11-18 

cmake_minimum_required(VERSION 3.5)

# The following plugins are supported:
# I2C keypad
# SD Card
# Trinamic TMC2130 stepper drivers via I2C <> SPI bridge
# Networking
# ---
# Change the corresponding OPTION setting below
# to ON if plugin is enabled in driver.h:

OPTION(Networking "Wifi + protocols" OFF)
OPTION(Keypad "I2C Keypad" OFF)
OPTION(SDcard "SD Card Streaming" OFF)
OPTION(Trinamic "Trinamic driver support over I2C" OFF)

set(SDCARD_SOURCE sdcard/sdcard.c)
set(KEYPAD_SOURCE keypad/keypad.c)
set(TRINAMIC_SOURCE trinamic/trinamic2130.c trinamic/TMC2130_I2C_map.c tmc2130/trinamic.c)
set(NETWORKING_SOURCE wifi.c dns_server.c web/backend.c networking/TCPStream.c networking/WsStream.c networking/base64.c networking/sha1.c )

set(MAIN_SRCS main.c driver.c nvs.c esp32-hal-uart.c bluetooth.c 
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

if(Networking)
list (APPEND MAIN_SRCS ${NETWORKING_SOURCE})
endif()

if(SDcard)
list (APPEND MAIN_SRCS ${SDCARD_SOURCE})
endif()

if(Keypad)
list (APPEND MAIN_SRCS ${KEYPAD_SOURCE})
endif()

if(Trinamic) 
list (APPEND MAIN_SRCS ${TRINAMIC_SOURCE})
endif()

unset(Networking CACHE)
unset(Keypad CACHE)
unset(SDcard CACHE)
unset(Trinamic CACHE)

set(INCLUDE_DIRS ".")

#file(GLOB GRBL_SOURCE "GRBL/*.c")
#list (APPEND GRBL_SOURCE MAIN_SRCS)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(grbl)

target_add_binary_data(grbl.elf "favicon.ico" BINARY)
target_add_binary_data(grbl.elf "index.html" BINARY)
target_add_binary_data(grbl.elf "ap_login.html" BINARY)

include_directories(BEFORE ".")
#target_compile_options(grbl PRIVATE -Werror -Wall -Wextra)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#add_subdirectory (GRBL)
