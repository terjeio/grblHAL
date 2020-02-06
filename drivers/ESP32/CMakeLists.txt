# CMake for GrblHAL driver for ESP32
# 2020-01-07

cmake_minimum_required(VERSION 3.5)

# The following plugins/options are supported:

OPTION(Networking "Wifi + protocols" OFF)
OPTION(Keypad "I2C Keypad" OFF)
OPTION(SDcard "SD Card Streaming" OFF)
OPTION(Trinamic "Trinamic driver support over I2C" OFF)
OPTION(WebUI "WebUI services" OFF)
OPTION(WebAuth "WebUI authentication" OFF)
OPTION(MPGMode "MPG mode" OFF)
OPTION(BoosterPack "Compile for CNC BoosterPack" OFF)

set(SDCARD_SOURCE sdcard/sdcard.c)
set(KEYPAD_SOURCE keypad/keypad.c)
set(TRINAMIC_SOURCE trinamic/trinamic2130.c trinamic/TMC2130_I2C_map.c tmc2130/trinamic.c)
set(NETWORKING_SOURCE wifi.c dns_server.c web/backend.c web/upload.c networking/TCPStream.c networking/WsStream.c networking/base64.c networking/sha1.c networking/urldecode.c networking/strutils.c networking/utils.c networking/multipartparser.c )
set(WEBUI_SOURCE webui/server.c webui/response.c webui/commands.c webui/flashfs.c )

set(MAIN_SRCS main.c driver.c nvs.c esp32-hal-uart.c bluetooth.c
 i2c.c
 ioexpand.c
 eeprom.c
 grbl/grbllib.c
 grbl/coolant_control.c
 grbl/eeprom_emulate.c
 grbl/gcode.c
 grbl/limits.c
 grbl/motion_control.c
 grbl/nuts_bolts.c
 grbl/override.c
 grbl/planner.c
 grbl/protocol.c
 grbl/report.c
 grbl/settings.c
 grbl/sleep.c
 grbl/spindle_control.c
 grbl/state_machine.c
 grbl/stepper.c
 grbl/system.c
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

if(WebUI) 
list (APPEND MAIN_SRCS ${WEBUI_SOURCE})
endif()

set(INCLUDE_DIRS ".")

#file(GLOB GRBL_SOURCE "grbl/*.c")
#list (APPEND GRBL_SOURCE MAIN_SRCS)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(grbl)

if(Networking)
target_compile_definitions(grbl.elf PUBLIC NETWORKING_ENABLE)
endif()

if(SDcard)
target_compile_definitions(grbl.elf PUBLIC SDCARD_ENABLE)
endif()

if(WebUI)
target_compile_definitions(grbl.elf PUBLIC WEBUI_ENABLE)
target_add_binary_data(grbl.elf "index.html.gz" BINARY)
if(WebUI)
target_compile_definitions(grbl.elf PUBLIC AUTH_ENABLE)
endif()
endif()

if(Trinamic) 
target_compile_definitions(grbl.elf PUBLIC TRINAMIC_ENABLE)
endif()

if(Keypad)
target_compile_definitions(grbl.elf PUBLIC KEYPAD_ENABLE)
endif()

if(MPGMode)
target_compile_definitions(grbl.elf PUBLIC MPG_MODE_ENABLE)
endif()

if(BoosterPack)
target_compile_definitions(grbl.elf PUBLIC CNC_BOOSTERPACK)
endif()

target_add_binary_data(grbl.elf "favicon.ico" BINARY)
target_add_binary_data(grbl.elf "index.html" BINARY)
target_add_binary_data(grbl.elf "ap_login.html" BINARY)

unset(Networking CACHE)
unset(Keypad CACHE)
unset(SDcard CACHE)
unset(Trinamic CACHE)
unset(WebUI CACHE)
unset(WebAuth CACHE)
unset(MPGMode CACHE)
unset(BoosterPack CACHE)

include_directories(BEFORE ".")

#target_compile_options(grbl.elf PRIVATE -Werror -Wall -Wextra)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#add_subdirectory (grbl)
