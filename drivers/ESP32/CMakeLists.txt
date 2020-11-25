# CMake for GrblHAL driver for ESP32
# 2020-11-22

cmake_minimum_required(VERSION 3.5)

# Select board to compile for:

# NOTE: Only one board may be enabled!
# If none is enabled pin mappings from generic_map.h will be used.

OPTION(BOARD_BDRING_V3P5 "Compile for v3.5 3-axis board" OFF)
OPTION(BOARD_BDRING_V4 "Compile for bdring v4 3-axis board" ON)
OPTION(BOARD_BDRING_I2S6A "Compile for bdring 6-axis I2S board - untested!" OFF)
OPTION(BOARD_CNC_BOOSTERPACK "Compile for CNC BoosterPack" OFF)

# The following plugins/options are supported:

OPTION(Networking "Wifi + protocols" OFF)
OPTION(Bluetooth "Bluetooth" OFF)
OPTION(Keypad "I2C Keypad" OFF)
OPTION(SDcard "SD Card Streaming" OFF)
OPTION(Trinamic "Trinamic driver support over I2C" OFF)
OPTION(WebUI "WebUI services" OFF)
OPTION(WebAuth "WebUI authentication" OFF)
OPTION(MPGMode "MPG mode" OFF)
OPTION(I2SStepping "Use I2S Stepping" OFF)

# For Huanyan spindle support copy the spindle plugin code to the project.
OPTION(HUANYANG "Compile with Huanyang RS485 Spindle support" OFF)

# ModBus options
OPTION(RS485_DIR_OUT "Compile with RS485 direction pin enabled" ON)

# For EEPROM/FRAM support copy the EEPROM plugin code to the project.
OPTION(EEPROM "Compile with I2C EEPROM support" OFF)
OPTION(FRAM "Compile with I2C FRAM support" OFF)

set(SDCARD_SOURCE sdcard/sdcard.c)
set(KEYPAD_SOURCE keypad/keypad.c)
set(TRINAMIC_SOURCE trinamic/trinamic2130.c trinamic/TMC2130_I2C_map.c tmc2130/trinamic.c)
set(NETWORKING_SOURCE wifi.c dns_server.c web/backend.c web/upload.c networking/TCPStream.c networking/WsStream.c networking/base64.c networking/sha1.c networking/urldecode.c networking/strutils.c networking/utils.c networking/multipartparser.c )
set(WEBUI_SOURCE webui/server.c webui/response.c webui/commands.c webui/flashfs.c )
set(BLUETOOTH_SOURCE bluetooth.c )
set(HUANYANG_SOURCE spindle/huanyang.c spindle/modbus)
set(EEPROM_SOURCE eeprom/eeprom_24LC16B.c eeprom/eeprom_24AAxxx.c)

set(MAIN_SRCS main.c driver.c nvs.c esp32-hal-uart.c
 i2c.c
 ioexpand.c
 i2s_out.c
 grbl/grbllib.c
 grbl/coolant_control.c
 grbl/nvs_buffer.c
 grbl/gcode.c
 grbl/limits.c
 grbl/motion_control.c
 grbl/my_plugin.c
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
 grbl/tool_change.c
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

if(Bluetooth) 
list (APPEND MAIN_SRCS ${BLUETOOTH_SOURCE})
endif()

if(HUANYANG)
list (APPEND MAIN_SRCS ${HUANYANG_SOURCE})
endif()

if(EEPROM OR FRAM OR BOARD_CNC_BOOSTERPACK)
list (APPEND MAIN_SRCS ${EEPROM_SOURCE})
endif()

set(INCLUDE_DIRS ".")

#file(GLOB GRBL_SOURCE "grbl/*.c")
#list (APPEND GRBL_SOURCE MAIN_SRCS)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(grbl)

target_compile_definitions(grbl.elf PUBLIC GRBL_ESP32)
target_compile_definitions(grbl.elf PUBLIC OVERRIDE_MY_MACHINE)

if(BOARD_BDRING_V3P5)
target_compile_definitions(grbl.elf PUBLIC BOARD_BDRING_V3P5)
elseif(BOARD_BDRING_V4)
target_compile_definitions(grbl.elf PUBLIC BOARD_BDRING_V4)
elseif(BOARD_BDRING_I2S6A)
target_compile_definitions(grbl.elf PUBLIC BOARD_BDRING_I2S6A)
elseif(BOARD_CNC_BOOSTERPACK)
target_compile_definitions(grbl.elf PUBLIC BOARD_CNC_BOOSTERPACK)
endif()

if(Networking)
target_compile_definitions(grbl.elf PUBLIC NETWORKING_ENABLE)
endif()

if(Bluetooth)
target_compile_definitions(grbl.elf PUBLIC BLUETOOTH_ENABLE)
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

if(HUANYANG)
target_compile_definitions(grbl.elf PUBLIC SPINDLE_HUANYANG=1)
target_compile_definitions(grbl.elf PUBLIC SPINDLE_RPM_CONTROLLED)
endif()

if(RS485_DIR_OUT)
target_compile_definitions(grbl.elf PUBLIC RS485_DIR_ENABLE)
endif()

if(EEPROM OR BOARD_CNC_BOOSTERPACK)
target_compile_definitions(grbl.elf PUBLIC EEPROM_ENABLE)
endif()

if(FRAM)
target_compile_definitions(grbl.elf PUBLIC EEPROM_IS_FRAM)
endif()

target_add_binary_data(grbl.elf "favicon.ico" BINARY)
target_add_binary_data(grbl.elf "index.html" BINARY)
target_add_binary_data(grbl.elf "ap_login.html" BINARY)

unset(BOARD_BDRING_V3P5 CACHE)
unset(BOARD_BDRING_V4 CACHE)
unset(BOARD_BDRING_I2S6A CACHE)
unset(BOARD_CNC_BOOSTERPACK CACHE)

unset(Networking CACHE)
unset(Bluetooth CACHE)
unset(Keypad CACHE)
unset(SDcard CACHE)
unset(Trinamic CACHE)
unset(WebUI CACHE)
unset(WebAuth CACHE)
unset(MPGMode CACHE)
unset(HUANYANG CACHE)
unset(RS485_DIR_OUT CACHE)
unset(EEPROM CACHE)
unset(FRAM CACHE)

include_directories(BEFORE ".")

#target_compile_options(grbl.elf PRIVATE -Werror -Wall -Wextra)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#add_subdirectory (grbl)
