# Select board to compile for:

# NOTE: Only one board may be enabled!
# If none is enabled pin mappings from generic_map.h will be used.

OPTION(BOARD_BDRING_V3P5 "Compile for v3.5 3-axis board" OFF)
OPTION(BOARD_BDRING_V4 "Compile for bdring v4 3-axis board" ON)
OPTION(BOARD_BDRING_I2S6A "Compile for bdring 6-axis I2S board - untested!" OFF)
OPTION(BOARD_CNC_BOOSTERPACK "Compile for CNC BoosterPack" OFF)
OPTION(BOARD_ESPDUINO32 "Compile for ESPDUINO-32 Wemos D1 R32" OFF)
OPTION(BOARD_MY_MACHINE "Compile for my_machine_map.h" OFF)

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

# Networking options (WiFi)
OPTION(SoftAP "Enable soft AP mode" OFF)

# For Huanyan spindle support copy the spindle plugin code to the project.
OPTION(HUANYANG "Compile with Huanyang RS485 Spindle support" OFF)

# ModBus options
OPTION(RS485_DIR_OUT "Compile with RS485 direction pin enabled" OFF)

# For EEPROM/FRAM support copy the EEPROM plugin code to the project.
OPTION(EEPROM "Compile with I2C EEPROM support" OFF)
OPTION(FRAM "Compile with I2C FRAM support" OFF)
OPTION(NOPROBE "Compile without probe support" OFF)

set(SDCARD_SOURCE sdcard/sdcard.c)
set(KEYPAD_SOURCE keypad/keypad.c)
set(TRINAMIC_SOURCE motors/trinamic.c trinamic/tmc2130.c trinamic/tmc2130hal.c trinamic/common.c trinamic/tmc_interface.c)
set(NETWORKING_SOURCE wifi.c dns_server.c web/backend.c web/upload.c networking/TCPStream.c networking/WsStream.c networking/base64.c networking/sha1.c networking/urldecode.c networking/strutils.c networking/utils.c networking/multipartparser.c )
set(WEBUI_SOURCE webui/server.c webui/response.c webui/commands.c webui/flashfs.c )
set(BLUETOOTH_SOURCE bluetooth.c )
set(HUANYANG_SOURCE spindle/huanyang.c spindle/modbus)
set(EEPROM_SOURCE eeprom/eeprom_24LC16B.c eeprom/eeprom_24AAxxx.c)

set(SRCS main.c driver.c nvs.c esp32-hal-uart.c
 i2c.c
 ioexpand.c
 i2s_out.c
 networking/strutils.c
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
 grbl/stream.c
 grbl/stepper.c
 grbl/system.c
 grbl/tool_change.c
)

if(Networking)
list (APPEND SRCS ${NETWORKING_SOURCE})
endif()

if(SDcard)
list (APPEND SRCS ${SDCARD_SOURCE})
endif()

if(Keypad)
list (APPEND SRCS ${KEYPAD_SOURCE})
endif()

if(Trinamic)
list (APPEND SRCS ${TRINAMIC_SOURCE})
endif()

if(WebUI)
list (APPEND SRCS ${WEBUI_SOURCE})
endif()

if(Bluetooth)
list (APPEND SRCS ${BLUETOOTH_SOURCE})
endif()

if(HUANYANG)
list (APPEND SRCS ${HUANYANG_SOURCE})
endif()

if(EEPROM OR FRAM OR BOARD_CNC_BOOSTERPACK)
list (APPEND SRCS ${EEPROM_SOURCE})
endif()

idf_component_register(SRCS "${SRCS}"
                    INCLUDE_DIRS ".")

target_compile_definitions("${COMPONENT_LIB}" PUBLIC GRBL_ESP32)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC OVERRIDE_MY_MACHINE)

if(BOARD_BDRING_V3P5)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC BOARD_BDRING_V3P5)
elseif(BOARD_BDRING_V4)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC BOARD_BDRING_V4)
elseif(BOARD_BDRING_I2S6A)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC BOARD_BDRING_I2S6A)
elseif(BOARD_ESPDUINO32)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC BOARD_ESPDUINO32)
elseif(BOARD_CNC_BOOSTERPACK)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC BOARD_CNC_BOOSTERPACK)
elseif(BOARD_MY_MACHINE)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC BOARD_MY_MACHINE)
endif()

if(Networking)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC NETWORKING_ENABLE)
endif()

if(SoftAP)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC WIFI_SOFTAP)
endif()

if(Bluetooth)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC BLUETOOTH_ENABLE)
endif()

if(SDcard)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC SDCARD_ENABLE)
endif()

if(WebUI)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC WEBUI_ENABLE)
target_add_binary_data("${COMPONENT_LIB}" "index.html.gz" BINARY)
if(WebUI)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC AUTH_ENABLE)
endif()
endif()

if(Trinamic)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC TRINAMIC_ENABLE)
endif()

if(Keypad)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC KEYPAD_ENABLE)
endif()

if(MPGMode)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC MPG_MODE_ENABLE)
endif()

if(HUANYANG)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC SPINDLE_HUANYANG=1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC SPINDLE_RPM_CONTROLLED)
endif()

if(RS485_DIR_OUT)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC RS485_DIR_ENABLE)
endif()

if(EEPROM OR BOARD_CNC_BOOSTERPACK)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC EEPROM_ENABLE)
endif()

if(FRAM)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC EEPROM_IS_FRAM)
endif()

if(NOPROBE)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC NOPROBE)
endif()

target_add_binary_data("${COMPONENT_LIB}" "favicon.ico" BINARY)
target_add_binary_data("${COMPONENT_LIB}" "index.html" BINARY)
target_add_binary_data("${COMPONENT_LIB}" "ap_login.html" BINARY)

unset(BOARD_BDRING_V3P5 CACHE)
unset(BOARD_BDRING_V4 CACHE)
unset(BOARD_BDRING_I2S6A CACHE)
unset(BOARD_CNC_BOOSTERPACK CACHE)
unset(BOARD_ESPDUINO32 CACHE)
unset(BOARD_MY_MACHINE CACHE)

unset(Networking CACHE)
unset(SoftAP CACHE)
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
unset(NOPROBE CACHE)

#target_compile_options("${COMPONENT_LIB}" PRIVATE -Werror -Wall -Wextra -Wmissing-field-initializers)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
