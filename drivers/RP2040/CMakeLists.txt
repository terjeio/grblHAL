cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(grblHAL)

pico_sdk_init()


add_executable(grblHAL
 main.c
 driver.c
 serial.c
 i2c.c
 PCA9654E.c
 eeprom/eeprom_24AAxxx.c
 eeprom/eeprom_24LC16B.c
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
 grbl/my_plugin.c
)

pico_generate_pio_header(grblHAL ${CMAKE_CURRENT_LIST_DIR}/driverPIO.pio)

target_sources(grblHAL PRIVATE driver.c)

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(grblHAL PRIVATE pico_stdlib hardware_uart hardware_pio hardware_i2c hardware_gpio hardware_pwm hardware_clocks)

# create map/bin/hex file etc.
pico_add_extra_outputs(grblHAL)