/*
  my_machine.h - configuration for STM32F4xx ARM processors

  Part of GrblHAL

  Copyright (c) 2020-2021 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

// NOTE: Only one board may be enabled!
// If none is enabled pin mappings from generic_map.h will be used.
//#define BOARD_CNC3040
//#define BOARD_PROTONEER_3XX   // For use with a Nucleo-F411RE and F446RE boards.
//#define BOARD_GENERIC_UNO     // For use with a Nucleo-F411RE and F446RE boards.
//#define BOARD_CNC_BOOSTERPACK
//#define BOARD_MORPHO_CNC      // For use with Nucleo-F411RE and F446RE boards. Work in progress.
//#define BOARD_MORPHO_DAC_CNC  // For use with F446RE boards and a 32-bit I2S stereo DAC for X and Y output. **EXPERIMENTAL**
//#define BOARD_MINI_BLACKPILL
//#define BOARD_MY_MACHINE      // Add my_machine_map.h before enabling this!

// Configuration
// Uncomment to enable.

#if !(defined(NUCLEO_F411) || defined(NUCLEO_F446)) // The Nucleo-F411RE board has an off-chip UART to USB interface.
#define USB_SERIAL_CDC       1 // Serial communication via native USB.
#endif
//#define SPINDLE_HUANYANG     1 // Set to 1 or 2 for Huanyang VFD spindle. Requires spindle plugin. !! NOT TESTED !!
//#define SDCARD_ENABLE        1 // Run gcode programs from SD card, requires sdcard plugin.
//#define KEYPAD_ENABLE        1 // I2C keypad for jogging etc., requires keypad plugin.
//#define ODOMETER_ENABLE      1 // Odometer plugin.
//#define PPI_ENABLE           1 // Laser PPI plugin. To be completed.
//#define LASER_COOLANT_ENABLE 1 // Laser coolant plugin. To be completed.
//#define TRINAMIC_ENABLE   2130 // Trinamic TMC2130 stepper driver support. NOTE: work in progress.
//#define TRINAMIC_ENABLE   5160 // Trinamic TMC5160 stepper driver support. NOTE: work in progress.
//#define TRINAMIC_I2C         1 // Trinamic I2C - SPI bridge interface.
//#define TRINAMIC_DEV         1 // Development mode, adds a few M-codes to aid debugging. Do not enable in production code.
//#define EEPROM_ENABLE        1 // I2C EEPROM support. Set to 1 for 24LC16(2K), 2 for larger sizes. Requires eeprom plugin.
//#define EEPROM_IS_FRAM       1 // Uncomment when EEPROM is enabled and chip is FRAM, this to remove write delay.
/**/
