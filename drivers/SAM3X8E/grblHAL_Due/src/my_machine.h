/*
  my_machine.h - configuration for Atmel SAM3X8E ARM processor

  Part of grblHAL

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
// If none is enabled pin mappings from generic_map.h will be used
// NOTE: Only one board may be enabled!
//#define BOARD_TINYG2_DUE
#define BOARD_RAMPS_16
//#define BOARD_MEGA256
//#define BOARD_PROTONEER
//#define BOARD_CMCGRATH
//#define BOARD_RADDS_16   // NOTE: untested!!
//#define BOARD_MY_MACHINE // Add my_machine_map.h before enabling this!

// Configuration
// Uncomment to enable.

//#define USB_SERIAL_CDC     1 // Use native USB port for communication.
//#define SPINDLE_HUANYANG   1 // Set to 1 or 2 for Huanyang VFD spindle. Requires spindle plugin.
//#define KEYPAD_ENABLE      1 // I2C keypad for jogging etc., requires keypad plugin.
//#define EEPROM_ENABLE      1 // I2C EEPROM support. Set to 1 for 24LC16(2K), 2 for larger sizes. Requires eeprom plugin.
//#define EEPROM_IS_FRAM     1 // Uncomment when EEPROM is enabled and chip is FRAM, this to remove write delay.

/**/
