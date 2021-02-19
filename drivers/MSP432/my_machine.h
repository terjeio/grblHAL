/*
  my_machine.h - configuration for MSP432 processor

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
#define BOARD_CNC_BOOSTERPACK
//#define BOARD_MY_MACHINE // Add my_machine_map.h before enabling this!

// Configuration
// Uncomment to enable, for some a value > 1 may be assigned, if so the default value is shown.

//#define PLASMA_ENABLE      1 // Plasma/THC plugin.
//#define SPINDLE_HUANYANG   1 // Set to 1 or 2 for Huanyang VFD spindle. Requires spindle plugin.
//#define KEYPAD_ENABLE      1 // I2C keypad for jogging etc., requires keypad plugin.
//#define TRINAMIC_ENABLE 2130 // Trinamic TMC2130 stepper driver support. NOTE: work in progress.
//#define TRINAMIC_ENABLE 5160 // Trinamic TMC5160 stepper driver support. NOTE: work in progress.
//#define TRINAMIC_I2C       0 // Trinamic I2C - SPI bridge interface.
//#define ODOMETER_ENABLE    1 // Odometer plugin. To be completed.
//#define EEPROM_ENABLE      1 // I2C EEPROM support. Set to 1 for 24LC16(2K), 2 for larger sizes. Requires eeprom plugin.
//#define EEPROM_IS_FRAM     1 // Uncomment when EEPROM is enabled and chip is FRAM, this to remove write delay.
#define LIMITS_OVERRIDE_ENABLE 1

//#define ESTOP_ENABLE       0 // When enabled only real-time report requests will be executed when the reset pin is asserted.
                               // Note: if left commented out the default setting is determined from COMPATIBILITY_LEVEL.

#ifdef BOARD_CNC_BOOSTERPACK
#define CNC_BOOSTERPACK_SHORTS 0 // Shorts added to BoosterPack for some signals (for faster and simpler driver)
#define CNC_BOOSTERPACK_A4998  1 // Using Polulu A4998 drivers - for suppying VDD via GPIO (PE5)
#endif
