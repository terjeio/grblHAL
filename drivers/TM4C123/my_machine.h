/*
  my_machine.h - configuration for Texas Instruments Tiva C (TM4C123GH6PM) ARM processor

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
// Uncomment to enable.

//#define KEYPAD_ENABLE           1 // I2C keypad for jogging etc.
//#define TRINAMIC_ENABLE      2130 // Trinamic TMC2130 stepper driver support. NOTE: work in progress.
//#define TRINAMIC_ENABLE      5160 // Trinamic TMC5160 stepper driver support. NOTE: work in progress.
//#define TRINAMIC_I2C            0 // Trinamic I2C - SPI bridge interface.
//#define PWM_RAMPED              1 // Ramped spindle PWM.
//#define PPI_ENABLE              1 // Laser PPI (Pulses Per Inch) option.

#ifdef BOARD_CNC_BOOSTERPACK
#define CNC_BOOSTERPACK_SHORTS  1 // Shorts added to BoosterPack for some signals (for faster and simpler driver)
#define CNC_BOOSTERPACK_A4998   1 // Using Polulu A4998 drivers - for suppying VDD via GPIO (PE5)
#endif
