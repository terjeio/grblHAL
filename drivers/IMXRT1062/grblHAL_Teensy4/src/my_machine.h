/*
  my_machine.h - configuration for IMXRT1062 processor (on Teensy 4.x board)

  Part of grblHAL

  Copyright (c) 2020 Terje Io

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

// BOARD_T40X101 and BOARD_T41U5XBB by Phil Barrett: https://github.com/phil-barrett/grblHAL-teensy-4.x

// NOTE: Only one board may be enabled!
// If none is enabled pin mappings from generic_map.h will be used
//#define BOARD_T40X101
#define BOARD_T41U5XBB
//#define BOARD_CNC_BOOSTERPACK
//#define BOARD_MY_MACHINE // Add my_machine_map.h before enabling this!

// Configuration
// Uncomment to enable, for some a value > 1 may be assigned, if so the default value is shown.

/*
              Plugin: | ETHERNET¹ | SDCARD¹ | KEYPAD | EEPROM | N_AXIS |
----------------------|-----------|---------|--------|--------|--------|
BOARD_T40X101         | no        | no      | yes    | yes³   | max 4  |
BOARD_T41U5XBB        | yes       | yes     | yes    | yes³   | max 5  |
BOARD_CNC_BOOSTERPACK | yes²      | yes     | yes    | yes    | max 3  |

¹ Teensy 4.1 only.
² External magjack.
³ EEPROM is optional and must be added to the board.

N_AXIS has a default value of 3, edit grbl\config.h to increase.

*/

#define USB_SERIAL_CDC       2 // 1 for Arduino class library and 2 for PJRC C library. Comment out to use UART communication.
//#define USB_SERIAL_WAIT    1 // Wait for USB connection before starting grblHAL.
//#define SPINDLE_HUANYANG   1 // Set to 1 or 2 for Huanyang VFD spindle. Requires spindle plugin.
//#define QEI_ENABLE         1 // Enable quadrature encoder interfaces. Max value is 1. Requires encoder plugin.
//#define ETHERNET_ENABLE    1 // Ethernet streaming. Requires networking plugin.
//#define SDCARD_ENABLE      1 // Run gcode programs from SD card, requires sdcard plugin.
//#define KEYPAD_ENABLE      1 // I2C keypad for jogging etc., requires keypad plugin.
//#define PLASMA_ENABLE      1 // Plasma/THC plugin. To be completed.
//#define PPI_ENABLE         1 // Laser PPI plugin. To be completed.
//#define ODOMETER_ENABLE    1 // Odometer plugin. To be completed.
//#define EEPROM_ENABLE      1 // I2C EEPROM support. Set to 1 for 24LC16(2K), 2 for larger sizes. Requires eeprom plugin.
//#define EEPROM_IS_FRAM     1 // Uncomment when EEPROM is enabled and chip is FRAM, this to remove write delay.

//#define ESTOP_ENABLE       0 // When enabled only real-time report requests will be executed when the reset pin is asserted.
                               // Note: if left commented out the default setting is determined from COMPATIBILITY_LEVEL.

#if ETHERNET_ENABLE > 0
#define TELNET_ENABLE           1 // Telnet daemon - requires Ethernet streaming enabled.
#define WEBSOCKET_ENABLE        1 // Websocket daemon - requires Ethernet streaming enabled.
#define NETWORK_HOSTNAME        "GRBL"
#define NETWORK_IPMODE          1 // 0 = static, 1 = DHCP, 2 = AutoIP
#define NETWORK_IP              "192.168.5.1"
#define NETWORK_GATEWAY         "192.168.5.1"
#define NETWORK_MASK            "255.255.255.0"
#define NETWORK_TELNET_PORT     23
#define NETWORK_WEBSOCKET_PORT  80
#define NETWORK_HTTP_PORT       80
#endif
