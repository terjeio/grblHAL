/*
  print.c - Functions for formatting output strings
  Part of Grbl

  Copyright (c) 2017-2018 Terje Io
  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#include "grbl.h"

static char buf[12];
#define MAX_PRECISION   (10)
static const float froundvalues[MAX_PRECISION + 1] =
{
    0.5,                // 0
    0.05,               // 1
    0.005,              // 2
    0.0005,             // 3
    0.00005,            // 4
    0.000005,           // 5
    0.0000005,          // 6
    0.00000005,         // 7
    0.000000005,        // 8
    0.0000000005,       // 9
    0.00000000005       // 10
};

// Prints an uint8 variable in base 10.
void print_uint8_base10 (uint8_t n)
{
    uint8_t d;

    if(n >= 100) {
        d = n / 100U;
        n -= d * 100U;
        hal.serial_write('0' + d);
        if(n < 10)
            hal.serial_write('0');
    }

    if(n >= 10) {
        d = n / 10U;
        n -= d * 10U;
        hal.serial_write('0' + d);
    }

    hal.serial_write('0' + n);
}


// Prints an uint32 variable in base 10.
void print_uint32_base10 (uint32_t n)
{
    if (n == 0) {
        hal.serial_write('0');
        return;
    }

    char *bptr = buf + sizeof(buf);

    *--bptr = '\0';

    while (n) {
        *--bptr = '0' + (n % 10);
        n /= 10;
    }

    hal.serial_write_string(bptr);
}

// Convert float to string by immediately converting to integers.
// Number of decimal places, which are tracked by a counter, must be set by the user.
// The integers is then efficiently converted to a string.
char *ftoa (float n, uint8_t decimal_places)
{
    bool isNegative;
    char *bptr = buf + sizeof(buf);

    *--bptr = '\0';

    if ((isNegative = n < 0.0f))
        n = -n;

    n += froundvalues[decimal_places];

    uint32_t a = (uint32_t)n;

    if (decimal_places) {

        n -= (float)a;

        uint_fast8_t decimals = decimal_places;
        while (decimals >= 2) { // Quickly convert values expected to be E0 to E-4.
            n *= 100.0f;
            decimals -= 2;
        }

        if (decimals)
            n *= 10.0f;

        uint32_t b = (uint32_t)n;

        while(decimal_places--) {
            if(b) {
                *--bptr = (b % 10) + '0'; // Get digit
                b /= 10;
            } else
                *--bptr = '0';
        }
    }

    *--bptr = '.'; // Always add decimal point (TODO: is this really needed?)

    if(a == 0)
        *--bptr = '0';

    else while(a) {
        *--bptr = (a % 10) + '0'; // Get digit
        a /= 10;
    }

    if(isNegative)
        *--bptr = '-';

    return bptr;
}

// Convert float to string by immediately converting to a long integer, which contains
// more digits than a float. Number of decimal places, which are tracked by a counter,
// may be set by the user. The integer is then efficiently converted to a string.
void printFloat (float n, uint8_t decimal_places)
{
    hal.serial_write_string(ftoa(n, decimal_places));
/*
   if (n < 0.0f) {
        hal.serial_write('-');
        n = -n;
    }

    uint_fast8_t decimals = decimal_places;
    while (decimals >= 2) { // Quickly convert values expected to be E0 to E-4.
        n *= 100.0f;
        decimals -= 2;
    }

    if (decimals)
        n *= 10.0f;

    n += 0.5f; // Add rounding factor. Ensures carryover through entire value.

    // Generate digits backwards and store in string.
    uint_fast8_t i = 0;
    uint32_t a = (uint32_t)n;
    buf[decimal_places] = '.'; // Place decimal point, even if decimal places are zero.

    while(a > 0) {
        if (i == decimal_places)
            i++; // Skip decimal point location
        buf[i++] = (a % 10) + '0'; // Get digit
        a /= 10;
    }

    while (i < decimal_places) {
        buf[i++] = '0'; // Fill in zeros to decimal point for (n < 1)
    }

    if (i == decimal_places) { // Fill in leading zero, if needed.
        i++;
        buf[i++] = '0';
    }

    while(i)
        hal.serial_write(buf[--i]);
*/
}


// Floating value printing handlers for special variables types used in Grbl and are defined
// in the config.h.
//  - CoordValue: Handles all position or coordinate values in inches or mm reporting.
//  - RateValue: Handles feed rate and current velocity in inches or mm reporting.
//  - SettingValue: Handles all floating point settings values (always in mm.)
void printFloat_CoordValue (float n) {
    if (settings.flags.report_inches)
        hal.serial_write_string(ftoa(n, N_DECIMAL_COORDVALUE_INCH));
    else
        hal.serial_write_string(ftoa(n, N_DECIMAL_COORDVALUE_MM));
}

void printFloat_RateValue (float n) {
    if (settings.flags.report_inches)
        hal.serial_write_string(ftoa(n, N_DECIMAL_RATEVALUE_INCH));
    else
        hal.serial_write_string(ftoa(n, N_DECIMAL_RATEVALUE_MM));
}

void printFloat_SettingValue (float n)
{
    hal.serial_write_string(ftoa(n, N_DECIMAL_SETTINGVALUE));
}
