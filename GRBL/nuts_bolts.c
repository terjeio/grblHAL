/*
  nuts_bolts.c - Shared functions
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
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


#define MAX_INT_DIGITS 8 // Maximum number of digits in int32 (and float)


// Extracts a floating point value from a string. The following code is based loosely on
// the avr-libc strtod() function by Michael Stumpf and Dmitry Xmelkov and many freely
// available conversion method examples, but has been highly optimized for Grbl. For known
// CNC applications, the typical decimal value is expected to be in the range of E0 to E-4.
// Scientific notation is officially not supported by g-code, and the 'E' character may
// be a g-code word on some CNC systems. So, 'E' notation will not be recognized.
// NOTE: Thanks to Radu-Eosif Mihailescu for identifying the issues with using strtod().
bool read_float (char *line, uint_fast8_t *char_counter, float *float_ptr)
{
    char c, *ptr = line + *char_counter;
    int_fast8_t exp = 0;
    uint_fast8_t ndigit = 0;
    uint32_t intval = 0;
    bool isnegative, isdecimal = false;

    // Grab first character and increment pointer. No spaces assumed in line.
    c = *ptr++;

    // Capture initial positive/minus character
    if ((isnegative = (c == '-')) || c == '+')
        c = *ptr++;

    // Extract number into fast integer. Track decimal in terms of exponent value.
    while(true) {
        c -= '0';
        if (c <= 9) {
            ndigit++;
            if (ndigit <= MAX_INT_DIGITS) {
                if (isdecimal)
                    exp--;
                intval = (((intval << 2) + intval) << 1) + c; // intval*10 + c
            } else if (!isdecimal)
                exp++;  // Drop overflow digits
        } else if (c == (('.'-'0') & 0xff) && !isdecimal)
            isdecimal = true;
         else
            break;

        c = *ptr++;
    }

    // Return if no digits have been read.
    if (!ndigit)
        return(false);

    // Convert integer into floating point.
    float fval = (float)intval;

    // Apply decimal. Should perform no more than two floating point multiplications for the
    // expected range of E0 to E-4.
    if (fval != 0.0f) {
        while (exp <= -2) {
            fval *= 0.01f;
            exp += 2;
        }
        if (exp < 0)
            fval *= 0.1f;
        else if (exp > 0) do {
            fval *= 10.0f;
        } while (--exp > 0);
    }

    // Assign floating point value with correct sign.
    *float_ptr = isnegative ? - fval : fval;
    *char_counter = ptr - line - 1; // Set char_counter to next statement

    return true;
}


// Non-blocking delay function used for general operation and suspend features.
void delay_sec (float seconds, delaymode_t mode)
{
    uint_fast16_t i = (uint_fast16_t)ceilf((1000.0f / DWELL_TIME_STEP) * seconds) + 1;

    while (--i && !sys.abort) {
        if (mode == DelayMode_Dwell) {
            protocol_execute_realtime();
        } else { // DelayMode_SysSuspend
          // Execute rt_system() only to avoid nesting suspend loops.
          protocol_exec_rt_system();
          if (state_door_reopened()) // Bail, if safety door reopens.
              return;
        }
        hal.delay_ms(DWELL_TIME_STEP, 0); // Delay DWELL_TIME_STEP increment
    }
}


float convert_delta_vector_to_unit_vector (float *vector)
{
    uint_fast8_t idx = N_AXIS;
    float magnitude = 0.0f, inv_magnitude;

    do {
        if (vector[--idx] != 0.0f)
            magnitude += vector[idx] * vector[idx];
    } while(idx);

    idx = N_AXIS;
    magnitude = sqrtf(magnitude);
    inv_magnitude = 1.0f / magnitude;

    do {
        vector[--idx] *= inv_magnitude;
    } while(idx);

    return magnitude;
}


float limit_value_by_axis_maximum (float *max_value, float *unit_vec)
{
    uint_fast8_t idx = N_AXIS;
    float limit_value = SOME_LARGE_VALUE;

    do {
        if (unit_vec[--idx] != 0.0f)  // Avoid divide by zero.
            limit_value = min(limit_value, fabsf(max_value[idx] / unit_vec[idx]));
    } while(idx);

    return limit_value;
}

// calculate checksum byte for EEPROM data
uint8_t calc_checksum (uint8_t *data, uint32_t size) {

    uint8_t checksum = 0;

    while(size--) {
        checksum = (checksum << 1) | (checksum >> 7);
        checksum += *(data++);
    }

    return checksum;
}

