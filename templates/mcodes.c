/*

  mcode.c - user defined M-codes template

  Part of GrblHAL

  Copyright (c) 2019 Terje Io

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

/*
 * NOTE: this template is also a bare bones example for adding M100 with two parameters: P and Q
 */

#ifdef ARDUINO
#include "src/grbl/grbl.h"
#else
#include "grbl/grbl.h"
#endif

// check - check if M-code is handled here.
// parameters: mcode - M-code to check for (some are predefined in user_mcode_t in grbl/gcode.h), use a cast if not.
// returns:    mcode if handled, UserMCode_Ignore otherwise (UserMCode_Ignore is defined in grbl/gcode.h).
static user_mcode_t check (user_mcode_t mcode)
{
    return mcode == (user_mcode_t)100 ? mcode : UserMCode_Ignore;
}

// validate - validate parameters
// parameters: gc_block - pointer to parser_block_t struct (defined in grbl/gcode.h).
//             value_words - pointer to bit map of value words available.
// returns:    status_code_t enum (defined in grbl/gcode.h): Status_OK if validated ok, appropriate status from enum if not.
static status_code_t validate (parser_block_t *gc_block, uint_fast16_t *value_words) {

    status_code_t state = Status_GcodeValueWordMissing;

    switch(gc_block->user_mcode) {

        case 100:
            if(bit_istrue(*value_words, bit(Word_P)|bit(Word_Q))) {         // Are required parameters provided?
                if(gc_block->values.q > 0.0f && gc_block->values.q <= 5.0f) // Yes, is Q parameter value in range (1-5)?
                    state = Status_OK;                                      // Yes - return ok status.
                else
                    state = Status_GcodeValueOutOfRange;                    // No - return error status.
                bit_false(*value_words, bit(Word_P)|bit(Word_Q));           // Claim parameters.
                gc_block->user_mcode_sync = true;                           // Optional: execute command synchronized
            }
            break;

        default:
            break;
    }

    return state;
}

// execute - execute M-code
// parameters: state - sys.state (bitmap, defined in system.h)
//             gc_block - pointer to parser_block_t struct (defined in grbl/gcode.h).
// returns:    -
static void execute (uint_fast16_t state, parser_block_t *gc_block) {

    switch((uint32_t)gc_block->user_mcode) {

        case 100:
            // do something: parameters in gc_block->values.p and gc_block->values.q
            break;

        default:
            break;
    }
}

// Set up HAL pointers for handling additional M-codes.
// Call this function on driver setup.
void mcodes_init (void)
{
    hal.user_mcode_check = check;
    hal.user_mcode_validate = validate;
    hal.user_mcode_execute = execute;
}
