/*

  mcode.c - user defined M-codes template

  Part of grblHAL

  Copyright (c) 2019-2021 Terje Io

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

#include <math.h>
#include <string.h>

#ifdef ARDUINO
#include "../grbl/hal.h"
#else
#include "grbl/hal.h"
#endif

static user_mcode_ptrs_t user_mcode;

// check - check if M-code is handled here.
// parameters: mcode - M-code to check for (some are predefined in user_mcode_t in grbl/gcode.h), use a cast if not.
// returns:    mcode if handled, UserMCode_Ignore otherwise (UserMCode_Ignore is defined in grbl/gcode.h).
static user_mcode_t check (user_mcode_t mcode)
{
    return mcode == UserMCode_Generic0
                     ? mcode                                                            // Handled by us.
                     : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Ignore); // If another handler present then call it or return ignore.
}

// validate - validate parameters
// parameters: gc_block - pointer to parser_block_t struct (defined in grbl/gcode.h).
//             parameter_words - pointer to bitfield of parameter words available.
//             If float values are NAN (Not A Number) this means they are not available.
//             If integer values has all bits set to 1 this means they are not available.
// returns:    status_code_t enum (defined in grbl/gcode.h): Status_OK if validated ok, appropriate status from enum if not.
static status_code_t validate (parser_block_t *gc_block, parameter_words_t *parameter_words)
{
    status_code_t state = Status_GcodeValueWordMissing;

    switch(gc_block->user_mcode) {

        case UserMCode_Generic0:
            if((*parameter_words).p && !isnan(gc_block->values.p))  // Check if P parameter value is supplied.
                state = Status_BadNumberFormat;                     // Return error if so.

            if((*parameter_words).q && isnan(gc_block->values.q))   // Check if Q parameter value is supplied.
                state = Status_BadNumberFormat;                     // Return error if not.

            if(state != Status_BadNumberFormat && (*parameter_words).q) {       // Are required parameters provided?
                if(gc_block->values.q > 0.0f && gc_block->values.q <= 5.0f)     // Yes, is Q parameter value in range (1-5)?
                    state = Status_OK;                                          // Yes - return ok status.
                else
                    state = Status_GcodeValueOutOfRange;                        // No - return error status.
                if((*parameter_words).q)                                        // If P parameter is present set
                    gc_block->values.p = 1.0f;                                  // value to 1 for execution.
                (*parameter_words).p = (*parameter_words).q = Off;              // Claim parameters.
                gc_block->user_mcode_sync = true;                               // Optional: execute command synchronized
            }
            break;

        default:
            state = Status_Unhandled;
            break;
    }

    // If not handled by us and another handler present then call it.
    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block, parameter_words) : state;
}

// execute - execute M-code
// parameters: state - sys.state (bitmap, defined in system.h)
//             gc_block - pointer to parser_block_t struct (defined in grbl/gcode.h).
// returns:    -
static void execute (sys_state_t state, parser_block_t *gc_block) {

    bool handled = true;

    switch(gc_block->user_mcode) {

        case UserMCode_Generic0:
            // do something: Q parameter value can be found in gc_block->values.q.
            //               P parameter has its value in gc_block->values.p set to 1 if present, NAN if not.
            break;

        default:
            handled = false;
            break;
    }


    if(!handled && user_mcode.execute)          // If not handled by us and another handler present
        user_mcode.execute(state, gc_block);    // then call it.
}

// Set up HAL pointers for handling additional M-codes.
// Call this function on driver setup.
void mcodes_init (void)
{
    // Save away current HAL pointers so that we can use them to keep
    // any chain of M-code handlers intact.
    memcpy(&user_mcode, &hal.user_mcode, sizeof(user_mcode_ptrs_t));

    // Redirect HAL pointers to our code.
    hal.user_mcode.check = check;
    hal.user_mcode.validate = validate;
    hal.user_mcode.execute = execute;
}
