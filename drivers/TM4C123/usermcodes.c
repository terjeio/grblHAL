/*
  usermcodes.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for Texas Instruments Tiva C (TM4C123GH6PM) ARM processor

  Part of GrblHAL

  Copyright (c) 2017-2018 Terje Io

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

// User defined mcode handling

#include <math.h>

#include "grbl/grbl.h"

#include "tiva.h"
#include "atc.h"
#include "driver.h"

//
// userMcodeCheck - return mcode if implemented, 0 otherwise
//

user_mcode_t userMCodeCheck (user_mcode_t mcode)
{
#ifdef LASER_PPI
    return mcode == 6 || mcode == 61 || mcode == 100 || mcode == 101 || ((mcode == 123 || mcode == 124) && settings.flags.laser_mode) ? mcode : UserMCode_Ignore;
#else
    return mcode == 6 || mcode == 61 || mcode == 100 || mcode == 101 ? mcode : UserMCode_Ignore;
#endif
}

//
// userMCodeValidate - validate parameters, return STATUS_OK when ok
//

status_code_t userMCodeValidate (parser_block_t *gc_block, uint_fast16_t *value_words) {

    status_code_t state = Status_GcodeValueWordMissing;

    switch(gc_block->user_mcode) {

#ifdef N_TOOLS
        case 6:
            state = Status_OK;
            break;

        case 61:
            if(bit_istrue(*value_words, bit(Word_Q))) {
                state = gc_block->values.q > 0.0f && gc_block->values.q <= (float)N_TOOLS ? Status_OK : Status_GcodeIllegalToolTableEntry;
                bit_false(*value_words, bit(Word_Q));
            }
            break;
#endif

        case 100:
            if(bit_istrue(*value_words, bit(Word_P))) {
                state = Status_OK;
                bit_false(*value_words, bit(Word_P));
            }
            break;

        case 101:
            if(bit_istrue(*value_words, bit(Word_P))) {
                state = Status_OK;
                gc_block->user_mcode_sync = true;
                bit_false(*value_words, bit(Word_P));
            }
            break;

        case 123:
            if(bit_istrue(*value_words, bit(Word_P))) {
                state = Status_OK;
                gc_block->user_mcode_sync = true;
                bit_false(*value_words, bit(Word_P));
            }
            break;

        case 124:
            if(bit_istrue(*value_words, bit(Word_P))) {
                state = Status_OK;
                gc_block->user_mcode_sync = true;
                bit_false(*value_words, bit(Word_P));
            }
            break;
    }

    return state;
}

//
// userMcodeExecute - execute user defined M-code
//

void userMCodeExecute (uint_fast16_t state, parser_block_t *gc_block) {

    switch((uint32_t)gc_block->user_mcode) {

        case 61:
            atc_tool_select((uint8_t)truncf(gc_block->values.q));
            break;

        case 100:
            // do something
            break;

        case 101:
            // do something
            break;
#if LASER_PPI
        case 123:
            laser.ppi = gc_block->values.p;
            break;

        case 124:
            laser.pulse_length = (uint_fast16_t)gc_block->values.p;
            laser_ppi_mode(laser.pulse_length != 0);
            break;
#endif
    }
}
