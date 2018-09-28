/*
  gcode.c - rs274/ngc parser.
  Part of Grbl

  Copyright (c) 2017-2018 Terje Io
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

// NOTE: Max line number is defined by the g-code standard to be 99999. It seems to be an
// arbitrary value, and some GUIs may require more. So we increased it based on a max safe
// value when converting a float (7.2 digit precision)s to an integer.
#define MAX_LINE_NUMBER 10000000
#define MAX_TOOL_NUMBER 255 // Limited by max unsigned 8-bit value

typedef enum {
    AxisCommand_None = 0,
    AxisCommand_NonModal,
    AxisCommand_MotionMode,
    AxisCommand_ToolLengthOffset,
    AxisCommand_Scaling
} axis_command_t;

// Declare gc extern struct
parser_state_t gc_state;
#ifdef N_TOOLS
tool_data_t tool_table[N_TOOLS + 1];
#else
tool_data_t tool_table;
#endif

#define FAIL(status) return(status);

struct {
    float xyz[N_AXIS];
    float ijk[3];
} scale_factor;

// Simple hypotenuse computation function.
inline float hypot_f (float x, float y)
{
    return sqrtf(x*x + y*y);
}

void reset_scaling (float factor)
{
    uint_fast8_t idx = N_AXIS;
    do {
        scale_factor.xyz[--idx] = factor;
        if(idx <= 2)
            scale_factor.ijk[idx] = factor;
    } while(idx);

    gc_state.modal.scaling_active = factor != 1.0f;
}

void gc_init()
{
    memset(&gc_state, 0, sizeof(parser_state_t));

#ifndef N_TOOLS
    memset(&tool_table, 0, sizeof(tool_table));
    gc_state.tool = &tool_table;
#else
    gc_state.tool = &tool_table[0];
#endif

    // Load default override status
    gc_state.modal.override_ctrl = sys.override_ctrl;

  #ifdef CONSTANT_SURFACE_SPEED_OPTION
    gc_state.spindle.max_rpm = settings.rpm_max;
  #endif

    reset_scaling(1.0f);

    // Load default G54 coordinate system.
    if (!settings_read_coord_data(gc_state.modal.coord_system.idx, &gc_state.modal.coord_system.xyz))
        report_status_message(Status_SettingReadFail);

}


// Set dynamic laser power mode to PPI (Pulses Per Inch)
// When active laser power is controlled by external hardware tracking motion and pulsing the laser
void gc_set_laser_ppimode (bool on)
{
	gc_state.is_laser_ppi_mode = on;
}

// Executes one block (line) of 0-terminated G-Code. The block is assumed to contain only uppercase
// characters and signed floating point values (no whitespace). Comments and block delete
// characters have been removed. In this function, all units and positions are converted and
// exported to grbl's internal functions in terms of (mm, mm/min) and absolute machine
// coordinates, respectively.
status_code_t gc_execute_block(char *block, char *message)
{
    static parser_block_t gc_block;

    // Determine if the line is a program start/end marker.
    // Old comment from protocol.c:
    // NOTE: This maybe installed to tell Grbl when a program is running vs manual input,
    // where, during a program, the system auto-cycle start will continue to execute
    // everything until the next '%' sign. This will help fix resuming issues with certain
    // functions that empty the planner buffer to execute its task on-time.
    if (block[0] == CMD_PROGRAM_DEMARCATION && block[1] == '\0') {
        gc_state.file_run = !gc_state.file_run;
        return Status_OK;
    }

  /* -------------------------------------------------------------------------------------
     STEP 1: Initialize parser block struct and copy current g-code state modes. The parser
     updates these modes and commands as the block line is parsed and will only be used and
     executed after successful error-checking. The parser block struct also contains a block
     values struct, word tracking variables, and a non-modal commands tracker for the new
     block. This struct contains all of the necessary information to execute the block. */

    memset(&gc_block, 0, sizeof(gc_block));                           // Initialize the parser block struct.
    memcpy(&gc_block.modal, &gc_state.modal, sizeof(gc_state.modal)); // Copy current modes

    axis_command_t axis_command = AxisCommand_None;
    plane_t plane;

    // Initialize bitflag tracking variables for axis indices compatible operations.
    uint8_t axis_words = 0; // XYZ tracking
    uint8_t ijk_words = 0; // IJK tracking

    // Initialize command and value words and parser flags variables.
    uint32_t command_words = 0; // Bitfield for tracking G and M command words. Also used for modal group violations.
    uint32_t value_words = 0;   // Bitfield for tracking value words.
    gc_parser_flags_t gc_parser_flags = {0};

    // Determine if the line is a jogging motion or a normal g-code block.
    if (block[0] == '$') { // NOTE: `$J=` already parsed when passed to this function.
        // Set G1 and G94 enforced modes to ensure accurate error checks.
        gc_parser_flags.jog_motion = On;
        gc_block.modal.motion = MotionMode_Linear;
        gc_block.modal.feed_mode = FeedMode_UnitsPerMin;
        gc_block.values.n = JOG_LINE_NUMBER; // Initialize default line number reported during jog.
    }

  /* -------------------------------------------------------------------------------------
     STEP 2: Import all g-code words in the block. A g-code word is a letter followed by
     a number, which can either be a 'G'/'M' command or sets/assigns a command value. Also,
     perform initial error-checks for command word modal group violations, for any repeated
     words, and for negative values set for the value words F, N, P, T, and S. */

    word_bit_t word_bit; // Bit-value for assigning tracking variables
    uint_fast8_t char_counter = gc_parser_flags.jog_motion ? 3 /* Start parsing after `$J=` */ : 0;
    char letter;
    float value;
    uint_fast8_t int_value = 0;
    uint_fast16_t mantissa = 0;

    while ((letter = block[char_counter++]) != '\0') { // Loop until no more g-code words in block.

        // Import the next g-code word, expecting a letter followed by a value. Otherwise, error out.
        if((letter < 'A') || (letter > 'Z'))
            FAIL(Status_ExpectedCommandLetter); // [Expected word letter]

        if (!read_float(block, &char_counter, &value))
            FAIL(Status_BadNumberFormat); // [Expected word value]

        // Convert values to smaller uint8 significand and mantissa values for parsing this word.
        // NOTE: Mantissa is multiplied by 100 to catch non-integer command values. This is more
        // accurate than the NIST gcode requirement of x10 when used for commands, but not quite
        // accurate enough for value words that require integers to within 0.0001. This should be
        // a good enough comprimise and catch most all non-integer errors. To make it compliant,
        // we would simply need to change the mantissa to int16, but this add compiled flash space.
        // Maybe update this later.
        int_value = (uint_fast8_t)truncf(value);
        mantissa = (uint_fast16_t)roundf(100.0f * (value - int_value)); // Compute mantissa for Gxx.x commands.
        // NOTE: Rounding must be used to catch small floating point errors.

        // Check if the g-code word is supported or errors due to modal group violations or has
        // been repeated in the g-code block. If ok, update the command or record its value.
        switch(letter) {

          /* 'G' and 'M' Command Words: Parse commands and check for modal group violations.
             NOTE: Modal group numbers are defined in Table 4 of NIST RS274-NGC v3, pg.20 */

            case 'G':
            // Determine 'G' command and its modal group
                switch(int_value) {

                    case 10: case 28: case 30: case 92:
                        // Check for G10/28/30/92 being called with G0/1/2/3/38 on same block.
                        // * G43.1 is also an axis command but is not explicitly defined this way.
                        if (mantissa == 0) { // Ignore G28.1, G30.1, and G92.1
                            if (axis_command)
                                FAIL(Status_GcodeAxisCommandConflict); // [Axis word/command conflict]
                            axis_command = AxisCommand_NonModal;
                        }
                        // No break. Continues to next line.

                    case 4: case 53:
                        word_bit.group = ModalGroup_G0;
                        gc_block.non_modal_command = (non_modal_t)int_value;
                        if ((int_value == 28) || (int_value == 30)) {
                            if (!((mantissa == 0) || (mantissa == 10)))
                                FAIL(Status_GcodeUnsupportedCommand);
                            gc_block.non_modal_command += mantissa;
                            mantissa = 0; // Set to zero to indicate valid non-integer G command.
                        } else if (int_value == 92) {
                            if (!((mantissa == 0) || (mantissa == 10) || (mantissa == 20) || (mantissa == 30)))
                                FAIL(Status_GcodeUnsupportedCommand);
                            gc_block.non_modal_command += mantissa;
                            mantissa = 0; // Set to zero to indicate valid non-integer G command.
                        }
                        break;

                    case 33:
                        if(!hal.spindle_get_data)
                            FAIL(Status_GcodeUnsupportedCommand); // [G33 not supported]
                        // No break. Continues to next line.

                    case 0: case 1: case 2: case 3: case 38:
                        // Check for G0/1/2/3/38 being called with G10/28/30/92 on same block.
                        // * G43.1 is also an axis command but is not explicitly defined this way.
                        if (axis_command)
                            FAIL(Status_GcodeAxisCommandConflict);// [Axis word/command conflict]
                        axis_command = AxisCommand_MotionMode;
                        // No break. Continues to next line.

                    case 80:
                        word_bit.group = ModalGroup_G1;
                        gc_block.modal.motion = (motion_mode_t)int_value;
                        gc_block.modal.canned_cycle_active = false;
                        if (int_value == 38) {
                            if (!((mantissa == 20) || (mantissa == 30) || (mantissa == 40) || (mantissa == 50)))
                                FAIL(Status_GcodeUnsupportedCommand); // [Unsupported G38.x command]
                            gc_block.modal.motion += (mantissa / 10) + 100;
                            mantissa = 0; // Set to zero to indicate valid non-integer G command.
                        }
                        break;

                    case 73: case 81: case 82: case 83: case 85: case 86: case 89:
                        if (axis_command)
                            FAIL(Status_GcodeAxisCommandConflict);// [Axis word/command conflict]
                        axis_command = AxisCommand_MotionMode;
                        word_bit.group = ModalGroup_G1;
                        gc_block.modal.canned_cycle_active = true;
                        gc_block.modal.motion = (motion_mode_t)int_value;
                        gc_parser_flags.canned_cycle_change = gc_block.modal.motion != gc_state.modal.motion;
                        break;

                    case 17: case 18: case 19:
                        word_bit.group = ModalGroup_G2;
                        gc_block.modal.plane_select = (plane_select_t)(int_value - 17);
                        break;

                    case 90: case 91:
                        if (mantissa == 0) {
                            word_bit.group = ModalGroup_G3;
                            gc_block.modal.distance = (distance_mode_t)(int_value - 90);
                        } else {
                            word_bit.group = ModalGroup_G4;
                            if ((mantissa != 10) || (int_value == 90))
                                FAIL(Status_GcodeUnsupportedCommand); // [G90.1 not supported]
                            mantissa = 0; // Set to zero to indicate valid non-integer G command.
                            // Otherwise, arc IJK incremental mode is default. G91.1 does nothing.
                        }
                        break;

                    case 93: case 94:
                        word_bit.group = ModalGroup_G5;
                        gc_block.modal.feed_mode = (feed_mode_t)(94 - int_value);
                        break;

                    case 95:
                        if(hal.spindle_get_data) {
                            word_bit.group = ModalGroup_G5;
                            gc_block.modal.feed_mode = FeedMode_UnitsPerRev;
                        } else
                            FAIL(Status_GcodeUnsupportedCommand); // [G95 not supported]
                        break;

                    case 20: case 21:
                        word_bit.group = ModalGroup_G6;
                        gc_block.modal.units = (units_mode_t)(21 - int_value);
                        break;

                    case 40:
                        word_bit.group = ModalGroup_G7;
                        // NOTE: Not required since cutter radius compensation is always disabled. Only here
                        // to support G40 commands that often appear in g-code program headers to setup defaults.
                        // gc_block.modal.cutter_comp = CUTTER_COMP_DISABLE; // G40
                        break;

                    case 43: case 49:
                        word_bit.group = ModalGroup_G8;
                        // NOTE: The NIST g-code standard vaguely states that when a tool length offset is changed,
                        // there cannot be any axis motion or coordinate offsets updated. Meaning G43, G43.1, and G49
                        // all are explicit axis commands, regardless if they require axis words or not.

                        if (axis_command)
                            FAIL(Status_GcodeAxisCommandConflict); // [Axis word/command conflict] }

                        axis_command = AxisCommand_ToolLengthOffset;
                        if (int_value == 49) // G49
                            gc_block.modal.tool_offset_mode = ToolLengthOffset_Cancel;
#ifdef N_TOOLS
                        else if (mantissa == 0) // G43
                            gc_block.modal.tool_offset_mode = ToolLengthOffset_Enable;
#endif
                        else if (mantissa == 10) // G43.1
                            gc_block.modal.tool_offset_mode = ToolLengthOffset_EnableDynamic;
                        else
                            FAIL(Status_GcodeUnsupportedCommand); // [Unsupported G43.x command]
                        mantissa = 0; // Set to zero to indicate valid non-integer G command.
                        break;

                    case 54: case 55: case 56: case 57: case 58: case 59:
                        // NOTE: G59.x are not supported. (But their int_values would be 60, 61, and 62.)
                        word_bit.group = ModalGroup_G12;
                        gc_block.modal.coord_system.idx = int_value - 54; // Shift to array indexing.
#if N_COORDINATE_SYSTEM > 6
                        if(int_value == 59) {
                            gc_block.modal.coord_system.idx += mantissa / 10;
                            mantissa = 0;
                        }
#endif
                        break;

                    case 61:
                        word_bit.group = ModalGroup_G13;
                        if (mantissa != 0) // [G61.1 not supported]
                            FAIL(Status_GcodeUnsupportedCommand);
                        // gc_block.modal.control = CONTROL_MODE_EXACT_PATH; // G61
                        break;

                  #ifdef CONSTANT_SURFACE_SPEED_OPTION
                    case 96: case 97:
                        word_bit.group = ModalGroup_G14;
                        gc_state.modal.spindle_rpm_mode = (spindle_rpm_mode_t)((int_value - 96) ^ 1);
                        gc_parser_flags.spindle_force_sync = gc_state.modal.spindle_rpm_mode == SpindleSpeedMode_CSS; // needed?
                        break;
                  #endif

                    case 98: case 99:
                        word_bit.group = ModalGroup_G11;
                        gc_state.canned.return_mode = (cc_return_mode_t)(int_value - 98);
                        break;

                    case 50: case 51:
                        axis_command = AxisCommand_Scaling;
                        word_bit.group = ModalGroup_G11;
                        gc_block.modal.scaling_active = int_value == 51;
                        break;

                    default: FAIL(Status_GcodeUnsupportedCommand); // [Unsupported G command]
                } // end G-value switch

                if (mantissa > 0)
                    FAIL(Status_GcodeCommandValueNotInteger); // [Unsupported or invalid Gxx.x command]
                // Check for more than one command per modal group violations in the current block
                // NOTE: Variable 'word_bit' is always assigned, if the command is valid.
                if (bit_istrue(command_words, bit(word_bit.group)))
                    FAIL(Status_GcodeModalGroupViolation);
                command_words |= bit(word_bit.group);
                break;

            case 'M': // Determine 'M' command and its modal group

                if (mantissa > 0)
                    FAIL(Status_GcodeCommandValueNotInteger); // [No Mxx.x commands]

                switch(int_value) {

                    case 0: case 1: case 2: case 30:
                        word_bit.group = ModalGroup_M4;
                        switch(int_value) {

                            case 0: // M0 - program pause
                                gc_block.modal.program_flow = ProgramFlow_Paused;
                                break;

                            case 1: // M1 - program pause
                                if(hal.driver_cap.program_stop && !hal.system_control_get_state().stop_disable)
                                    gc_block.modal.program_flow = ProgramFlow_Paused;
                                break;

                            default: // M2, M30 - program end and reset
                                gc_block.modal.program_flow = (program_flow_t)int_value;
                        }
                        break;

                    case 3: case 4: case 5:
                        word_bit.group = ModalGroup_M7;
                        gc_block.modal.spindle.on = !(int_value == 5);
                        gc_block.modal.spindle.ccw = int_value == 4;
                        break;
#ifndef N_TOOLS
                    case 6:
                        word_bit.group = ModalGroup_M6;
                        break;
#endif
                    case 7: case 8: case 9:
                        word_bit.group = ModalGroup_M8;
                        switch(int_value) {

                            case 7:
                            	if(!hal.driver_cap.mist_control)
                            		FAIL(Status_GcodeUnsupportedCommand);
                                gc_block.modal.coolant.mist = On;
                                break;

                            case 8:
                                gc_block.modal.coolant.flood = On;
                                break;

                            case 9:
                                gc_block.modal.coolant.value = 0;
                                break;
                        }
                        break;

                #ifdef ENABLE_PARKING_OVERRIDE_CONTROL
                    case 56:
                #endif
                    case 49: case 50: case 51: case 53:
                        word_bit.group = ModalGroup_M9;
                        gc_block.override_command = (override_mode_t)int_value;
                        break;

                    default:
                        if(hal.userdefined_mcode_check && (gc_block.user_defined_mcode = hal.userdefined_mcode_check(int_value)))
                            if(int_value == 6) // M6
                                word_bit.group = ModalGroup_M6;
                            else
                                gc_block.non_modal_command = NonModal_UserDefinedMCode;
                        else
                            FAIL(Status_GcodeUnsupportedCommand); // [Unsupported M command]
                } // end M-value switch

                // Check for more than one command per modal group violations in the current block
                // NOTE: Variable 'word_bit' is always assigned, if the command is valid.
                if (bit_istrue(command_words, bit(word_bit.group)))
                    FAIL(Status_GcodeModalGroupViolation);
                command_words |= bit(word_bit.group);
                break;

            // NOTE: All remaining letters assign values.
            default:

                /* Non-Command Words: This initial parsing phase only checks for repeats of the remaining
                legal g-code words and stores their value. Error-checking is performed later since some
                words (I,J,K,L,P,R) have multiple connotations and/or depend on the issued commands. */

                switch(letter) {

                  #ifdef A_AXIS
					case 'A':
						word_bit.parameter = Word_A;
						gc_block.values.xyz[A_AXIS] = value;
						bit_true(axis_words, bit(A_AXIS));
						break;
                  #endif

				  #ifdef B_AXIS
					case 'B':
						word_bit.parameter = Word_B;
						gc_block.values.xyz[B_AXIS] = value;
						bit_true(axis_words, bit(B_AXIS));
						break;
                  #endif

				  #ifdef C_AXIS
					case 'C':
						word_bit.parameter = Word_C;
						gc_block.values.xyz[C_AXIS] = value;
						bit_true(axis_words, bit(C_AXIS));
						break;
                 #endif

                    case 'D':
                        word_bit.parameter = Word_D;
                        gc_block.values.d = value;
                        break;

                    case 'F':
                        word_bit.parameter = Word_F;
                        gc_block.values.f = value;
                        break;
#ifdef N_TOOLS
                    case 'H':
                        word_bit.parameter = Word_H;
                        gc_block.values.h = int_value;
                        break;
#else
                    // case 'H': // Not supported
#endif
                    case 'I':
                        word_bit.parameter = Word_I;
                        gc_block.values.ijk[X_AXIS] = value;
                        bit_true(ijk_words, bit(X_AXIS));
                        break;

                    case 'J':
                        word_bit.parameter = Word_J;
                        gc_block.values.ijk[Y_AXIS] = value;
                        bit_true(ijk_words, bit(Y_AXIS));
                        break;

                    case 'K':
                        word_bit.parameter = Word_K;
                        gc_block.values.ijk[Z_AXIS] = value;
                        bit_true(ijk_words, bit(Z_AXIS));
                        break;

                    case 'L':
                        word_bit.parameter = Word_L;
                        gc_block.values.l = int_value;
                        break;

                    case 'N':
                        word_bit.parameter = Word_N;
                        gc_block.values.n = (int32_t)truncf(value);
                        break;

                    case 'P': // NOTE: For certain commands, P value must be an integer, but none of these commands are supported.
                        word_bit.parameter = Word_P;
                        gc_block.values.p = value;
                        break;

                    case 'Q': // may be used for user defined mcodes
                        word_bit.parameter = Word_Q;
                        gc_block.values.q = value;
                        break;

                    case 'R':
                        word_bit.parameter = Word_R;
                        gc_block.values.r = value;
                        break;

                    case 'S':
                        word_bit.parameter = Word_S;
                        gc_block.values.s = value;
                        break;

                    case 'T':
                        word_bit.parameter = Word_T;
                        if (value > MAX_TOOL_NUMBER)
                            FAIL(Status_GcodeMaxValueExceeded);
                        gc_block.values.t = int_value;
                        break;

                    case 'X':
                        word_bit.parameter = Word_X;
                        gc_block.values.xyz[X_AXIS] = value;
                        bit_true(axis_words, bit(X_AXIS));
                        break;

                    case 'Y':
                        word_bit.parameter = Word_Y;
                        gc_block.values.xyz[Y_AXIS] = value;
                        bit_true(axis_words, bit(Y_AXIS));
                        break;

                    case 'Z':
                        word_bit.parameter = Word_Z;
                        gc_block.values.xyz[Z_AXIS] = value;
                        bit_true(axis_words, bit(Z_AXIS));
                        break;

                    default: FAIL(Status_GcodeUnsupportedCommand);

                } // end parameter letter switch

                // NOTE: Variable 'word_bit' is always assigned, if the non-command letter is valid.
                if (bit_istrue(value_words, bit(word_bit.parameter)))
                    FAIL(Status_GcodeWordRepeated); // [Word repeated]

                // Check for invalid negative values for words F, H, N, P, T, and S.
                // NOTE: Negative value check is done here simply for code-efficiency.
                if ((bit(word_bit.parameter) & (bit(Word_D)|bit(Word_F)|bit(Word_H)|bit(Word_N)|bit(Word_P)|bit(Word_T)|bit(Word_S))) && value < 0.0f)
                    FAIL(Status_NegativeValue); // [Word value cannot be negative]

                value_words |= bit(word_bit.parameter); // Flag to indicate parameter assigned.

        } // end main letter switch
    }

    // Parsing complete!


  /* -------------------------------------------------------------------------------------
     STEP 3: Error-check all commands and values passed in this block. This step ensures all of
     the commands are valid for execution and follows the NIST standard as closely as possible.
     If an error is found, all commands and values in this block are dumped and will not update
     the active system g-code modes. If the block is ok, the active system g-code modes will be
     updated based on the commands of this block, and signal for it to be executed.

     Also, we have to pre-convert all of the values passed based on the modes set by the parsed
     block. There are a number of error-checks that require target information that can only be
     accurately calculated if we convert these values in conjunction with the error-checking.
     This relegates the next execution step as only updating the system g-code modes and
     performing the programmed actions in order. The execution step should not require any
     conversion calculations and would only require minimal checks necessary to execute.
  */

  /* NOTE: At this point, the g-code block has been parsed and the block line can be freed.
     NOTE: It's also possible, at some future point, to break up STEP 2, to allow piece-wise
     parsing of the block on a per-word basis, rather than the entire block. This could remove
     the need for maintaining a large string variable for the entire block and free up some memory.
     To do this, this would simply need to retain all of the data in STEP 1, such as the new block
     data struct, the modal group and value bitflag tracking variables, and axis array indices
     compatible variables. This data contains all of the information necessary to error-check the
     new g-code block when the EOL character is received. However, this would break Grbl's startup
     lines in how it currently works and would require some refactoring to make it compatible.
  */

 /*
  * Order of execution as per RS274-NGC_3 table 8:
  *
  *      1. comment (includes message)
  *      2. set feed rate mode (G93, G94 - inverse time or per minute)
  *      3. set feed rate (F)
  *      4. set spindle speed (S)
  *      5. select tool (T)
  *      6. change tool (M6)
  *      7. spindle on or off (M3, M4, M5)
  *      8. coolant on or off (M7, M8, M9)
  *      9. enable or disable overrides (M48, M49, M50, M51, M53)
  *      10. dwell (G4)
  *      11. set active plane (G17, G18, G19)
  *      12. set length units (G20, G21)
  *      13. cutter radius compensation on or off (G40, G41, G42)
  *      14. cutter length compensation on or off (G43, G49)
  *      15. coordinate system selection (G54, G55, G56, G57, G58, G59, G59.1, G59.2, G59.3)
  *      16. set path control mode (G61, G61.1, G64)
  *      17. set distance mode (G90, G91)
  *      18. set retract mode (G98, G99)
  *      19. home (G28, G30) or
  *              change coordinate system data (G10) or
  *              set axis offsets (G92, G92.1, G92.2, G94).
  *      20. perform motion (G0 to G3, G33, G80 to G89) as modified (possibly) by G53
  *      21. stop and end (M0, M1, M2, M30, M60)
  */

  // [0. Non-specific/common error-checks and miscellaneous setup]:

    // Determine implicit axis command conditions. Axis words have been passed, but no explicit axis
    // command has been sent. If so, set axis command to current motion mode.
    if (axis_words && !axis_command)
        axis_command = AxisCommand_MotionMode; // Assign implicit motion-mode

    if(gc_state.tool_change && axis_command == AxisCommand_MotionMode && !gc_parser_flags.jog_motion)
        FAIL(Status_GcodeToolChangePending); // [Motions (except jogging) not allowed when changing tool]

    // Check for valid line number N value.
    // Line number value cannot be less than zero (done) or greater than max line number.
    if (bit_istrue(value_words, bit(Word_N)) && gc_block.values.n > MAX_LINE_NUMBER)
        FAIL(Status_GcodeInvalidLineNumber); // [Exceeds max line number]

    // bit_false(value_words,bit(Word_N)); // NOTE: Single-meaning value word. Set at end of error-checking.

    // Track for unused words at the end of error-checking.
    // NOTE: Single-meaning value words are removed all at once at the end of error-checking, because
    // they are always used when present. This was done to save a few bytes of flash. For clarity, the
    // single-meaning value words may be removed as they are used. Also, axis words are treated in the
    // same way. If there is an explicit/implicit axis command, XYZ words are always used and are
    // are removed at the end of error-checking.

    // [1. Comments ]: MSG's may be supported by driver layer. Comment handling performed by protocol.

    // [2. Set feed rate mode ]: G93 F word missing with G1,G2/3 active, implicitly or explicitly. Feed rate
    //   is not defined after switching between G93, G94 and G95.
    // NOTE: For jogging, ignore prior feed rate mode. Enforce G94 and check for required F word.
    if (gc_parser_flags.jog_motion) {

        if(bit_isfalse(value_words, bit(Word_F)))
            FAIL(Status_GcodeUndefinedFeedRate);

        if (gc_block.modal.units == UnitsMode_Inches)
            gc_block.values.f *= MM_PER_INCH;

    } else if(gc_block.modal.motion == MotionMode_SpindleSynchronized) {

        if (bit_isfalse(value_words, bit(Word_K))) {
            gc_block.values.k = gc_state.distance_per_rev;
        } else {
            bit_false(value_words, bit(Word_K));
            gc_block.values.k = gc_block.modal.units == UnitsMode_Inches ? gc_block.values.ijk[Z_AXIS] *= MM_PER_INCH : gc_block.values.ijk[Z_AXIS];
        }

    } else if (gc_block.modal.feed_mode == FeedMode_InverseTime) { // = G93
        // NOTE: G38 can also operate in inverse time, but is undefined as an error. Missing F word check added here.
        if (axis_command == AxisCommand_MotionMode) {
            if (!(gc_block.modal.motion == MotionMode_None || gc_block.modal.motion == MotionMode_Seek)) {
                if (bit_isfalse(value_words, bit(Word_F)))
                    FAIL(Status_GcodeUndefinedFeedRate); // [F word missing]
            }
        }
        // NOTE: It seems redundant to check for an F word to be passed after switching from G94 to G93. We would
        // accomplish the exact same thing if the feed rate value is always reset to zero and undefined after each
        // inverse time block, since the commands that use this value already perform undefined checks. This would
        // also allow other commands, following this switch, to execute and not error out needlessly. This code is
        // combined with the above feed rate mode and the below set feed rate error-checking.

        // [3. Set feed rate ]: F is negative (done.)
        // - In inverse time mode: Always implicitly zero the feed rate value before and after block completion.
        // NOTE: If in G93 mode or switched into it from G94, just keep F value as initialized zero or passed F word
        // value in the block. If no F word is passed with a motion command that requires a feed rate, this will error
        // out in the motion modes error-checking. However, if no F word is passed with NO motion command that requires
        // a feed rate, we simply move on and the state feed rate value gets updated to zero and remains undefined.

    } else if (gc_block.modal.feed_mode == FeedMode_UnitsPerMin || gc_block.modal.feed_mode == FeedMode_UnitsPerRev) {
          // if F word passed, ensure value is in mm/min or mm/rev depending on mode, otherwise push last state value.
        if (bit_isfalse(value_words, bit(Word_F))) {
            if(gc_block.modal.feed_mode == gc_state.modal.feed_mode)
                gc_block.values.f = gc_state.feed_rate; // Push last state feed rate
            else
                FAIL(Status_GcodeUndefinedFeedRate); // [F word missing]
        } else if (gc_block.modal.units == UnitsMode_Inches)
            gc_block.values.f *= MM_PER_INCH;
    } // else, switching to G94 from G93, so don't push last state feed rate. Its undefined or the passed F word value.

    // bit_false(value_words,bit(Word_F)); // NOTE: Single-meaning value word. Set at end of error-checking.

    // [4. Set spindle speed ]: S or D is negative (done.)
#ifdef CONSTANT_SURFACE_SPEED_OPTION
    if (bit_isfalse(value_words, bit(Word_S)))
        gc_block.values.s = gc_state.modal.spindle_rpm_mode == SpindleSpeedMode_RPM ? gc_state.spindle.rpm : gc_state.spindle.surface_speed;
    else if(gc_state.modal.spindle_rpm_mode == SpindleSpeedMode_CSS) {
        gc_block.values.s *= (gc_block.modal.units == UnitsMode_Inches ? MM_PER_INCH * 12.0f : 1000.0f); // mm/min
        if (bit_istrue(command_words, bit(ModalGroup_G14)) && bit_istrue(value_words, bit(Word_D))) {
            gc_state.spindle.max_rpm = min(gc_block.values.d, settings.rpm_max);
            bit_false(value_words, bit(Word_D));
        }
    }
#else
    if (bit_isfalse(value_words, bit(Word_S)))
        gc_block.values.s = gc_state.spindle.rpm;
#endif

    // bit_false(value_words,bit(Word_S)); // NOTE: Single-meaning value word. Set at end of error-checking.

    // [5. Select tool ]: NOT SUPPORTED. Only tracks value. T is negative (done.) Not an integer. Greater than max tool value.
    // bit_false(value_words,bit(Word_T)); // NOTE: Single-meaning value word. Set at end of error-checking.

    // [6. Change tool ]:
#ifdef N_TOOLS
    if (bit_istrue(command_words, bit(ModalGroup_M6))) {
        if (bit_isfalse(value_words, bit(Word_T)))
            gc_block.values.t = gc_state.tool->tool;
        if(gc_block.values.t == 0 || gc_block.values.t > N_TOOLS)
            FAIL(Status_GcodeIllegalToolTableEntry); // or ignore?
    }
#endif
    // [7. Spindle control ]: N/A
    // [8. Coolant control ]: N/A
    // [9. Override control ]: Not supported except for a Grbl-only parking motion override control.

    if (bit_istrue(command_words, bit(ModalGroup_M9))) {

        if(bit_isfalse(value_words, bit(Word_P)))
            gc_block.values.p = 1.0f;
        else
            bit_false(value_words, bit(Word_P));

        switch(gc_block.override_command) {

            case 49:
                gc_block.modal.override_ctrl.feed_rate_disable = gc_block.values.p == 0.0f;
                gc_block.modal.override_ctrl.spindle_rpm_disable = gc_block.values.p == 0.0f;
                break;

            case 50:
                gc_block.modal.override_ctrl.feed_rate_disable = gc_block.values.p == 0.0f;
                break;

            case 51:
                gc_block.modal.override_ctrl.spindle_rpm_disable = gc_block.values.p == 0.0f;
                break;

            case 53:
                gc_block.modal.override_ctrl.feed_hold_disable = gc_block.values.p == 0.0f;
                break;

#ifdef ENABLE_PARKING_OVERRIDE_CONTROL
            case 56:
                gc_block.modal.override_ctrl.parking_disable = gc_block.values.p == 0.0f;
                break;
#endif

            default:
                break;
        }
    }

    // [10. Dwell ]: P value missing. P is negative (done.) NOTE: See below.
    if (gc_block.non_modal_command == NonModal_Dwell) {
        if (bit_isfalse(value_words, bit(Word_P)))
            FAIL(Status_GcodeValueWordMissing); // [P word missing]
        bit_false(value_words, bit(Word_P));
    }

    // [11. Set active plane ]: N/A
    switch (gc_block.modal.plane_select) {

        case PlaneSelect_XY:
            plane.axis_0 = X_AXIS;
            plane.axis_1 = Y_AXIS;
            plane.axis_linear = Z_AXIS;
            break;

        case PlaneSelect_ZX:
            plane.axis_0 = Z_AXIS;
            plane.axis_1 = X_AXIS;
            plane.axis_linear = Y_AXIS;
            break;

        default: // case PlaneSelect_YZ:
            plane.axis_0 = Y_AXIS;
            plane.axis_1 = Z_AXIS;
            plane.axis_linear = X_AXIS;
    }

    // [12. Set length units ]: N/A
    // Pre-convert XYZ coordinate values to millimeters, if applicable.
    uint_fast8_t idx = N_AXIS;
    if (gc_block.modal.units == UnitsMode_Inches) do { // Axes indices are consistent, so loop may be used.
        if (bit_istrue(axis_words, bit(--idx)))
            gc_block.values.xyz[idx] *= MM_PER_INCH;
    } while(idx);

    // Scale axis words if commanded
    if(axis_command == AxisCommand_Scaling) {

        if(gc_block.modal.scaling_active) {

            // [G50 Errors]: No axis words. TODO: add support for P (scale all with same factor)?
            if (!(axis_words || ijk_words))
                FAIL(Status_GcodeNoAxisWords); // [No axis words]

            // TODO: precheck for 0.0f and fail if found?

            gc_block.modal.scaling_active = false;

            idx = N_AXIS;
            do {
                if (bit_istrue(axis_words, bit(--idx))) {
                    scale_factor.xyz[idx] = gc_block.values.xyz[idx];
                    bit_false(axis_words, bit(idx));
                }
                gc_block.modal.scaling_active = gc_block.modal.scaling_active | (scale_factor.xyz[idx] != 1.0f);
            } while(idx);

            bit_false(value_words, AXIS_WORDS_MASK); // Remove axis words.

            idx = 3;
            do {
                if(bit_istrue(ijk_words, bit(--idx))) {
                    scale_factor.ijk[idx] = gc_block.values.ijk[idx];
                    bit_false(ijk_words, bit(idx));
                    bit_false(value_words, bit(idx)); // Remove axis words.
                }
                gc_block.modal.scaling_active = gc_block.modal.scaling_active | (scale_factor.ijk[idx] != 1.0f);
            } while(idx);

            gc_state.modal.scaling_active = gc_block.modal.scaling_active;

        } else
            reset_scaling(1.0f);
    }

    // Scale axis words if scaling active
    if(gc_state.modal.scaling_active) {
        idx = N_AXIS;
        do {
            if (bit_istrue(axis_words, bit(--idx)))
                gc_block.values.xyz[idx] *= scale_factor.xyz[idx];
        } while(idx);
    }

    // [13. Cutter radius compensation ]: G41/42 NOT SUPPORTED. Error, if enabled while G53 is active.
    // [G40 Errors]: G2/3 arc is programmed after a G40. The linear move after disabling is less than tool diameter.
    //   NOTE: Since cutter radius compensation is never enabled, these G40 errors don't apply. Grbl supports G40
    //   only for the purpose to not esrror when G40 is sent with a g-code program header to setup the default modes.

    // [14. Cutter length compensation ]: G43 NOT SUPPORTED, but G43.1 and G49 are.
    // [G43.1 Errors]: Motion command in same line.
    //   NOTE: Although not explicitly stated so, G43.1 should be applied to only one valid
    //   axis that is configured (in config.h). There should be an error if the configured axis
    //   is absent or if any of the other axis words are present.
    if (axis_command == AxisCommand_ToolLengthOffset) { // Indicates called in block.

        switch(gc_block.modal.tool_offset_mode) {

            case ToolLengthOffset_EnableDynamic:
                if (axis_words ^ bit(TOOL_LENGTH_OFFSET_AXIS))
                    FAIL(Status_GcodeG43DynamicAxisError);
                break;
#ifdef N_TOOLS
            case ToolLengthOffset_Enable:
                if (bit_istrue(value_words, bit(Word_H))) {
                    if(gc_block.values.h > N_TOOLS)
                        FAIL(Status_GcodeIllegalToolTableEntry);
                    gc_block.values.xyz[TOOL_LENGTH_OFFSET_AXIS] += tool_table[gc_block.values.h].offset[TOOL_LENGTH_OFFSET_AXIS];
                    bit_false(value_words, bit(Word_H));
                } else
                    gc_block.values.xyz[TOOL_LENGTH_OFFSET_AXIS] += gc_state.tool->offset[TOOL_LENGTH_OFFSET_AXIS];
                break;
#endif
            default:
                break;
        }
    }

    // [15. Coordinate system selection ]: *N/A. Error, if cutter radius comp is active.
    // TODO: An EEPROM read of the coordinate data may require a buffer sync when the cycle
    // is active. The read pauses the processor temporarily and may cause a rare crash. For
    // future versions on processors with enough memory, all coordinate data should be stored
    // in memory and written to EEPROM only when there is not a cycle active.

    if (bit_istrue(command_words, bit(ModalGroup_G12))) { // Check if called in block
        if (gc_block.modal.coord_system.idx > N_COORDINATE_SYSTEM)
            FAIL(Status_GcodeUnsupportedCoordSys); // [Greater than N sys]
        if (gc_state.modal.coord_system.idx != gc_block.modal.coord_system.idx && !settings_read_coord_data(gc_block.modal.coord_system.idx, &gc_block.modal.coord_system.xyz))
            FAIL(Status_SettingReadFail);
    }

    // [16. Set path control mode ]: N/A. Only G61. G61.1 and G64 NOT SUPPORTED.
    // [17. Set distance mode ]: N/A. Only G91.1. G90.1 NOT SUPPORTED.
    // [18. Set retract mode ]: N/A.

    // [19. Remaining non-modal actions ]: Check go to predefined position, set G10, or set axis offsets.
    // NOTE: We need to separate the non-modal commands that are axis word-using (G10/G28/G30/G92), as these
    // commands all treat axis words differently. G10 as absolute offsets or computes current position as
    // the axis value, G92 similarly to G10 L20, and G28/30 as an intermediate target position that observes
    // all the current coordinate system and G92 offsets.
    switch (gc_block.non_modal_command) {

        case NonModal_SetCoordinateData:

            // [G10 Errors]: L missing and is not 2 or 20. P word missing. (Negative P value done.)
            // [G10 L2 Errors]: R word NOT SUPPORTED. P value not 0 to nCoordSys(max 9). Axis words missing.
            // [G10 L20 Errors]: P must be 0 to nCoordSys(max 9). Axis words missing.

            if (!axis_words)
                FAIL(Status_GcodeNoAxisWords); // [No axis words]

            if (bit_isfalse(value_words, (bit(Word_P)|bit(Word_L))))
                FAIL(Status_GcodeValueWordMissing); // [P/L word missing]

            uint8_t p_value;

            p_value = (uint8_t)truncf(gc_block.values.p); // Convert p value to int.

            switch(gc_block.values.l) {

                case 2:
                    if (bit_istrue(value_words, bit(Word_R)))
                        FAIL(Status_GcodeUnsupportedCommand); // [G10 L2 R not supported]
                    // no break

                case 20:
                    if (p_value > N_COORDINATE_SYSTEM)
                        FAIL(Status_GcodeUnsupportedCoordSys); // [Greater than N sys]
                    // Determine coordinate system to change and try to load from EEPROM.
                    gc_block.values.coord_data.idx = p_value == 0
                                                      ? gc_block.modal.coord_system.idx // Index P0 as the active coordinate system
                                                      : (p_value - 1); // else adjust index to EEPROM coordinate data indexing.

                    if (!settings_read_coord_data(gc_block.values.coord_data.idx, &gc_block.values.coord_data.xyz))
                        FAIL(Status_SettingReadFail); // [EEPROM read fail]

                    // Pre-calculate the coordinate data changes.
                    idx = N_AXIS;
                    do { // Axes indices are consistent, so loop may be used.
                        // Update axes defined only in block. Always in machine coordinates. Can change non-active system.
                        if (bit_istrue(axis_words, bit(--idx))) {
                            if (gc_block.values.l == 20)
                                // L20: Update coordinate system axis at current position (with modifiers) with programmed value
                                // WPos = MPos - WCS - G92 - TLO  ->  WCS = MPos - G92 - TLO - WPos
                                gc_block.values.coord_data.xyz[idx] = gc_state.position[idx] - gc_state.g92_coord_offset[idx] - gc_block.values.xyz[idx] - gc_state.tool_length_offset[idx];
                            else // L2: Update coordinate system axis to programmed value.
                                gc_block.values.coord_data.xyz[idx] = gc_block.values.xyz[idx];
                        } // else, keep current stored value.
                    } while(idx);
                    break;

#ifdef N_TOOLS
                case 1: case 10: case 11:;
                    if(p_value == 0 || p_value > N_TOOLS)
                       FAIL(Status_GcodeIllegalToolTableEntry); // [Greater than N tools]

                    tool_table[p_value].tool = p_value;

                    if(bit_istrue(value_words, bit(Word_R))) {
                        tool_table[p_value].radius = gc_block.values.r;
                        bit_false(value_words, bit(Word_R));
                    }

                    float g59_3_offset[N_AXIS];
                    if(gc_block.values.l == 11 && !settings_read_coord_data(SETTING_INDEX_G59_3, &g59_3_offset))
                        FAIL(Status_SettingReadFail);

                    idx = N_AXIS;
                    do {
                        if (bit_istrue(axis_words, bit(--idx))) {
                            if(gc_block.values.l == 1)
                                tool_table[p_value].offset[idx] = gc_block.values.xyz[idx];
                            else if(gc_block.values.l == 10)
                                tool_table[p_value].offset[idx] = gc_state.position[idx] - gc_state.g92_coord_offset[idx] - gc_block.values.xyz[idx];
                            else if(gc_block.values.l == 11)
                                tool_table[p_value].offset[idx] = g59_3_offset[idx] - gc_block.values.xyz[idx];
                            if (gc_block.values.l != 1)
                                tool_table[p_value].offset[idx] -= gc_state.tool_length_offset[idx];
                        }
                        // else, keep current stored value.
                    } while(idx);

                    if(gc_block.values.l == 1)
                        settings_write_tool_data(p_value, &tool_table[p_value]);

                    break;
#endif
                default:
                    FAIL(Status_GcodeUnsupportedCommand); // [Unsupported L]
            }
            bit_false(value_words, (bit(Word_L)|bit(Word_P)));
            break;

        case NonModal_SetCoordinateOffset:

            // [G92 Errors]: No axis words.
            if (!axis_words)
                FAIL(Status_GcodeNoAxisWords); // [No axis words]

            // Update axes defined only in block. Offsets current system to defined value. Does not update when
            // active coordinate system is selected, but is still active unless G92.1 disables it.
            idx = N_AXIS;
            do { // Axes indices are consistent, so loop may be used.
                if (bit_istrue(axis_words, bit(--idx))) {
            // WPos = MPos - WCS - G92 - TLO  ->  G92 = MPos - WCS - TLO - WPos
                    gc_block.values.xyz[idx] = gc_state.position[idx] - gc_block.modal.coord_system.xyz[idx] - gc_block.values.xyz[idx] - gc_state.tool_length_offset[idx];
                } else
                    gc_block.values.xyz[idx] = gc_state.g92_coord_offset[idx];
            } while(idx);
            break;

        default:

            // At this point, the rest of the explicit axis commands treat the axis values as the traditional
            // target position with the coordinate system offsets, G92 offsets, absolute override, and distance
            // modes applied. This includes the motion mode commands. We can now pre-compute the target position.
            // NOTE: Tool offsets may be appended to these conversions when/if this feature is added.
            if (axis_words && axis_command != AxisCommand_ToolLengthOffset) { // TLO block any axis command.
                idx = N_AXIS;
                do { // Axes indices are consistent, so loop may be used to save flash space.
                    if (bit_isfalse(axis_words, bit(--idx)))
                        gc_block.values.xyz[idx] = gc_state.position[idx]; // No axis word in block. Keep same axis position.
                    else if (gc_block.non_modal_command != NonModal_AbsoluteOverride) {
                        // Update specified value according to distance mode or ignore if absolute override is active.
                        // NOTE: G53 is never active with G28/30 since they are in the same modal group.
                        // Apply coordinate offsets based on distance mode.
                        if (gc_block.modal.distance == DistanceMode_Absolute)
                            gc_block.values.xyz[idx] += gc_block.modal.coord_system.xyz[idx] + gc_state.g92_coord_offset[idx] + gc_state.tool_length_offset[idx];
                        else  // Incremental mode
                            gc_block.values.xyz[idx] += gc_state.position[idx];
                    }
                } while(idx);
            }

            // Check remaining non-modal commands for errors.
            switch (gc_block.non_modal_command) {

                case NonModal_GoHome_0: // G28
                case NonModal_GoHome_1: // G30
                    // [G28/30 Errors]: Cutter compensation is enabled.
                    // Retreive G28/30 go-home position data (in machine coordinates) from EEPROM

                    if (!settings_read_coord_data(gc_block.non_modal_command == NonModal_GoHome_0 ? SETTING_INDEX_G28 : SETTING_INDEX_G30, &gc_block.values.coord_data.xyz))
                        FAIL(Status_SettingReadFail);

                    if (axis_words) {
                        // Move only the axes specified in secondary move.
                        idx = N_AXIS;
                        do {
                            if (bit_isfalse(axis_words, bit(--idx)))
                                gc_block.values.coord_data.xyz[idx] = gc_state.position[idx];
                        } while(idx);
                    } else
                        axis_command = AxisCommand_None; // Set to none if no intermediate motion.
                    break;

                case NonModal_SetHome_0: // G28.1
                case NonModal_SetHome_1: // G30.1
                    // [G28.1/30.1 Errors]: Cutter compensation is enabled.
                    // NOTE: If axis words are passed here, they are interpreted as an implicit motion mode.
                    break;

                case NonModal_ResetCoordinateOffset:
                    // NOTE: If axis words are passed here, they are interpreted as an implicit motion mode.
                    break;

                case NonModal_AbsoluteOverride:
                    // [G53 Errors]: G0 and G1 are not active. Cutter compensation is enabled.
                    // NOTE: All explicit axis word commands are in this modal group. So no implicit check necessary.
                    if (!(gc_block.modal.motion == MotionMode_Seek || gc_block.modal.motion == MotionMode_Linear))
                        FAIL(Status_GcodeG53InvalidMotionMode); // [G53 G0/1 not active]
                    break;

                case NonModal_UserDefinedMCode:
                    if((int_value = (uint_fast8_t)hal.userdefined_mcode_validate(&gc_block, &value_words)))
                        FAIL((status_code_t)int_value);
                    break;

                default:
					break;
            }
    } // end gc_block.non_modal_command

    // [20. Motion modes ]:
    if (gc_block.modal.motion == MotionMode_None) {

        // [G80 Errors]: Axis word are programmed while G80 is active.
        // NOTE: Even non-modal commands or TLO that use axis words will throw this strict error.
        if (axis_words) // [No axis words allowed]
            FAIL(Status_GcodeAxisWordsExist);

    // Check remaining motion modes, if axis word are implicit (exist and not used by G10/28/30/92), or
    // was explicitly commanded in the g-code block.
    } else if (axis_command == AxisCommand_MotionMode) {

        if (gc_block.modal.motion == MotionMode_Seek) {
            // [G0 Errors]: Axis letter not configured or without real value (done.)
            // Axis words are optional. If missing, set axis command flag to ignore execution.
            if (!axis_words)
                axis_command = AxisCommand_None;

        // All remaining motion modes (all but G0 and G80), require a valid feed rate value. In units per mm mode,
        // the value must be positive. In inverse time mode, a positive value must be passed with each block.
        } else {

            // Check if feed rate is defined for the motion modes that require it.
            if (gc_block.modal.motion == MotionMode_SpindleSynchronized) {
                if(gc_block.values.k == 0.0f)
                    FAIL(Status_GcodeUndefinedFeedRate); // [Feed rate undefined]
            } else if (gc_block.values.f == 0.0f) // Check if feed rate is defined for the motion modes that require it.
                FAIL(Status_GcodeUndefinedFeedRate); // [Feed rate undefined]

            if (gc_block.modal.canned_cycle_active) {

                if(gc_parser_flags.canned_cycle_change) {

                    if(gc_state.modal.feed_mode == FeedMode_InverseTime)
                        FAIL(Status_InvalidStatement);

                    if(bit_isfalse(value_words, bit(Word_R)))
                        FAIL(Status_GcodeValueWordMissing);

                    if(!(axis_words & bit(plane.axis_linear)))
                        FAIL(Status_GcodeValueWordMissing);

                    gc_state.canned.dwell = 0.0f;
                    gc_state.canned.xyz[plane.axis_0] = 0.0f;
                    gc_state.canned.xyz[plane.axis_1] = 0.0f;
                    gc_state.canned.rapid_retract = On;
                    gc_state.canned.spindle_off = Off;
                    gc_state.canned.prev_position = gc_state.position[plane.axis_linear];

                }

                if(bit_isfalse(value_words, bit(Word_L)))
                    gc_block.values.l = 1;
                else if(gc_block.values.l <= 0)
                    FAIL(Status_NonPositiveValue); // [L <= 0]

                if (bit_istrue(value_words, bit(Word_R))) {
                    gc_state.canned.retract_position = gc_block.values.r;
                    if(gc_state.modal.distance == DistanceMode_Incremental)
                        gc_state.canned.retract_position += gc_state.position[plane.axis_linear];
                    gc_state.canned.retract_position = gc_block.modal.coord_system.xyz[plane.axis_linear] + gc_state.canned.retract_position;
                }

                idx = N_AXIS;
                do {
                    if(bit_istrue(axis_words, bit(--idx))) {
                        gc_state.canned.xyz[idx] = gc_block.values.xyz[idx];
                        if(idx != plane.axis_linear)
                            gc_state.canned.xyz[idx] -= gc_state.position[idx];
                    }
                } while(idx);

                if(gc_state.canned.retract_position < gc_state.canned.xyz[plane.axis_linear])
                    FAIL(Status_GcodeAxisCommandConflict);

                bit_false(value_words, (bit(Word_R)|bit(Word_L))); // Remove single-meaning value words.

                switch (gc_block.modal.motion) {

                    case MotionMode_CannedCycle86:
                    case MotionMode_CannedCycle89:
                        gc_state.canned.spindle_off = gc_block.modal.motion == MotionMode_CannedCycle86;
                        gc_state.canned.rapid_retract = gc_block.modal.motion == MotionMode_CannedCycle86;
                        // no break

                    case MotionMode_CannedCycle82:
                        if(bit_istrue(value_words, bit(Word_P))) {
                            if(gc_block.values.p < 0.0f)
                                FAIL(Status_NegativeValue);
                            gc_state.canned.dwell = gc_block.values.p;
                            bit_false(value_words, bit(Word_P)); // Remove single-meaning value word.
                        } else if(gc_parser_flags.canned_cycle_change)
                            FAIL(Status_GcodeValueWordMissing);
                        // no break

                    case MotionMode_CannedCycle85:
                    case MotionMode_CannedCycle81:
                        gc_state.canned.delta = - gc_state.canned.xyz[plane.axis_linear] + gc_state.canned.retract_position;
                        if(gc_block.modal.motion == MotionMode_CannedCycle85)
                            gc_state.canned.rapid_retract = Off;
                        break;

                    case MotionMode_DrillChipBreak:
                    case MotionMode_CannedCycle83:
                        if(bit_istrue(value_words, bit(Word_Q))) {
                            if(gc_block.values.q <= 0.0f)
                                FAIL(Status_NegativeValue); // [Q <= 0]
                            gc_state.canned.delta = gc_block.values.q;
                            bit_false(value_words, bit(Word_Q)); // Remove single-meaning value word.
                        } else if(gc_parser_flags.canned_cycle_change)
                            FAIL(Status_GcodeValueWordMissing);
                        gc_state.canned.dwell = 0.25f;
                        break;

                    default:
                        break;

                } // end switch gc_state.canned.motion

            } else switch (gc_block.modal.motion) {

                case MotionMode_Linear:
                    // [G1 Errors]: Feed rate undefined. Axis letter not configured or without real value.
                    // Axis words are optional. If missing, set axis command flag to ignore execution.
                    if (!axis_words)
                        axis_command = AxisCommand_None;
                    break;

                case MotionMode_CwArc:
                    gc_parser_flags.arc_is_clockwise = On;
                    // No break intentional.

                case MotionMode_CcwArc:
                    // [G2/3 Errors All-Modes]: Feed rate undefined.
                    // [G2/3 Radius-Mode Errors]: No axis words in selected plane. Target point is same as current.
                    // [G2/3 Offset-Mode Errors]: No axis words and/or offsets in selected plane. The radius to the current
                    //   point and the radius to the target point differs more than 0.002mm (EMC def. 0.5mm OR 0.005mm and 0.1% radius).
                    // [G2/3 Full-Circle-Mode Errors]: NOT SUPPORTED. Axis words exist. No offsets programmed. P must be an integer.
                    // NOTE: Both radius and offsets are required for arc tracing and are pre-computed with the error-checking.

                    if (!axis_words)
                        FAIL(Status_GcodeNoAxisWords); // [No axis words]

                    if (!(axis_words & (bit(plane.axis_0)|bit(plane.axis_1))))
                        FAIL(Status_GcodeNoAxisWordsInPlane); // [No axis words in plane]

                    // Calculate the change in position along each selected axis
                    float x, y;
                    x = gc_block.values.xyz[plane.axis_0] - gc_state.position[plane.axis_0]; // Delta x between current position and target
                    y = gc_block.values.xyz[plane.axis_1] - gc_state.position[plane.axis_1]; // Delta y between current position and target

                    if (value_words & bit(Word_R)) { // Arc Radius Mode

                        bit_false(value_words, bit(Word_R));

                        if (isequal_position_vector(gc_state.position, gc_block.values.xyz))
                            FAIL(Status_GcodeInvalidTarget); // [Invalid target]

                        // Convert radius value to proper units.
                        if (gc_block.modal.units == UnitsMode_Inches)
                            gc_block.values.r *= MM_PER_INCH;

                        /*  We need to calculate the center of the circle that has the designated radius and passes
                             through both the current position and the target position. This method calculates the following
                             set of equations where [x,y] is the vector from current to target position, d == magnitude of
                             that vector, h == hypotenuse of the triangle formed by the radius of the circle, the distance to
                             the center of the travel vector. A vector perpendicular to the travel vector [-y,x] is scaled to the
                             length of h [-y/d*h, x/d*h] and added to the center of the travel vector [x/2,y/2] to form the new point
                             [i,j] at [x/2-y/d*h, y/2+x/d*h] which will be the center of our arc.

                             d^2 == x^2 + y^2
                             h^2 == r^2 - (d/2)^2
                             i == x/2 - y/d*h
                             j == y/2 + x/d*h

                                                                                  O <- [i,j]
                                                                               -  |
                                                                     r      -     |
                                                                         -        |
                                                                      -           | h
                                                                   -              |
                                                     [0,0] ->  C -----------------+--------------- T  <- [x,y]
                                                               | <------ d/2 ---->|

                             C - Current position
                             T - Target position
                             O - center of circle that pass through both C and T
                             d - distance from C to T
                             r - designated radius
                             h - distance from center of CT to O

                             Expanding the equations:

                             d -> sqrt(x^2 + y^2)
                             h -> sqrt(4 * r^2 - x^2 - y^2)/2
                             i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2
                             j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2

                             Which can be written:

                             i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
                             j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2

                             Which we for size and speed reasons optimize to:

                             h_x2_div_d = sqrt(4 * r^2 - x^2 - y^2)/sqrt(x^2 + y^2)
                             i = (x - (y * h_x2_div_d))/2
                             j = (y + (x * h_x2_div_d))/2
                         */

                        // First, use h_x2_div_d to compute 4*h^2 to check if it is negative or r is smaller
                        // than d. If so, the sqrt of a negative number is complex and error out.
                        float h_x2_div_d = 4.0f * gc_block.values.r * gc_block.values.r - x * x - y * y;

                        if (h_x2_div_d < 0.0f)
                            FAIL(Status_GcodeArcRadiusError); // [Arc radius error]

                        // Finish computing h_x2_div_d.
                        h_x2_div_d = -sqrtf(h_x2_div_d) / hypot_f(x, y); // == -(h * 2 / d)

                        // Invert the sign of h_x2_div_d if the circle is counter clockwise (see sketch below)
                        if (gc_block.modal.motion == MotionMode_CcwArc)
                            h_x2_div_d = -h_x2_div_d;

                        /* The counter clockwise circle lies to the left of the target direction. When offset is positive,
                           the left hand circle will be generated - when it is negative the right hand circle is generated.

                                                                               T  <-- Target position

                                                                               ^
                                    Clockwise circles with this center         |          Clockwise circles with this center will have
                                    will have > 180 deg of angular travel      |          < 180 deg of angular travel, which is a good thing!
                                                                     \         |          /
                        center of arc when h_x2_div_d is positive ->  x <----- | -----> x <- center of arc when h_x2_div_d is negative
                                                                               |
                                                                               |

                                                                               C  <-- Current position
                        */
                        // Negative R is g-code-alese for "I want a circle with more than 180 degrees of travel" (go figure!),
                        // even though it is advised against ever generating such circles in a single line of g-code. By
                        // inverting the sign of h_x2_div_d the center of the circles is placed on the opposite side of the line of
                        // travel and thus we get the unadvisably long arcs as prescribed.
                        if (gc_block.values.r < 0.0f) {
                            h_x2_div_d = -h_x2_div_d;
                            gc_block.values.r = -gc_block.values.r; // Finished with r. Set to positive for mc_arc
                        }
                        // Complete the operation by calculating the actual center of the arc
                        gc_block.values.ijk[plane.axis_0] = 0.5f * (x - (y * h_x2_div_d));
                        gc_block.values.ijk[plane.axis_1] = 0.5f * (y + (x * h_x2_div_d));

                    } else { // Arc Center Format Offset Mode

                        if (!(ijk_words & (bit(plane.axis_0)|bit(plane.axis_1))))
                            FAIL(Status_GcodeNoOffsetsInPlane);// [No offsets in plane]

                        bit_false(value_words, (bit(Word_I)|bit(Word_J)|bit(Word_K)));

                        // Convert IJK values to proper units.
                        if (gc_block.modal.units == UnitsMode_Inches) {
                            idx = 3;
                            do { // Axes indices are consistent, so loop may be used to save flash space.
                                if (ijk_words & bit(--idx))
                                    gc_block.values.ijk[idx] *= MM_PER_INCH;
                            } while(idx);
                        }

                        // Scale values if scaling active
                        if(gc_state.modal.scaling_active) {
                            idx = 3;
                            do {
                                if (ijk_words & bit(--idx))
                                    gc_block.values.ijk[idx] *= scale_factor.ijk[idx];
                            } while(idx);
                        }

                        // Arc radius from center to target
                        x -= gc_block.values.ijk[plane.axis_0]; // Delta x between circle center and target
                        y -= gc_block.values.ijk[plane.axis_1]; // Delta y between circle center and target
                        float target_r = hypot_f(x, y);

                        // Compute arc radius for mc_arc. Defined from current location to center.
                        gc_block.values.r = hypot_f(gc_block.values.ijk[plane.axis_0], gc_block.values.ijk[plane.axis_1]);

                        // Compute difference between current location and target radii for final error-checks.
                        float delta_r = fabsf(target_r - gc_block.values.r);
                        if (delta_r > 0.005f) {
                            if (delta_r > 0.5f)
                                FAIL(Status_GcodeInvalidTarget); // [Arc definition error] > 0.5mm
                            if (delta_r > (0.001f * gc_block.values.r))
                                FAIL(Status_GcodeInvalidTarget); // [Arc definition error] > 0.005mm AND 0.1% radius
                        }
                    }
                    break;

                case MotionMode_ProbeTowardNoError:
                case MotionMode_ProbeAwayNoError:
                    gc_parser_flags.probe_is_no_error = On;
                    // No break intentional.

                case MotionMode_ProbeToward:
                case MotionMode_ProbeAway:
                    if(gc_block.modal.motion == MotionMode_ProbeAway || gc_block.modal.motion == MotionMode_ProbeAwayNoError)
                        gc_parser_flags.probe_is_away = On;
                    // [G38 Errors]: Target is same current. No axis words. Cutter compensation is enabled. Feed rate
                    //   is undefined. Probe is triggered. NOTE: Probe check moved to probe cycle. Instead of returning
                    //   an error, it issues an alarm to prevent further motion to the probe. It's also done there to
                    //   allow the planner buffer to empty and move off the probe trigger before another probing cycle.
                    if (!axis_words)
                        FAIL(Status_GcodeNoAxisWords); // [No axis words]
                    if (isequal_position_vector(gc_state.position, gc_block.values.xyz))
                        FAIL(Status_GcodeInvalidTarget); // [Invalid target]
                    break;

                default:
					break;

            } // end switch gc_block.modal.motion
        }
    }

    // [21. Program flow ]: No error checks required.

    // [0. Non-specific error-checks]: Complete unused value words check, i.e. IJK used when in arc
    // radius mode, or axis words that aren't used in the block.
    if (gc_parser_flags.jog_motion) // Jogging only uses the F feed rate and XYZ value words. N is valid, but S and T are invalid.
        bit_false(value_words, (bit(Word_N)|bit(Word_F)));
    else
        bit_false(value_words, (bit(Word_N)|bit(Word_F)|bit(Word_S)|bit(Word_T))); // Remove single-meaning value words.

    if (axis_command)
        bit_false(value_words, AXIS_WORDS_MASK); // Remove axis words.

    if (value_words)
        FAIL(Status_GcodeUnusedWords); // [Unused words]

    /* -------------------------------------------------------------------------------------
     STEP 4: EXECUTE!!
     Assumes that all error-checking has been completed and no failure modes exist. We just
     need to update the state and execute the block according to the order-of-execution.
    */

    // Initialize planner data struct for motion blocks.
    plan_line_data_t plan_data;
    memset(&plan_data, 0, sizeof(plan_line_data_t)); // Zero plan_data struct

    // Intercept jog commands and complete error checking for valid jog commands and execute.
    // NOTE: G-code parser state is not updated, except the position to ensure sequential jog
    // targets are computed correctly. The final parser position after a jog is updated in
    // protocol_execute_realtime() when jogging completes or is canceled.
    if (gc_parser_flags.jog_motion) {

        // Only distance and unit modal commands and G53 absolute override command are allowed.
        // NOTE: Feed rate word and axis word checks have already been performed in STEP 3.
        if (command_words & ~(bit(ModalGroup_G3)|bit(ModalGroup_G6|bit(ModalGroup_G0))))
            FAIL(Status_InvalidJogCommand);

        if (!(gc_block.non_modal_command == NonModal_AbsoluteOverride || gc_block.non_modal_command == NonModal_NoAction))
            FAIL(Status_InvalidJogCommand);

        // Initialize planner data to current spindle and coolant modal state.
        memcpy(&plan_data.spindle, &gc_state.spindle, sizeof(spindle_t));
        plan_data.condition.spindle = gc_state.modal.spindle;
        plan_data.condition.coolant = gc_state.modal.coolant;
        plan_data.condition.is_rpm_rate_adjusted = gc_state.is_rpm_rate_adjusted;

        if ((status_code_t)(int_value = (uint8_t)jog_execute(&plan_data, &gc_block)) == Status_OK)
            memcpy(gc_state.position, gc_block.values.xyz, sizeof(gc_state.position));

        return (status_code_t)int_value;
    }

    // If in laser mode, setup laser power based on current and past parser conditions.
    if (settings.flags.laser_mode) {

        if (!((gc_block.modal.motion == MotionMode_Linear) || (gc_block.modal.motion == MotionMode_CwArc) || (gc_block.modal.motion == MotionMode_CcwArc)))
          gc_parser_flags.laser_disable = On;

        // Any motion mode with axis words is allowed to be passed from a spindle speed update.
        // NOTE: G1 and G0 without axis words sets axis_command to none. G28/30 are intentionally omitted.
        // TODO: Check sync conditions for M3 enabled motions that don't enter the planner. (zero length).
        if (axis_words && (axis_command == AxisCommand_MotionMode))
            gc_parser_flags.laser_is_motion = On;
        else if (gc_state.modal.spindle.on && !gc_state.modal.spindle.ccw) {
            // M3 constant power laser requires planner syncs to update the laser when changing between
            // a G1/2/3 motion mode state and vice versa when there is no motion in the line.
            if ((gc_state.modal.motion == MotionMode_Linear) || (gc_state.modal.motion == MotionMode_CwArc) || (gc_state.modal.motion == MotionMode_CcwArc)) {
                if (gc_parser_flags.laser_disable)
                    gc_parser_flags.spindle_force_sync = On; // Change from G1/2/3 motion mode.
            } else if (!gc_parser_flags.laser_disable) // When changing to a G1 motion mode without axis words from a non-G1/2/3 motion mode.
                gc_parser_flags.spindle_force_sync = On;
        }

        gc_state.is_rpm_rate_adjusted = gc_state.modal.spindle.ccw && !gc_parser_flags.laser_disable;

    }

    // [0. Non-specific/common error-checks and miscellaneous setup]:
    // NOTE: If no line number is present, the value is zero.
    gc_state.line_number = gc_block.values.n;
    plan_data.line_number = gc_state.line_number; // Record data for planner use.

    // [1. Comments feedback ]: Extracted in protocol.c if HAL entry point provided
    if(message && (plan_data.message = malloc(strlen(message) + 1)))
        strcpy(plan_data.message, message);

    // [2. Set feed rate mode ]:
    gc_state.modal.feed_mode = gc_block.modal.feed_mode;
    if (gc_state.modal.feed_mode == FeedMode_InverseTime)
        plan_data.condition.inverse_time = On; // Set condition flag for planner use.

    // [3. Set feed rate ]:
    gc_state.feed_rate = gc_block.values.f; // Always copy this value. See feed rate error-checking.
    plan_data.feed_rate = gc_state.feed_rate; // Record data for planner use.

    // [4. Set spindle speed ]:
  #ifdef CONSTANT_SURFACE_SPEED_OPTION
    plan_data.condition.is_rpm_pos_adjusted = gc_state.modal.spindle_rpm_mode == SpindleSpeedMode_CSS;
    if(gc_state.modal.spindle_rpm_mode == SpindleSpeedMode_CSS) {
        gc_state.spindle.axis = plane.axis_0;
        gc_state.spindle.surface_speed = plan_data.spindle.surface_speed = gc_block.values.s;
        gc_block.values.s = min(gc_state.spindle.max_rpm, gc_state.spindle.surface_speed / (gc_state.position[gc_state.spindle.axis] * 2.0f * M_PI));
    }
  #endif
    if ((gc_state.spindle.rpm != gc_block.values.s) || gc_parser_flags.spindle_force_sync) {
        if (gc_state.modal.spindle.on && !gc_parser_flags.laser_is_motion)
            spindle_sync(gc_state.modal.spindle, gc_parser_flags.laser_disable ? 0.0f : gc_block.values.s);
        gc_state.spindle.rpm = gc_block.values.s; // Update spindle speed state.
    }

    // NOTE: Pass zero spindle speed for all restricted laser motions.
    if (!gc_parser_flags.laser_disable)
        memcpy(&plan_data.spindle, &gc_state.spindle, sizeof(spindle_t)); // Record data for planner use.
    // else { plan_data.spindle.speed = 0.0; } // Initialized as zero already.

    // [5. Select tool ]: NOT SUPPORTED. Only tracks tool value.
#ifdef N_TOOLS
    gc_state.tool = &tool_table[gc_block.values.t < N_TOOLS ? gc_block.values.t : 0];
#endif
    gc_state.tool->tool = gc_block.values.t;
    if(gc_state.modal.tool_offset_mode == ToolLengthOffset_EnableDynamic) {
        idx = N_AXIS;
        do {
            idx--;
            gc_state.tool_length_offset[idx] = gc_state.tool->offset[idx];
        } while(idx);
    }
    if(hal.tool_select)  // NOTE: M6 needs to be implemented as a user-defined M-code
        hal.tool_select(gc_state.tool);

    // [6. Change tool ]: Delegated to (possible) driver implementation
    if (bit_istrue(command_words, bit(ModalGroup_M6))) {
        protocol_buffer_synchronize();
        gc_state.tool_change = true;
#ifdef N_TOOLS
        hal.tool_change(&gc_state);
        gc_state.tool_change = false;
#else
// TODO: see if it is possible somehow to allow jogging when state is suspended (not easy!)...
//        system_set_exec_state_flag(EXEC_TOOL_CHANGE); // Enter program pause for manual tool change.
//        protocol_execute_realtime(); // Execute suspend.
#endif
    }

    // [7. Spindle control ]:
    if (gc_state.modal.spindle.value != gc_block.modal.spindle.value) {
        // Update spindle control and apply spindle speed when enabling it in this block.
        // NOTE: All spindle state changes are synced, even in laser mode. Also, plan_data,
        // rather than gc_state, is used to manage laser state for non-laser motions.
        spindle_sync(gc_block.modal.spindle, plan_data.spindle.rpm);
        gc_state.modal.spindle = gc_block.modal.spindle;
    }

    plan_data.condition.spindle = gc_state.modal.spindle; // Set condition flag for planner use.
    plan_data.condition.is_rpm_rate_adjusted = gc_state.is_rpm_rate_adjusted;
    plan_data.condition.is_laser_ppi_mode = gc_state.is_rpm_rate_adjusted && gc_state.is_laser_ppi_mode;

    // [8. Coolant control ]:
    if (gc_state.modal.coolant.value != gc_block.modal.coolant.value) {
    // NOTE: Coolant M-codes are modal. Only one command per line is allowed. But, multiple states
    // can exist at the same time, while coolant disable clears all states.
        coolant_sync(gc_block.modal.coolant);
        gc_state.modal.coolant = gc_block.modal.coolant;
    }

    plan_data.condition.coolant = gc_state.modal.coolant; // Set condition flag for planner use.

    // [9. Override control ]:

    if (gc_state.modal.override_ctrl.value != gc_block.modal.override_ctrl.value) {
        gc_state.modal.override_ctrl = gc_block.modal.override_ctrl;

        if(gc_state.modal.override_ctrl.feed_rate_disable)
            plan_feed_override(0, 0);

        if(gc_state.modal.override_ctrl.spindle_rpm_disable)
            spindle_set_override(0);

        mc_override_ctrl_update(gc_state.modal.override_ctrl); // NOTE: must be called last!
    }

    // [10. Dwell ]:
    if (gc_block.non_modal_command == NonModal_Dwell)
        mc_dwell(gc_block.values.p);

    // [11. Set active plane ]:
    gc_state.modal.plane_select = gc_block.modal.plane_select;

    // [12. Set length units ]:
    gc_state.modal.units = gc_block.modal.units;

    // [13. Cutter radius compensation ]: G41/42 NOT SUPPORTED
    // gc_state.modal.cutter_comp = gc_block.modal.cutter_comp; // NOTE: Not needed since always disabled.

    // [14. Cutter length compensation ]: G43, G43.1 and G49 supported. G43 supported when N_TOOLS > 0.
    // NOTE: If G43 were supported, its operation wouldn't be any different from G43.1 in terms
    // of execution. The error-checking step would simply load the offset value into the correct
    // axis of the block XYZ value array.
    if (axis_command == AxisCommand_ToolLengthOffset ) { // Indicates a change.
        bool tlo_changed = false;
        gc_state.modal.tool_offset_mode = gc_block.modal.tool_offset_mode;
        if (gc_state.modal.tool_offset_mode == ToolLengthOffset_Cancel) {// G49
            idx = N_AXIS;
            do {
                gc_block.values.xyz[--idx] = 0.0f;
            } while(idx);
        }
        // else G43.1
        idx = N_AXIS;
        do {
            idx--;
            if (gc_state.tool_length_offset[idx] != gc_block.values.xyz[idx]) {
                tlo_changed = true;
                gc_state.tool_length_offset[idx] = gc_block.values.xyz[idx];
            }
        } while(idx);

        if(tlo_changed)
            system_flag_wco_change();
    }

    // [15. Coordinate system selection ]:
    if (gc_state.modal.coord_system.idx != gc_block.modal.coord_system.idx) {
        memcpy(&gc_state.modal.coord_system, &gc_block.modal.coord_system, sizeof(gc_state.modal.coord_system));
        system_flag_wco_change();
    }

    // [16. Set path control mode ]: G61.1/G64 NOT SUPPORTED
    // gc_state.modal.control = gc_block.modal.control; // NOTE: Always default.

    // [17. Set distance mode ]:
    gc_state.modal.distance = gc_block.modal.distance;

    // [18. Set retract mode ]: NOT N/A

    // [19. Go to predefined position, Set G10, or Set axis offsets ]:
    switch(gc_block.non_modal_command) {

        case NonModal_SetCoordinateData:
            settings_write_coord_data(gc_block.values.coord_data.idx, &gc_block.values.coord_data.xyz);
            // Update system coordinate system if currently active.
            if (gc_state.modal.coord_system.idx == gc_block.values.coord_data.idx) {
                memcpy(gc_state.modal.coord_system.xyz, gc_block.values.coord_data.xyz, sizeof(gc_state.modal.coord_system.xyz));
                system_flag_wco_change();
            }
            break;

        case NonModal_GoHome_0:
        case NonModal_GoHome_1:
            // Move to intermediate position before going home. Obeys current coordinate system and offsets
            // and absolute and incremental modes.
            plan_data.condition.rapid_motion = On; // Set rapid motion condition flag.
            if (axis_command)
                mc_line(gc_block.values.xyz, &plan_data);
            mc_line(gc_block.values.coord_data.xyz, &plan_data);
            memcpy(gc_state.position, gc_block.values.coord_data.xyz, sizeof(gc_state.position));
            reset_scaling(1.0f);
            break;

        case NonModal_SetHome_0:
            settings_write_coord_data(SETTING_INDEX_G28, &gc_state.position);
            break;

        case NonModal_SetHome_1:
            settings_write_coord_data(SETTING_INDEX_G30, &gc_state.position);
            break;

        case NonModal_SetCoordinateOffset: // G92
            memcpy(gc_state.g92_coord_offset, gc_block.values.xyz, sizeof(gc_state.g92_coord_offset));
            system_flag_wco_change();
            break;

        case NonModal_ResetCoordinateOffset: // G92.1
            clear_vector(gc_state.g92_coord_offset); // Disable G92 offsets by zeroing offset vector.
            settings_write_coord_data(SETTING_INDEX_G92, &gc_state.g92_coord_offset); // Save G92 offsets to EEPROM
            system_flag_wco_change();
            break;

        case NonModal_ClearCoordinateOffset: // G92.2
            clear_vector(gc_state.g92_coord_offset); // Disable G92 offsets by zeroing offset vector.
            system_flag_wco_change();
            break;

        case NonModal_RestoreCoordinateOffset: // G92.3
            settings_read_coord_data(SETTING_INDEX_G92, &gc_state.g92_coord_offset); // Restore G92 offsets from EEPROM
            system_flag_wco_change();
            break;

        default:
			break;
    }

    // [20. Motion modes ]:
    // NOTE: Commands G10,G28,G30,G92 lock out and prevent axis words from use in motion modes.
    // Enter motion modes only if there are axis words or a motion mode command word in the block.
    gc_state.modal.motion = gc_block.modal.motion;
    gc_state.modal.canned_cycle_active = gc_block.modal.canned_cycle_active;

    if (gc_state.modal.motion != MotionMode_None) {

        if (axis_command == AxisCommand_MotionMode) {

            pos_update_t gc_update_pos = GCUpdatePos_Target;

            switch(gc_state.modal.motion) {

                case MotionMode_Linear:
                    if(gc_state.modal.feed_mode == FeedMode_UnitsPerRev) {
                        plan_data.condition.spindle.synchronized = On;
                    //??    gc_state.distance_per_rev = plan_data.feed_rate;
                        // check initial feed rate - fail if zero?
                    }
                    mc_line(gc_block.values.xyz, &plan_data);
                    break;

                case MotionMode_Seek:
                    plan_data.condition.rapid_motion = On; // Set rapid motion condition flag.
                    mc_line(gc_block.values.xyz, &plan_data);
                    break;

                case MotionMode_CwArc:
                case MotionMode_CcwArc:
                    // fail if spindle synchronized motion?
                    mc_arc(gc_block.values.xyz, &plan_data, gc_state.position, gc_block.values.ijk, gc_block.values.r,
                            plane, gc_parser_flags.arc_is_clockwise);
                    break;

                case MotionMode_SpindleSynchronized:
                    plan_data.condition.spindle.synchronized = On;
                    plan_data.condition.inverse_time = Off;
                    plan_data.feed_rate = gc_state.distance_per_rev = gc_block.values.k;
                    // TODO: need for gc_state.distance_per_rev to be reset on modal change?
                    gc_block.values.k = hal.spindle_get_data(SpindleData_RPM).rpm;
                    gc_block.values.k = plan_data.feed_rate * gc_block.values.k;
                    if(gc_block.values.k == 0.0f)
                        FAIL(Status_GcodeUndefinedFeedRate); // [Feed rate undefined - spindle not running]
                    if(gc_block.values.k > settings.max_rate[plane.axis_linear])
                        FAIL(Status_GcodeUndefinedFeedRate); // [Feed rate too high]
                    mc_dwell(0.01f); // Needed for now since initial spindle sync is done just before st_wake_up
                    mc_line(gc_block.values.xyz, &plan_data);
                    break;

                case MotionMode_DrillChipBreak:
                case MotionMode_CannedCycle81:
                case MotionMode_CannedCycle82:
                case MotionMode_CannedCycle83:;
                    plan_data.spindle.rpm = gc_block.values.s;
                    mc_canned_drill(gc_state.modal.motion, gc_block.values.xyz, &plan_data, gc_state.position, plane, gc_block.values.l, &gc_state.canned);
                    break;

                default:
                    // NOTE: gc_block.values.xyz is returned from mc_probe_cycle with the updated position value. So
                    // upon a successful probing cycle, the machine position and the returned value should be the same.
                  #ifndef ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES
                    plan_data.condition.no_feed_override = On;
                  #endif
                    gc_update_pos = (pos_update_t)mc_probe_cycle(gc_block.values.xyz, &plan_data, gc_parser_flags);
                    break;
            }

            // As far as the parser is concerned, the position is now == target. In reality the
            // motion control system might still be processing the action and the real tool position
            // in any intermediate location.
            if (gc_update_pos == GCUpdatePos_Target)
                memcpy(gc_state.position, gc_block.values.xyz, sizeof(gc_state.position)); // gc_state.position[] = gc_block.values.xyz[]
            else if (gc_update_pos == GCUpdatePos_System)
                gc_sync_position(); // gc_state.position[] = sys_position
            // == GCUpdatePos_None
        }
    }

    if(plan_data.message)
        protocol_message(plan_data.message);

    // [21. Program flow ]:
    // M0,M1,M2,M30: Perform non-running program flow actions. During a program pause, the buffer may
    // refill and can only be resumed by the cycle start run-time command.
    gc_state.modal.program_flow = gc_block.modal.program_flow;

    if (gc_state.modal.program_flow) {

        protocol_buffer_synchronize(); // Sync and finish all remaining buffered motions before moving on.
        if (gc_state.modal.program_flow == ProgramFlow_Paused) {
            if (sys.state != STATE_CHECK_MODE) {
                system_set_exec_state_flag(EXEC_FEED_HOLD); // Use feed hold for program pause.
                protocol_execute_realtime(); // Execute suspend.
            }
        } else { // == ProgramFlow_Completed
            // Upon program complete, only a subset of g-codes reset to certain defaults, according to
            // LinuxCNC's program end descriptions and testing. Only modal groups [G-code 1,2,3,5,7,12]
            // and [M-code 7,8,9] reset to [G1,G17,G90,G94,G40,G54,M5,M9,M48]. The remaining modal groups
            // [G-code 4,6,8,10,13,14,15] and [M-code 4,5,6] and the modal words [F,S,T,H] do not reset.
            gc_state.file_run = false;
            gc_state.modal.motion = MotionMode_Linear;
            gc_block.modal.canned_cycle_active = false;
            gc_state.modal.plane_select = PlaneSelect_XY;
            gc_state.modal.distance = DistanceMode_Absolute;
            gc_state.modal.feed_mode = FeedMode_UnitsPerMin;
// TODO: check           gc_state.distance_per_rev = 0.0f;
            // gc_state.modal.cutter_comp = CUTTER_COMP_DISABLE; // Not supported.
            gc_state.modal.coord_system.idx = 0; // G54
            gc_state.modal.spindle.value = 0;
            gc_state.modal.coolant.value = 0;
            gc_state.modal.override_ctrl.feed_rate_disable = Off;
            gc_state.modal.override_ctrl.spindle_rpm_disable = Off;
          #ifdef ENABLE_PARKING_OVERRIDE_CONTROL
           #ifdef DEACTIVATE_PARKING_UPON_INIT
            gc_state.modal.override_ctrl.parking_disable = On;
           #else
            gc_state.modal.override_ctrl.parking_disable = Off;
           #endif
          #endif
            sys.override_ctrl = gc_state.modal.override_ctrl;

          #ifdef RESTORE_OVERRIDES_AFTER_PROGRAM_END
            sys.f_override = DEFAULT_FEED_OVERRIDE;
            sys.r_override = DEFAULT_RAPID_OVERRIDE;
            sys.spindle_rpm_ovr = DEFAULT_SPINDLE_RPM_OVERRIDE;
          #endif

            // Execute coordinate change and spindle/coolant stop.
            if (sys.state != STATE_CHECK_MODE) {

                float g92_offset_stored[N_AXIS];
                if(settings_read_coord_data(SETTING_INDEX_G92, &g92_offset_stored) && !isequal_position_vector(g92_offset_stored, gc_state.g92_coord_offset))
                    settings_write_coord_data(SETTING_INDEX_G92, &gc_state.g92_coord_offset); // Save G92 offsets to EEPROM

                if (!(settings_read_coord_data(gc_state.modal.coord_system.idx, &gc_state.modal.coord_system.xyz)))
                    FAIL(Status_SettingReadFail);
                system_flag_wco_change(); // Set to refresh immediately just in case something altered.
                spindle_stop();
                hal.coolant_set_state((coolant_state_t){0});
            }
            report_feedback_message(Message_ProgramEnd);
        }
        gc_state.modal.program_flow = ProgramFlow_Running; // Reset program flow.
    }

    if(gc_block.non_modal_command == NonModal_UserDefinedMCode && sys.state != STATE_CHECK_MODE) {

        if(gc_block.user_defined_mcode_sync)
            protocol_buffer_synchronize(); // Ensure user defined mcode is executed when specified in program.

        hal.userdefined_mcode_execute(sys.state, &gc_block);

    }

    // TODO: % to denote start of program.

    return Status_OK;
}

/*
  Not supported:

  - Canned cycles
  - Tool radius compensation
  - A,B,C-axes
  - Evaluation of expressions
  - Variables
  - Override control (TBD)
  - Tool changes
  - Switches

   (*) Indicates optional parameter, enabled through config.h and re-compile
   group 0 = {G92.2, G92.3} (Non modal: Cancel and re-enable G92 offsets)
   group 1 = {G81 - G89} (Motion modes: Canned cycles)
   group 4 = {M1} (Optional stop, ignored)
   group 6 = {M6} (Tool change)
   group 7 = {G41, G42} cutter radius compensation (G40 is supported)
   group 8 = {G43} tool length offset (G43.1/G49 are supported)
   group 8 = {M7*} enable mist coolant (* Compile-option)
   group 9 = {M48, M49, M56*} enable/disable override switches (* Compile-option)
   group 10 = {G98, G99} return mode canned cycles
   group 13 = {G61.1, G64} path control mode (G61 is supported)
*/
