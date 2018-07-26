/*
  system.c - Handles system level commands and real-time processes
  Part of Grbl

  Copyright (c) 2017-2018 Terje Io
  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC

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

// Pin change interrupt for pin-out commands, i.e. cycle start, feed hold, and reset. Sets
// only the realtime command execute variable to have the main program execute these when
// its ready. This works exactly like the character-based realtime commands when picked off
// directly from the incoming serial data stream.
void control_interrupt_handler (control_signals_t signals)
{
    if (signals.value) {
        if ((signals.reset || signals.e_stop) && sys.state != STATE_ESTOP)
            mc_reset();
        else if (signals.cycle_start) {
            // Cancel any pending tool change
            if(gc_state.tool_change)
                gc_state.tool_change = false;
            bit_true(sys_rt_exec_state, EXEC_CYCLE_START);
        }
        else if (signals.feed_hold)
            bit_true(sys_rt_exec_state, EXEC_FEED_HOLD);
#ifdef SAFETY_DOOR_IGNORE_WHEN_IDLE
		else if (signals.safety_door_ajar) {
			if(sys.state != STATE_IDLE && sys.state != STATE_JOG)
				bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
			spindle_stop();
		}
#else
		else if (signals.safety_door_ajar)
			bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
#endif
    }
}


// Executes user startup script, if stored.
void system_execute_startup (char *line)
{
    uint_fast8_t n;
    for (n = 0; n < N_STARTUP_LINE; n++) {
        if (!(settings_read_startup_line(n, line)))
            report_execute_startup_message(line, Status_SettingReadFail);
        else if (line[0] != '\0')
            report_execute_startup_message(line, gc_execute_block(line, NULL));
    }
}


// Directs and executes one line of formatted input from protocol_process. While mostly
// incoming streaming g-code blocks, this also executes Grbl internal commands, such as
// settings, initiating the homing cycle, and toggling switch states. This differs from
// the realtime command module by being susceptible to when Grbl is ready to execute the
// next line during a cycle, so for switches like block delete, the switch only effects
// the lines that are processed afterward, not necessarily real-time during a cycle,
// since there are motions already stored in the buffer. However, this 'lag' should not
// be an issue, since these commands are not typically used during a cycle.
status_code_t system_execute_line (char *line)
{

    uint_fast8_t counter = 1;
    float parameter, value;
    control_signals_t control_signals;

    switch (line[1]) {

        case '\0' :
            report_grbl_help();
            break;

        case 'J' : // Jogging
            // Execute only if in IDLE or JOG states.
            if (!(sys.state == STATE_IDLE || (sys.state & (STATE_JOG|STATE_TOOL_CHANGE))))
                return Status_IdleError;
            return line[2] != '=' ? Status_InvalidStatement : gc_execute_block(line, NULL); // NOTE: $J= is ignored inside g-code parser and used to detect jog motions.
//            break;

        case '$' : // Prints Grbl settings
            if (line[2] != '\0' )
                return Status_InvalidStatement;
            if (sys.state & (STATE_CYCLE|STATE_HOLD))
                return Status_IdleError; // Block during cycle. Takes too long to print.
            report_grbl_settings();
            break;

        case 'G' : // Prints gcode parser state
            if (line[2] != '\0' )
                return Status_InvalidStatement;
            // TODO: Move this to realtime commands for GUIs to request this data during suspend-state.
            report_gcode_modes();
            break;

        case 'B' : // Toggle block delete mode
            if (line[2] != '\0')
                return Status_InvalidStatement;
            sys.block_delete_enabled = !sys.block_delete_enabled;
            report_feedback_message(sys.block_delete_enabled ? Message_Enabled : Message_Disabled);
            break;

        case 'C' : // Set check g-code mode [IDLE/CHECK]
            if (line[2] != '\0')
                return Status_InvalidStatement;
            // Perform reset when toggling off. Check g-code mode should only work if Grbl
            // is idle and ready, regardless of alarm locks. This is mainly to keep things
            // simple and consistent.
            if (sys.state == STATE_CHECK_MODE) {
                mc_reset();
                report_feedback_message(Message_Disabled);
            }
            else if (sys.state != STATE_IDLE)  // Requires idle mode.
                return Status_IdleError;
            set_state(STATE_CHECK_MODE);
            report_feedback_message(Message_Enabled);
            break;

        case 'X' : // Disable alarm lock [ALARM]
            if (line[2] != '\0' )
                return Status_InvalidStatement;
            else if (sys.state & (STATE_ALARM|STATE_ESTOP)) {

            	control_signals = hal.system_control_get_state();

                // Block if e-stop is active.
                if (control_signals.e_stop)
                    return Status_EStop;

            	// Block if safety door is ajar.
                if (control_signals.safety_door_ajar)
                    return Status_CheckDoor;

            	// Block if safety reset is active.
                if(control_signals.reset)
                    return Status_Reset;

                report_feedback_message(Message_AlarmUnlock);
                set_state(STATE_IDLE);
                // Don't run startup script. Prevents stored moves in startup from causing accidents.
            } // Otherwise, no effect.
            break;

        default :

            // Block any system command that requires the state as IDLE/ALARM. (i.e. sleep, homing)
            if (sys.state == STATE_IDLE || sys.state == STATE_IDLE) switch(line[1]) {

                case 'H' : // Perform homing cycle [IDLE/ALARM]

                    control_signals = hal.system_control_get_state();

                    // Block if e-stop is active.
                    if (control_signals.e_stop)
                        return Status_EStop;

                    if (!settings.flags.homing_enable)
                        return Status_SettingDisabled;

                    // Block if safety door is ajar.
                    if (control_signals.safety_door_ajar)
                        return Status_CheckDoor;

                    // Block if safety reset is active.
                    if(control_signals.reset)
                        return Status_Reset;

                    set_state(STATE_HOMING); // Set system state variable

                    if (line[2] == '\0')
                        mc_homing_cycle(0); // Home axes according to configuration

                    else if (line[3] == '\0') {
                        switch (line[2]) {
                            case 'X':
                                mc_homing_cycle(X_AXIS_BIT);
                                break;
                            case 'Y':
                                mc_homing_cycle(Y_AXIS_BIT);
                                break;
                            case 'Z':
                                mc_homing_cycle(Z_AXIS_BIT);
                                break;
                          #ifdef A_AXIS
                            case 'A':
                                mc_homing_cycle(A_AXIS_BIT);
                                break;
                          #endif
                          #ifdef B_AXIS
                            case 'B':
                                mc_homing_cycle(B_AXIS_BIT);
                                break;
                          #endif
                          #ifdef C_AXIS
                            case 'C':
                                mc_homing_cycle(C_AXIS_BIT);
                                break;
                          #endif
                          default:
                                return Status_InvalidStatement;
                        }

                    } else
                        return Status_InvalidStatement;

                    if (!sys.abort) {  // Execute startup scripts after successful homing.
                        set_state(STATE_IDLE); // Set to IDLE when complete.
                        st_go_idle(); // Set steppers to the settings idle state before returning.
                        if (line[2] == '\0')
                            system_execute_startup(line);
                    }
                    break;

                case 'S' : // Puts Grbl to sleep [IDLE/ALARM]
                    if ((line[2] != 'L') || (line[3] != 'P') || (line[4] != '\0'))
                        return Status_InvalidStatement;
                    system_set_exec_state_flag(EXEC_SLEEP); // Set to execute sleep mode immediately
                    break;

            }

            // Block any system command that requires the state as IDLE/ALARM/ESTOP. (i.e. EEPROM)
            if (sys.state == STATE_IDLE || (sys.state & (STATE_ALARM|STATE_ESTOP))) switch(line[1]) {

                case '#' : // Print Grbl NGC parameters
                    if (line[2] != '\0')
                        return Status_InvalidStatement;
                    report_ngc_parameters();
                    break;

                case 'I' : // Print or store build info. [IDLE/ALARM]
                    if (line[2] == '\0') {
                        settings_read_build_info(line);
                        report_build_info(line);
                    }
                  #ifdef ENABLE_BUILD_INFO_WRITE_COMMAND
                    else if (line[2] == '=' && strlen(&line[3]) < (MAX_STORED_LINE_LENGTH - 1))
                        settings_write_build_info(&line[3]);
                  #endif
                    else
                        return Status_InvalidStatement;
                    break;

                case 'R' : // Restore defaults [IDLE/ALARM]

                    if ((line[2] != 'S') || (line[3] != 'T') || (line[4] != '=') || (line[6] != '\0'))
                        return Status_InvalidStatement;

                    switch (line[5]) {

                      #ifdef ENABLE_RESTORE_EEPROM_DEFAULT_SETTINGS
                        case '$':
                            settings_restore SETTINGS_RESTORE_DEFAULTS;
                            break;
                      #endif

                      #ifdef ENABLE_RESTORE_EEPROM_CLEAR_PARAMETERS
                        case '#':
                            settings_restore(SETTINGS_RESTORE_PARAMETERS);
                            break;
                      #endif

                      #ifdef ENABLE_RESTORE_EEPROM_WIPE_ALL
                        case '*':
                            settings_restore(SETTINGS_RESTORE_ALL);
                            break;
                      #endif

                      #ifdef ENABLE_RESTORE_DRIVER_PARAMETERS
                        case '&':
                            settings_restore(SETTINGS_RESTORE_DRIVER_PARAMETERS);
                            break;
                      #endif

                        default: return Status_InvalidStatement;
                    }
                    report_feedback_message(Message_RestoreDefaults);
                    mc_reset(); // Force reset to ensure settings are initialized correctly.
                    break;

                case 'N' : // Startup lines. [IDLE/ALARM]
                    if (line[++counter] == '\0') { // Print startup lines
                        for (counter = 0; counter < N_STARTUP_LINE; counter++) {
                            if (!(settings_read_startup_line(counter, line)))
                                report_status_message(Status_SettingReadFail);
                            else
                                report_startup_line(counter, line);
                        }
                        break;
                    }
                    if (sys.state != STATE_IDLE) // Store startup line [IDLE Only] Prevents motion during ALARM.
                        return Status_IdleError;
                    // No break. Continues into default: to read remaining command characters.

                default :  // Storing setting methods [IDLE/ALARM]
                    if(!read_float(line, &counter, &parameter))
                        return Status_BadNumberFormat;
                    if(line[counter++] != '=')
                        return Status_InvalidStatement;
                    if (line[1] == 'N') { // Store startup line
                        line = &line[counter];
                        if(strlen(line) >= (MAX_STORED_LINE_LENGTH - 1))
                            return Status_Overflow;
                        status_code_t retval;
                        if ((retval = gc_execute_block(line, NULL)) == Status_OK) // Execute gcode block to ensure block is valid.
                            settings_write_startup_line((uint8_t)truncf(parameter), line);
                        else
                            return retval;
                    } else { // Store global setting.
                        if(!read_float(line, &counter, &value))
                            return Status_BadNumberFormat;
                         if(line[counter] != '\0')
                            return Status_InvalidStatement;
                         return settings_store_global_setting((uint_fast16_t)parameter, value);
                    }
            } else
                return Status_IdleError;
    }

    return Status_OK; // If '$' command makes it to here, then everything's ok.
}



void system_flag_wco_change ()
{
  #ifdef FORCE_BUFFER_SYNC_DURING_WCO_CHANGE
    protocol_buffer_synchronize();
  #endif
    sys.report_wco_counter = 0;
}


// Returns machine position of axis 'idx'. Must be sent a 'step' array.
// NOTE: If motor steps and machine position are not in the same coordinate frame, this function
//   serves as a central place to compute the transformation.
inline static float system_convert_axis_steps_to_mpos (int32_t *steps, uint_fast8_t idx)
{
  #ifdef COREXY
    return (float)(idx == X_AXIS ? system_convert_corexy_to_x_axis_steps(steps) : (idx == Y_AXIS ? system_convert_corexy_to_y_axis_steps(steps) : steps[idx])) / settings.steps_per_mm[idx];
  #else
    return steps[idx] / settings.steps_per_mm[idx];
  #endif
}


void system_convert_array_steps_to_mpos (float *position, int32_t *steps)
{
    uint_fast8_t idx = N_AXIS;
    do {
        idx--;
        position[idx] = system_convert_axis_steps_to_mpos(steps, idx);
    } while(idx);
}


// CoreXY calculation only. Returns x or y-axis "steps" based on CoreXY motor steps.
#ifdef COREXY
int32_t system_convert_corexy_to_x_axis_steps (int32_t *steps)
{
    return (steps[A_MOTOR] + steps[B_MOTOR]) >> 1;
}

int32_t system_convert_corexy_to_y_axis_steps (int32_t *steps)
{
    return (steps[A_MOTOR] - steps[B_MOTOR]) >> 1;
}
#endif


// Checks and reports if target array exceeds machine travel limits. Returns true if check failed.
bool system_check_travel_limits(float *target)
{
    bool failed = false;
    uint_fast8_t idx = N_AXIS;

    do {
        idx--;
    #ifdef HOMING_FORCE_SET_ORIGIN
    // When homing forced set origin is enabled, soft limits checks need to account for directionality.
    // NOTE: max_travel is stored as negative
        failed = bit_istrue(settings.homing_dir.mask, bit(idx))
                  ? (target[idx] < 0.0f || target[idx] > -settings.max_travel[idx])
                  : (target[idx] > 0.0f || target[idx] < settings.max_travel[idx]);
    #else
        // NOTE: max_travel is stored as negative
        failed = target[idx] > 0.0f || target[idx] < settings.max_travel[idx];
    #endif
    } while(!failed && idx);

  return failed;
}

