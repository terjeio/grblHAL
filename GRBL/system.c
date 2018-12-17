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
// directly from the incoming data stream.
ISR_CODE void control_interrupt_handler (control_signals_t signals)
{
    if (signals.value) {
        if ((signals.reset || signals.e_stop) && sys.state != STATE_ESTOP)
            mc_reset();
        else {
    		if (signals.safety_door_ajar) {
    		    if(settings.flags.safety_door_ignore_when_idle) {
    		        // Only stop the spindle (laser off) when idle or jogging,
    		        // this to allow positioning the controlled point (spindle) when door is open.
    		        // NOTE: at least for lasers there should be an external interlock blocking laser power.
                    if(sys.state != STATE_IDLE && sys.state != STATE_JOG)
                        bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
                    spindle_stop(); // TODO: stop spindle in laser mode only?
    		    } else
    		    	bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
    		}
    		if (signals.feed_hold)
                bit_true(sys_rt_exec_state, EXEC_FEED_HOLD);
            else if (signals.cycle_start)
				bit_true(sys_rt_exec_state, EXEC_CYCLE_START);
        }
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
    status_code_t retval = Status_OK;
    char c, *org = line, *ucline = line, *lcline = line + (LINE_BUFFER_SIZE / 2);

    if(strlen(line) >= ((LINE_BUFFER_SIZE / 2) - 1))
        return Status_Overflow;

    // Uppercase original and copy original out in the buffer
    // TODO: create a common function for stripping down uppercase version?
    do {
        c = *org++;
        if(c != ' ') // Remove spaces from uppercase version
            *ucline++ = CAPS(c);
        *lcline++ = c;
    } while(c);

    lcline = line + (LINE_BUFFER_SIZE / 2);

    switch (line[1]) {

        case '\0':
            report_grbl_help();
            break;

        case 'J': // Jogging, execute only if in IDLE or JOG states.
            if (!(sys.state == STATE_IDLE || (sys.state & (STATE_JOG|STATE_TOOL_CHANGE))))
                retval = Status_IdleError;
            else
                retval = line[2] != '=' ? Status_InvalidStatement : gc_execute_block(line, NULL); // NOTE: $J= is ignored inside g-code parser and used to detect jog motions.
            break;

        case '$': // Prints Grbl settings
            if (line[2] != '\0' )
                retval = Status_InvalidStatement;
            else if (sys.state & (STATE_CYCLE|STATE_HOLD))
                retval =  Status_IdleError; // Block during cycle. Takes too long to print.
            else
                report_grbl_settings();
            break;

        case 'G': // Prints gcode parser state
            if (line[2] != '\0' )
                retval = Status_InvalidStatement;
            else
                // TODO: Move this to realtime commands for GUIs to request this data during suspend-state.
                report_gcode_modes();
            break;

        case 'B': // Toggle block delete mode
            if (line[2] != '\0')
                retval = Status_InvalidStatement;
            else {
                sys.block_delete_enabled = !sys.block_delete_enabled;
                hal.report.feedback_message(sys.block_delete_enabled ? Message_Enabled : Message_Disabled);
            }
            break;

        case 'C': // Set check g-code mode [IDLE/CHECK]
            if (line[2] != '\0')
                retval = Status_InvalidStatement;
            else if (sys.state == STATE_CHECK_MODE) {
                // Perform reset when toggling off. Check g-code mode should only work if Grbl
                // is idle and ready, regardless of alarm locks. This is mainly to keep things
                // simple and consistent.
                mc_reset();
                hal.report.feedback_message(Message_Disabled);
            } else if (sys.state == STATE_IDLE) { // Requires idle mode.
                set_state(STATE_CHECK_MODE);
                hal.report.feedback_message(Message_Enabled);
            } else
                retval = Status_IdleError;
            break;

        case 'X': // Disable alarm lock [ALARM]
            if (line[2] != '\0' )
                retval = Status_InvalidStatement;
            else if (sys.state & (STATE_ALARM|STATE_ESTOP)) {

                control_signals_t control_signals = hal.system_control_get_state();

                // Block if e-stop is active.
                if (control_signals.e_stop)
                    retval = Status_EStop;
            	// Block if safety door is ajar.
                else if (control_signals.safety_door_ajar)
                    retval = Status_CheckDoor;
            	// Block if safety reset is active.
                else if(control_signals.reset)
                    retval = Status_Reset;
                else {
                    hal.report.feedback_message(Message_AlarmUnlock);
                    set_state(STATE_IDLE);
                }
                // Don't run startup script. Prevents stored moves in startup from causing accidents.
            } // Otherwise, no effect.
            break;

        case 'H' : // Perform homing cycle [IDLE/ALARM]
            if(!(sys.state == STATE_IDLE || sys.state == STATE_ALARM))
                retval = Status_IdleError;
            else {

                control_signals_t control_signals = hal.system_control_get_state();

                // Block if e-stop is active.
                if (control_signals.e_stop)
                    retval = Status_EStop;
                else if (!settings.homing.flags.enabled)
                    retval = Status_SettingDisabled;
                // Block if safety door is ajar.
                else if (control_signals.safety_door_ajar)
                    retval = Status_CheckDoor;
                // Block if safety reset is active.
                else if(control_signals.reset)
                    retval = Status_Reset;
            }

            if(retval == Status_OK) {

                set_state(STATE_HOMING); // Set system state variable

                if (line[2] == '\0')
                    mc_homing_cycle(0); // Home axes according to configuration

                else if (settings.flags.homing_single_axis_commands && line[3] == '\0') {

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
                          retval = Status_InvalidStatement;
                          break;
                    }
                } else
                    retval = Status_InvalidStatement;
            }

            if (retval == Status_OK && !sys.abort) {  // Execute startup scripts after successful homing.
                set_state(STATE_IDLE); // Set to IDLE when complete.
                st_go_idle(); // Set steppers to the settings idle state before returning.
                if (line[2] == '\0')
                    system_execute_startup(line);
            }
            break;

        case 'S' : // Puts Grbl to sleep [IDLE/ALARM]
            if(!settings.flags.sleep_enable || !(line[2] == 'L' && line[3] == 'P' && line[4] == '\0'))
                retval = Status_InvalidStatement;
            else if(!(sys.state == STATE_IDLE || sys.state == STATE_ALARM))
                retval = Status_IdleError;
            else
                system_set_exec_state_flag(EXEC_SLEEP); // Set to execute sleep mode immediately
            break;

        case '#' : // Print Grbl NGC parameters
            if (line[2] != '\0')
                retval = Status_InvalidStatement;
            else if (!(sys.state == STATE_IDLE || (sys.state & (STATE_ALARM|STATE_ESTOP))))
                retval = Status_IdleError;
            else
                report_ngc_parameters();
            break;

        case 'I' : // Print or store build info. [IDLE/ALARM]
            if (!(sys.state == STATE_IDLE || (sys.state & (STATE_ALARM|STATE_ESTOP))))
                retval = Status_IdleError;
            else if (line[2] == '\0') {
                settings_read_build_info(line);
                report_build_info(line);
            }
          #ifdef ENABLE_BUILD_INFO_WRITE_COMMAND
            else if (line[2] == '=' && strlen(&line[3]) < (MAX_STORED_LINE_LENGTH - 1))
                settings_write_build_info(&lcline[3]);
          #endif
            else
                retval = Status_InvalidStatement;
            break;

        case 'R' : // Restore defaults [IDLE/ALARM]
            if (!(line[2] == 'S' && line[3] == 'T' && line[4] == '=' && line[6] == '\0'))
                retval = Status_InvalidStatement;
            else if (!(sys.state == STATE_IDLE || (sys.state & (STATE_ALARM|STATE_ESTOP))))
                retval = Status_IdleError;
            else switch (line[5]) {

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

                default:
                    retval = Status_InvalidStatement;
                    break;
            }
            if(retval == Status_OK) {
                hal.report.feedback_message(Message_RestoreDefaults);
                mc_reset(); // Force reset to ensure settings are initialized correctly.
            }
            break;

        case 'N' : // Startup lines. [IDLE/ALARM]
            if (!(sys.state == STATE_IDLE || (sys.state & (STATE_ALARM|STATE_ESTOP))))
                retval = Status_IdleError;
            else if (line[2] == '\0') { // Print startup lines
                uint_fast8_t counter;
                for (counter = 0; counter < N_STARTUP_LINE; counter++) {
                    if (!(settings_read_startup_line(counter, line)))
                        hal.report.status_message(Status_SettingReadFail);
                    else
                        report_startup_line(counter, line);
                }
                break;
            } else if (sys.state == STATE_IDLE) { // Store startup line [IDLE Only] Prevents motion during ALARM.

                uint_fast8_t counter = 2;
                float parameter;
                if(!read_float(line, &counter, &parameter))
                    retval = Status_BadNumberFormat;
                else if(line[counter++] != '=' || parameter - truncf(parameter) != 0.0f)
                    retval = Status_InvalidStatement;
                else if(parameter > (float)N_STARTUP_LINE)
                    retval = Status_InvalidStatement;
                else {
                    line = &line[counter];
                    if(strlen(line) >= (MAX_STORED_LINE_LENGTH - 1))
                        retval = Status_Overflow;
                    else if ((retval = gc_execute_block(line, NULL)) == Status_OK) // Execute gcode block to ensure block is valid.
                        settings_write_startup_line((uint8_t)parameter, line);
                }
            } else
                retval = Status_IdleError;
            break;

#ifdef DEBUGOUT
        case 'Q':
            hal.stream.write(uitoa((uint32_t)sizeof(settings_t)));
            hal.stream.write(" ");
            hal.stream.write(uitoa((uint32_t)sizeof(coord_data_t) + 1));
            hal.stream.write("\r\n");
            break;
#endif

        default :
            retval = Status_Unhandled;

            // Let user code have a peek at system commands before check for global setting
            if(hal.driver_sys_command_execute)
                retval = hal.driver_sys_command_execute(sys.state, line, lcline);

            if (retval == Status_Unhandled) {
                // Check for global setting, store if so
                if(sys.state == STATE_IDLE || (sys.state & (STATE_ALARM|STATE_ESTOP))) {
                    uint_fast8_t counter = 1;
                    float parameter;
                    if(!read_float(line, &counter, &parameter))
                        retval = Status_BadNumberFormat;
                    else if(line[counter++] != '=' || parameter - truncf(parameter) != 0.0f)
                        retval = Status_InvalidStatement;
                    else if((uint_fast16_t)parameter > Setting_AxisSettingsMax)
                        retval = Status_InvalidStatement;
                    else
                        retval = settings_store_global_setting((uint_fast16_t)parameter, &lcline[counter]);
                } else
                    retval = Status_IdleError;
            }
    }

    return retval;
}

void system_flag_wco_change ()
{
    if(!settings.flags.force_buffer_sync_on_wco_change)
        protocol_buffer_synchronize();

    sys.report.wco_counter = 0;
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

    // NOTE: max_travel is stored as negative

    if(settings.flags.homing_force_set_origin) {
        do {
            idx--;
        // When homing forced set origin is enabled, soft limits checks need to account for directionality.
            failed = bit_istrue(settings.homing.dir_mask, bit(idx))
                      ? (target[idx] < 0.0f || target[idx] > -settings.max_travel[idx])
                      : (target[idx] > 0.0f || target[idx] < settings.max_travel[idx]);
        } while(!failed && idx);
    } else do {
        idx--;
        failed = target[idx] > 0.0f || target[idx] < settings.max_travel[idx];
    } while(!failed && idx);

  return failed;
}

