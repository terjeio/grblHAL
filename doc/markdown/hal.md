### GrblHAL, driver interface v4

There is only one fixed entry point into the driver, `driver_init()`, that is used for enabling the serial stream, nonvolatile storage and initializing the required function pointers. Please note that the grbl core can, and will, only access the driver via this function call and the published HAL entries. The driver, however, is free to call the public functions and reference data structures in the grbl core as it sees fit.

__NOTE:__ this is an incomplete in details, preliminary and pretty terse documentation.

__Startup sequence:__

The `main()` function should call , or start grbl as a task, `grbl_enter()` as the last call. This will never return.

This in turn:

* clears the HAL structure
* sets the HAL version number
* calls `driver_init()` which then:
    - verifies that the HAL version number matches.
    - initializes serial stream.
    - initializes nonvolatile storage if available.
    - sets up the HAL function pointers (into the driver).
    - sets up the driver capabilities.
	- returns `true` if everything went well
* sets up the HAL callback function pointers (into grbl).
* optionally initializes the EEPROM emulation (requires heap memory, will fail if not enough available).
* loads the settings.
* calls `hal.driver_setup` so that the driver can configure the hardware.
* sets up internal variables and starts the main protocol loop.

---

__Constants:__

`uint32_t version` - HAL version, driver should verify match (set by grbl core).

`char *info`	   - pointer to driver info string, typically name of processor/platform.

`uint32_t f_step_timer` - frequency of main stepper timer in Hz.

`uint32_t rx_buffer_size` - input stream buffer size.

---

__Required entry points that must be assigned by the driver:__

`bool (*driver_setup)(settings_t *settings)`  
Called after settings has been loaded from nonvolatile storage. The driver should use this to enable the required peripherals and perform initial initialization of these.

`void (*limits_enable)(bool on)`  
Used for enabling/disabling limit switches interrupt.

`axes_signals_t (*limits_get_state)(void)`  
Returns current limit switches status.

`void (*coolant_set_state)(coolant_state_t mode)`
Set coolant state (flood and mist).

`coolant_state_t (*coolant_get_state)(void)`  
Returns current coolant state (flood and mist) as read from GPIO outputs.

`void (*delay_ms)(uint32_t ms, void (*callback)(void))`  
Delay execution for a number of milliseconds, if a callback function is provided \(not `NULL`\) it will return immediately after setting up the callback for execution after the delay expires.
NOTE: the callback function may be called from interrupt context.

`bool (*probe_get_state)(void)`  
Returns current probe state taking into account the current inversion status. The driver may implement this by the means of an interrupt handler for maximum responsiveness.

`void (*spindle_set_state)(spindle_state_t state, float rpm)`  
Enable/disable spindle according to the supplied parameters.

`spindle_state_t (*spindle_get_state)(void)`  
Returns spindle state.

`void (*spindle_update_rpm)(float rpm)`  
Update the spindle speed.

`control_signals_t (*system_control_get_state)(void)`  
Returns control input signals status.

`void (*stepper_wake_up)(void)`  
Starts motion by either enabling the stepper driver timer interrupt after a short delay or by calling `hal.stepper_interrupt_callback()` directly.

`void (*stepper_go_idle)(void)`
Disables the stepper timer interrupt.  

`void (*stepper_enable)(axes_signals_t enable)`  s
Enables/disables the stepper drivers, may be per axis or, if only one GPIO is available, by using the x-axis status.

`void (*stepper_set_outputs)(axes_signals_t step_outbits)`  
Set stepper drive GPIO pins to a the given level. Superfluous?

`void (*stepper_set_directions)(axes_signals_t dir_outbits)`  
Set stepper direction GPIO pins to a the given level. Superfluous?

`void (*stepper_cycles_per_tick)(uint32_t cycles_per_tick)`  
Sets up the stepper timer for the next step interval. A free running timer with a moving may be best for jitter free output?

`void (*stepper_pulse_start)(stepper_t *stepper)`  
Starts pulse output. The driver may implement several version of this to reduce call overhead. Eg. one for "standard" pulse output \(no delay\), a delayed version or, if spindle sync is supported, a version handling the spindle sync. The driver can then dynamically switch between these as required.

`uint16_t (*stream.get_rx_buffer_available)(void)`  
Returns number of free character slots in the input stream buffer.

`bool (*stream.write)(const char *s)`  
Write a 0 terminated string to the current output stream.
 
`void (*stream.write_all)(const char *s)`  
Write a 0 terminated string to all enabled output streams.

`int16_t (*stream.read)(void)`  
Read a character from the current input stream. Returns -1 if no character available.

`void (*stream.reset_read_buffer)(void)`  
Flushes the current input stream buffer.

`void (*stream.cancel_read_buffer)(void)`  
Flushes the current input stream buffer and inserts a CAN character. This will typically be used downstream to flush the block input buffer as part of jog cancelling.

`bool (*stream.suspend_read)(bool await)`  

The stream handling functions may be used to redirect input and output streams by replacing the handlers. Eg. when reading from a SD card redirect `stream.read` to the SD card get_next_character function.  Similarily other streams may be dynamically switched in or out by replacing the handler functions on connect/disconnect events - there is no need for the grbl core to know what the stream source is. An added benefit of this is that the serial stream will be available for reconfiguration if there is any problems with the setup.

`void (*set_bits_atomic)(volatile uint_fast16_t *value, uint_fast16_t bits)`  
Set bits in a variable atomically.

`uint_fast16_t (*clear_bits_atomic)(volatile uint_fast16_t *value, uint_fast16_t bits)`  
Clear bits in a variable atomically. Returns the original value.

`uint_fast16_t (*set_value_atomic)(volatile uint_fast16_t *value, uint_fast16_t bits)`  
Set a variable atomically. Returns the original value.

`void (*settings_changed)(settings_t *settings)`  
Called after initialization is complete and after settings change.

---

__Optional entry points that may be assigned by the driver:__

__NOTE:__ these entry points may temporarily be set 'active' by the driver, deactivate by assigning `NULL`.

`bool (*driver_release)(void)`  
Uninstalls the driver by releasing claimed hardware.

`void (*execute_realtime)(uint8_t state)`  
Called regularly by the core, may be used for functions like keypad scanning or display update, 

`uint8_t (*driver_mcode_check)(uint8_t mcode)`  
Return the `mcode` value if it is implemented by the driver, 0 if not.

`status_code_t (*driver_mcode_validate)(parser_block_t *gc_block, uint32_t *value_words)`  
Return `Status_OK` if parameters are valid. The M code to be validated is stored in `gc_block->user_defined_mcode`, set `gc_block->user_defined_mcode_sync` to true if execution has to be synchronized. Available parameters are stored in `gc_block->values`.
NOTE: any value words claimed by the M code must be cleared from the `value_words` variable pointed to. 

`void (*driver_mcode_execute)(uint_fast16_t state, parser_block_t *gc_block)`  
Execute user defined M code, The M code to be executed is stored in `gc_block->user_defined_mcode`, parameters in `gc_block->values`.

 
`void (*driver_rt_command_execute)(uint8_t cmd)`  
Execute a real time command, `cmd` is an top bit set character received from the input stream.  
NOTE: this is not called from an interrupt context.

`void (*driver_rt_report)(void)`  
Add information to the standard real time report by calling `hal.serial_write_string`. The supplied string must start with a `|` character.

`void (*driver_feedback_message)(void)`  
Allows adding feedback messages to the standard set. 

`status_code_t (*driver_sys_command_execute)(uint_fast16_t state, char *line, char *lcline)`  
Parse user defined system command \($ prefixed\), two versions of the command is provided; `line` is a uppercase version with spaces removed, `lcline` is a lowercase version. Return `Status_Unhandled` if not handled, `Status_OK` or an appropriate error status if handled.


`bool (*get_position)(int32_t (*position)[N_AXIS])`  
Returns the current machine coordinate position, will be used by to set the initial position on a cold start.

`void (*tool_select)(tool_data_t *tool)`  
Called when M6T<n> is parsed, for ATC implementation. 

`void (*tool_change)(parser_state_t *gc_state)`  
Called when a tool change is to take place, for ATC implementation.

`void (*show_message)(const char *msg)`  
Display a message from the G code program synchronosly, requires memory heap memory available for the core.
Suggested implementation:
```
static void showMessage (const char *msg)
{
    hal.serial_write_string("[MSG:");
    hal.serial_write_string(msg);
    hal.serial_write_string("]\r\n");
```

`void (*report_options)(void)`  
Called when options are reported on $I request. Allows the driver to output custom options.  

`void (*driver_reset)(void)`  
Called when a soft reset is executed by the core.

`bool (*driver_setting)(uint_fast16_t setting, float value, char *svalue)`  
Called when an unknown (for the core) setting parameter has been issued. May be used to implement driver specific settings.
`setting` is the parameter, `value` is the numerical parameter value or `NAN` if not a number, `svalue` the parameter string.

`void (*driver_settings_restore)(uint8_t restore_flag)` 
Called when there is a need to restore settings to their default value. Test for the `SETTINGS_RESTORE_DRIVER_PARAMETERS` flag for when to restore the driver specific settings.
 
`void (*driver_settings_report)(bool axis_settings)` 
Called when the settings report is generated, first time before the axis specific parameters, second time after.
 
`spindle_data_t (*spindle_get_data)(spindle_data_request_t request)`  
Called when spindle synchronized motion is called for. TBC.

`void (*spindle_reset_data)(void)`  
Called when spindle synchronized motion is called for. TBC.

---

__Non volatile storage of settings:__

Register in `hal.eeprom` struct. If the core has `EMULATE_EEPROM` enabled \(default on\) this will provide buffered access to storage with writes deferred to when grbl is in idle mode. Writes will then only occur if there are actual changes to memory.

`eeprom_type type`  
The type of storage provided, set to one of:  
```
EEPROM_None     - no storage available
EEPROM_Physical - physical storage available (typically EEPROM) that supports read/writes without erasing.
EEPROM_Emulated - block sized storage available (typically flash) that must be erased before updates. Requires `EMULATE_EEPROM` enabled.

```

`uint16_t size`  
Set to 0 to use default grbl size or to a larger value if driver specific settings are required that cannot fit into the default grbl area.

`uint16_t driver_area.address`  
If driver requires a private block of memory set the offset here. It may be inside the default grbl area or outside.

`uint16_t driver_area.size`  
Size of driver area allocated.

When settings are stored in flash use `EEPROM_Emulated` type and provide implementation of these functions:

`bool (*memcpy_from_flash)(uint8_t *dest)`  
Copy `hal.eeprom.size` bytes from flash to memory.

`bool (*memcpy_to_flash)(uint8_t *source)`  
Perform sector\(s\) erase and copy `hal.eeprom.size` bytes to flash from memory.

When settings are stored in EEPROM \(or similar storage such as FRAM\) use `EEPROM_Physical` type and provide implementation of these functions: 

`uint8_t (*get_byte)(uint32_t addr)`  
Return a byte read from the given address. 

`void (*put_byte)(uint32_t addr, uint8_t new_value)`  
Write a byte to given address. 

`void (*memcpy_to_with_checksum)(uint32_t destination, uint8_t *source, uint32_t size)`  
Copy a memory block of given size to the given address adding a one byte checksum.

`bool (*memcpy_from_with_checksum)(uint8_t *destination, uint32_t source, uint32_t size)`  
Copy a memory block of given size from the given address. Return `false` if the checksum does not match.

---
	
__Entry points set by and normally used internally by the core:__

`void (*report_status_message)(status_code_t status_code)`  
Called when reporting status following the execution of a block. This entry point may temporarily be redirected, eg. to a SD card streaming implementation, if the feedback is required for processing.

---

__Entry points to be called by the driver, set up by the core:__

`bool (*protocol_enqueue_gcode)(char *data)`  
Entry point for driver supplied gcode blocks, will only be accepted if grbl is in idle or jog state. May be used for jogging via pendant.

`bool (*protocol_process_realtime)(char data)`  
Must be called for each character received from the input stream prior to buffering. Returns `false` if the character is to be discarded.

`bool (*serial_blocking_callback)(void)`  
Not yet implemented in a meaningful way.

`void (*stepper_interrupt_callback)(void)`  
To be called on main stepper timer timeouts.

`void (*limit_interrupt_callback)(axes_signals_t state)`  
To be called when a limit interrupt occurs, possibly after a debounce delay.

`void (*control_interrupt_callback)(control_signals_t signals)`  
To be called when a control signals interrupt occurs, possibly after a debounce delay.

`void (*spindle_index_callback)(spindle_data_t *rpm)`  
To be called when a spindle index event occurs. Used for spindle synchronized motion. TBC.

---

__Driver capabilities:__

Driver capabilities are announced to the core via the `hal.driver_cap` bit map.
The following bits are defined and _must be set_ to comply with the driver implementation:

```
mist_control
variable_spindle
safety_door
spindle_dir
software_debounce
step_pulse_delay
limits_pull_up
control_pull_up
probe_pull_up
amass_level - this has two bits allocated allowing 0-3 range to be specified, may be removed
program_stop - input signal
spindle_at_speed - required for spindle synchronized motion
spindle_pwm_invert
spindle_pid
laser_ppi_mode - laser pulses per inch supported
spindle_sync - for spindle synchronized motion
sd_card
bluetooth
ethernet
wifi
```
---
2019-04-12
