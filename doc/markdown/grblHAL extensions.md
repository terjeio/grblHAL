### GrblHAL, assorted extensions since master Grbl 1.1

#### Realtime report:

\<Status>|\<WPos:|MPos:\><axis positions\>[|Bf:\<block buffers free\>,\<RX characters free>\][|PN:\<signals\>][WPos:][|MPG:\<0|1\>][|H:\<0|1\>][|D:\<0|1\>]

New status, __Tool__, for manual tool change, driver dependent.
If supported the `NEWOPT:` report contains `TC` \(see below\) and a `M6` triggers the new state, if not `M6` returns error as before.
After the sender application receives a __Tool__ state RT report it should suspend sending further blocks and acknowledge by sending `0xA3 (CMD_TOOL_ACK)`. Grbl will then switch to a secondary input buffer allowing jogging commands etc, this until a cycle start is issued. Upon receiving the cycle start command grbl will revert to the primary input buffer and resume normal processing.  
_NOTE:_ currently state is not saved/restored so spindle stop/start etc. needs to be added to the gcode.

New status message `|MPG:<0|1>` used to inform that a pendant has released/taken control of the input stream.  
0 - pendant has released control, senders should resume normal operation - pendant enters listening mode.  
1 - pendant has taken over input stream, senders should disable UI but still update controls.

New status message `|SD:<pct complete>[,<filename>]` used to inform status when streaming from SD card.

New status message `|H:<0|1>` used to report homing status changes.  
0 - homing is not complete.  
1 - homing complete.

This status is only reported when homing is enabled and then on homing status changes.

New status message `|D:<0|1>` used to report radius/diameter mode changes.  
0 - G8: radius mode \(default\).  
1 - G7: diameter mode.

This status is only reported when lathe mode is enabled, also G7 and G8 will return an error if not.

#### OPT report:

Number of supported axes added to first line:  
`[OPT:<options>,<block buffer size>,<RX buffer size>,<number of axes>]`

A second line of options is added:  
`NEWOPT:<comma separated list>]`

`<comma separated list>` can contain any of the following values:
```
SD - SD Card streaming available.
BT - Bluetooth streaming available.
WIFI - Wi-Fi streaming available.
ETH - Ethernet streaming available.
TC - Manual tool change (M6) available.
ATC - Automatic tool change (M6) available.
LATHE - Lathe mode configured.
SS - Spindle sync available.
PID - PID log data available.
```

#### NGC parameters report:

If scaling is active scaling factors are reported:  
`[G51:<axis scaling factors>]`

If a tool table is available \(option in config.h\) tool table contents is added.

`[T:<tool number>,<axis offsets>]`

`TLO` parameter includes offsets for all axes.

<a name='settings'>#### Settings:

Datatypes:
```
<n> = integer value
<float> = floating point value
<boolean> = 0 = false, 1 = true
<string> = ASCII string with some limitations for acceptable characters. To be revised.
<axis mask> = bitmask where bit 0 = X, 1 = Y, 2 = Z, 3 = A...
<coolant mask> = bitmask where bit 0 = flood, 1 = mist
<spindle mask> = bitmask where bit 0 = spindle on, 1 = spindle ccw
<control mask> = bitmask where bit 0 = reset, 1 = feed hold, 2 = cycle start, 3 = safety door, 4 = block delete, 5 = stop disable
```

$14=\<control mask\> default from `INVERT_CONTROL_PIN_MASK`.  
Invert control input signals.

$15=\<coolant mask\> default from `INVERT_COOLANT_FLOOD_PIN` and `INVERT_COOLANT_MIST_PIN`.  
Invert coolant output signals.

$16=\<spindle mask\> default from `INVERT_SPINDLE_ENABLE_PIN`.  
Invert spindle output signals.

$17=\<control mask\> default from `DISABLE_CONTROL_PINS_PULL_UP_MASK`.  
Disable control signal pullup, replaces `#define DISABLE_CONTROL_PIN_PULL_UP`.
	
$18=\<axis mask\> default from `DISABLE_LIMIT_PINS_PULL_UP_MASK`.  
Disable limit signals pull up, replaces `#define DISABLE_LIMIT_PIN_PULL_UP`.
Driver may apply pull down instead.
 
$19=\<boolean\> default from `DISABLE_PROBE_PIN_PULL_UP`  
Disable probe pull up.
Driver may apply pull down instead.

$21=\<mask\>  
Changed from original boolean for enabling hard limits.  
```
bit0 - enable hard limits.  
bit1 - enable strict mode when hard limits enabled
```

`bit 1` replaces `#define CHECK_LIMITS_AT_INIT`.  
__NOTE:__ In strict mode switches will also be checked when `$X` is issued, if still engaged `error 45` will be reported. Homing is still possible, but for `$X` to work limit switches has to be disengaged or overridden.

$28=\<float\> default from `DEFAULT_G73_RETRACT`.
Specifies `G73` retract distance in mm.

$29=\<n\> : default from `DEFAULT_STEP_PULSE_DELAY`, range 0 - 10.  
Stepper pulse delay in microseconds, replaces `#define STEP_PULSE_DELAY`.

$31=\<n\> default value derived from `DEFAULT_LASER_MODE` and `DEFAULT_LATHE_MODE`.  
Changed from original boolean for enabling laser mode.  
```
0 - normal mode.  
1 - laser mode.  
2 - lathe mode.
```

$33=\<float\> : default from `DEFAULT_SPINDLE_PWM_FREQ`, range driver dependent.  
Spindle PWM frequency i Hz \(from LPC port\).

$34=\<n\> : default from `DEFAULT_SPINDLE_PWM_OFF_VALUE`, range 0 - 100.  
Spindle off PWM duty cycle in percent \(from LPC port\).

$35=\<n\> : default from `DEFAULT_SPINDLE_PWM_MIN_VALUE`, range 0 - 100.  
Spindle minimum PWM duty cycle in percent.

$36=\<n\> : default from `DEFAULT_SPINDLE_PWM_MAX_VALUE`, range 0 - 100.  
Spindle maximum PWM duty cycle in percent \(from LPC port\).

$37=\<axis mask\> : defaults to all axes.  
Defines which steppers is to be deenergized when motion completes.
Driver/hardware dependent which are supported. At least X should be, disables all motors.

$38=\<n\> : default driver dependent.  
Spindle encoder pulses per revolution. Usage is driver dependent \(for spindle synchronized motion\).

$39=\<n\> : default 1, enable printable realtime command characters.  
Set to 0 to disable, when disabled these characters \(`?`, `!` and `~`\) are ignored as realtime commands and added to the input instead when part of a comment or a $-setting.

__NOTE:__ top bit set alternatives are provided as a safer alternative, see [config.h](../../GRBL/config.h).

$40=\<boolean\> default 0 (off).  
Enable soft limits for jogging. When enabled jog targets will be limited to machine travel limits for homed axes. 

$43=\<n\> : default from DEFAULT_N_HOMING_LOCATE_CYCLE, range 0 - 255.  
Number of homing locate cycles

$44=\<axis mask\> : default 0.  
$45=\<axis mask\> : default 0.  
$46=\<axis mask\> : default 0.  
$47=\<axis mask\> : default 0.  
$48=\<axis mask\> : default 0.  
$49=\<axis mask\> : default 0.  
Axis priority for homing lowest numbered executed first, number of available settings is same as number of supported axes.
Replaces #define HOMING_CYCLE_0 etc.

$50=\<float\> : default driver dependent.  
Jogging step speed in mm/min. Not used by core, indended use by driver and/or sender.
Senders may query this for keyboard jogging modified by CTRL key.

$51=\<float\> : default driver dependent.  
Jogging slow speed in mm/min. Not used by core, indended use by driver and/or sender.
Senders may query this for keyboard jogging.

$52=\<float\> : default driver dependent.  
Jogging fast speed in mm/min. Not used by core, indended use by driver and/or sender.
Senders may query this for keyboard jogging modified by SHIFT key.

$53=\<float\> : default driver dependent.  
Jogging step distance in mm. Not used by core, indended use by driver and/or sender.
Senders may query this for keyboard jogging modified by CTRL key.

$54=\<float\> : default driver dependent.  
Jogging slow distance in mm. Not used by core, indended use by driver and/or sender.
Senders may query this for keyboard jogging.

$55=\<float\> : default driver dependent.  
Jogging fast distance in mm. Not used by core, indended use by driver and/or sender.
Senders may query this for keyboard jogging modified by SHIFT key.

$60=\<boolean\> : default 1 (on).  
Restore default overrides when program ends. Replaces #define RESTORE_OVERRIDES_AFTER_PROGRAM_END.

$61=\<boolean\> : default 0 (off).  
Ignore safety door signal when idle. If on only the spindle \(laser\) will be switched off.
May be useful if positioning a laser head with the lid open is needed.

$62=\<boolean\> : default 0 (off).  
Enable sleep function. Replaces #define SLEEP_ENABLE \(ATMega port\)

$63=\<boolean\> : default 0 (on).  
Disable laser during hold. Replaces #define DISABLE_LASER_DURING_HOLD.

$64=\<boolean\> : default 0 (off).  
Force grbl to enter alarm mode on startup. Replaces #define FORCE_INITIALIZATION_ALARM.

$66=\<boolean\> : default 0 (off).  
Require homing sequence to be executed at startup\(?\). Replaces #define HOMING_INIT_LOCK.

$70=\<n\> : default 0, range 0 - 4 \(driver dependent\).  
Input stream selection:  
<pre>
0: Serial (default)  
1: Bluetooth  
2: Ethernet  
3: WiFi  
4: SD Card (?)  
</pre>

$71=\<string\> : max 64 characters, default empty.  
WiFi SSID.
 
$72=\<string\> : max 32 characters, default empty.  
WiFi password.

$73=\<n\> : Default 23, range 1 - 65535.  
WiFi port number listening for incoming connections.

$74=\<string\> : max 32 characters, default "GRBL".  
Bluetooth device name.

$75=\<string\> : max 32 characters, default "GRBL serial port".  
Bluetooth service name.

$80=\<float\> : default driver dependent.  
Spindle PID regulator proportional gain. Usage is driver dependent.

$81=\<float\> : default driver dependent.  
Spindle PID regulator integral gain. Usage is driver dependent.

$82=\<float\> : default driver dependent.  
Spindle PID regulator derivative gain. Usage is driver dependent.

$84=\<float\> : default driver dependent.  
Spindle PID max output error. Usage is driver dependent.

$85=\<float\> : default driver dependent.  
Spindle PID regulator max integral error. Usage is driver dependent.

$90=\<float\> : default driver dependent.  
Spindle synced motion PID regulator proportional gain. Usage is driver dependent.

$91=\<float\> : default driver dependent.  
Spindle synced motion PID regulator integral gain. Usage is driver dependent.

$92=\<float\> : default driver dependent.  
Spindle synced motion PID regulator derivative gain. Usage is driver dependent.

ENABLE_SOFTWARE_DEBOUNCE
HARD_LIMIT_FORCE_STATE_CHECK
SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED

---
2019-05-18
