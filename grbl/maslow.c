/*
  maslow.c - Maslow router kinematics implementation

  Part of GrblHAL

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

  The basis for this code has been pulled from MaslowDue created by Larry D O'Cull.
  <https://github.com/ldocull/MaslowDue>

  Some portions of that package directly or indirectly has been pulled from from the Maslow CNC
  firmware for Aduino Mega. Those parts are Copyright 2014-2017 Bar Smith.
  <https://www.maslowcnc.com/>

  It has been adapted for grblHAL by Terje Io.
*/

#include "grbl.h"

#ifdef MASLOW_ROUTER

#include <math.h>

#include "driver.h"

#include "settings.h"
#include "planner.h"
#include "nvs_buffer.h"
#include "kinematics.h"
#include "maslow.h"
#include "report.h"

#define A_MOTOR X_AXIS // Must be X_AXIS
#define B_MOTOR Y_AXIS // Must be Y_AXIS

typedef struct {
    float halfWidth;        //Half the machine width
    float halfHeight;       //Half the machine height
    float xCordOfMotor;
    float xCordOfMotor_x4;
    float xCordOfMotor_x2_pow;
    float yCordOfMotor;
    float height_to_bit; //distance between sled attach point and bit
} machine_t;

static machine_t machine = {0};

uint_fast8_t selected_motor = A_MOTOR;

maslow_hal_t maslow_hal = {0};

static const maslow_settings_t maslow_defaults = {
    .pid[A_MOTOR].Kp = MASLOW_A_KP,
    .pid[A_MOTOR].Ki = MASLOW_A_KI,
    .pid[A_MOTOR].Kd = MASLOW_A_KD,
    .pid[A_MOTOR].Imax = MASLOW_A_IMAX,

    .pid[B_MOTOR].Kp = MASLOW_B_KP,
    .pid[B_MOTOR].Ki = MASLOW_B_KI,
    .pid[B_MOTOR].Kd = MASLOW_B_KD,
    .pid[B_MOTOR].Imax = MASLOW_B_IMAX,

    .pid[Z_AXIS].Kp = MASLOW_Z_KP,
    .pid[Z_AXIS].Ki = MASLOW_Z_KI,
    .pid[Z_AXIS].Kd = MASLOW_Z_KD,
    .pid[Z_AXIS].Imax = MASLOW_Z_IMAX,

    .chainOverSprocket = MASLOW_CHAINOVERSPROCKET,
    .machineWidth = MASLOW_MACHINEWIDTH,
    .machineHeight = MASLOW_MACHINEHEIGHT,
    .distBetweenMotors = MASLOW_DISTBETWEENMOTORS,

    .motorOffsetY = MASLOW_MOTOROFFSETY,
    .chainSagCorrection = MASLOW_CHAINSAGCORRECTION,
    .leftChainTolerance = MASLOW_LEFTCHAINTOLERANCE,
    .rightChainTolerance = MASLOW_RIGHTCHAINTOLERANCE,
    .rotationDiskRadius = MASLOW_ROTATIONDISKRADIUS,

    .chainLength = MASLOW_CHAINLENGTH,
    .sledHeight = MASLOW_SLEDHEIGHT,
    .sledWidth = MASLOW_SLEDWIDTH,

    .XcorrScaling = MASLOW_ACORRSCALING,
    .YcorrScaling = MASLOW_BCORRSCALING
};

status_code_t maslow_setting (setting_type_t param, float value, char *svalue)
{
    status_code_t status = Status_Unhandled;

    // TODO: PID settings should be moved to axis settings range...

    switch((maslow_config_t)param) {

        case Maslow_A_KP:
            status = Status_OK;
            maslow_hal.settings->pid[A_MOTOR].Kp = value;
            break;

        case Maslow_A_KI:
            status = Status_OK;
            maslow_hal.settings->pid[A_MOTOR].Ki = value;
            break;

        case Maslow_A_KD:
            status = Status_OK;
            maslow_hal.settings->pid[A_MOTOR].Kd = value;
            break;

        case Maslow_A_IMAX:
            status = Status_OK;
            maslow_hal.settings->pid[A_MOTOR].Imax = (uint32_t)value;
            break;

        case Maslow_B_KP:
            status = Status_OK;
            maslow_hal.settings->pid[B_MOTOR].Kp = value;
            break;

        case Maslow_B_KI:
            status = Status_OK;
            maslow_hal.settings->pid[B_MOTOR].Ki = value;
            break;

        case Maslow_B_KD:
            status = Status_OK;
            maslow_hal.settings->pid[B_MOTOR].Kd = value;
            break;

        case Maslow_B_IMAX:
            status = Status_OK;
            maslow_hal.settings->pid[B_MOTOR].Imax = (uint32_t)value;
            break;

        case Maslow_Z_KP:
            status = Status_OK;
            maslow_hal.settings->pid[Z_AXIS].Kp = value;
            break;

        case Maslow_Z_KI:
            status = Status_OK;
            maslow_hal.settings->pid[Z_AXIS].Ki = value;
            break;

        case Maslow_Z_KD:
            status = Status_OK;
            maslow_hal.settings->pid[Z_AXIS].Kd = value;
            break;

        case Maslow_Z_IMAX:
            status = Status_OK;
            maslow_hal.settings->pid[Z_AXIS].Imax = (uint32_t)value;
            break;

        case Maslow_chainOverSprocket:
            status = Status_OK;
            maslow_hal.settings->chainOverSprocket = value;
            break;

        case Maslow_machineWidth:
            status = Status_OK;
            maslow_hal.settings->machineWidth = value;
            break;   /* Maslow specific settings */

        case Maslow_machineHeight:
            status = Status_OK;
            maslow_hal.settings->machineHeight = value;
            break;

        case Maslow_distBetweenMotors:
            status = Status_OK;
            maslow_hal.settings->distBetweenMotors = value;
            break;

        case Maslow_motorOffsetY:
            status = Status_OK;
            maslow_hal.settings->motorOffsetY = value;
            break;

        case Maslow_AcorrScaling:
            status = Status_OK;
            maslow_hal.settings->XcorrScaling = value;
            break;

        case Maslow_BcorrScaling:
            status = Status_OK;
            maslow_hal.settings->YcorrScaling = value;
            break;

        default:
            break;
    }

    if(status == Status_OK)
        hal.nvs.memcpy_to_with_checksum(hal.nvs.driver_area.address, (uint8_t *)&driver_settings, sizeof(driver_settings));

    return status;
}

static void do_maslow_settings_report (setting_type_t setting)
{
    switch(setting) {

        // A motor PID

        case (setting_type_t)Maslow_A_KP:
            report_float_setting(setting, maslow_hal.settings->pid[A_MOTOR].Kp, 3);
            break;

        case (setting_type_t)Maslow_A_KI:
            report_float_setting(setting, maslow_hal.settings->pid[A_MOTOR].Ki, 3);
            break;

        case (setting_type_t)Maslow_A_KD:
            report_float_setting(setting, maslow_hal.settings->pid[A_MOTOR].Kd, 3);
            break;

        case (setting_type_t)Maslow_A_IMAX:
            report_uint_setting(setting, maslow_hal.settings->pid[A_MOTOR].Imax);
            break;

        // B motor PID

        case (setting_type_t)Maslow_B_KI:
            report_float_setting(setting, maslow_hal.settings->pid[B_MOTOR].Kp, 3);
            break;

        case (setting_type_t)Maslow_B_KP:
            report_float_setting(setting, maslow_hal.settings->pid[B_MOTOR].Ki, 3);
            break;

        case (setting_type_t)Maslow_B_KD:
            report_float_setting(setting, maslow_hal.settings->pid[B_MOTOR].Kd, 3);
            break;

        case (setting_type_t)Maslow_B_IMAX:
            report_uint_setting(setting, maslow_hal.settings->pid[B_MOTOR].Imax);
            break;

        // Z motor PID

        case (setting_type_t)Maslow_Z_KP:
            report_float_setting(setting, maslow_hal.settings->pid[Z_AXIS].Kp, 3);
            break;

        case (setting_type_t)Maslow_Z_KI:
            report_float_setting(setting, maslow_hal.settings->pid[Z_AXIS].Ki, 3);
            break;

        case (setting_type_t)Maslow_Z_KD:
            report_float_setting(setting, maslow_hal.settings->pid[Z_AXIS].Kd, 3);
            break;

        case (setting_type_t)Maslow_Z_IMAX:
            report_uint_setting(setting, maslow_hal.settings->pid[Z_AXIS].Imax);
            break;

        //

        case (setting_type_t)Maslow_chainOverSprocket:
            report_uint_setting(setting, maslow_hal.settings->chainOverSprocket);
            break;

        case (setting_type_t)Maslow_machineWidth:
            report_float_setting(setting, maslow_hal.settings->machineWidth, N_DECIMAL_SETTINGVALUE);
            break;

        case (setting_type_t)Maslow_machineHeight:
            report_float_setting(setting, maslow_hal.settings->machineHeight, N_DECIMAL_SETTINGVALUE);
            break;

        case (setting_type_t)Maslow_distBetweenMotors:
            report_float_setting(setting, maslow_hal.settings->distBetweenMotors, N_DECIMAL_SETTINGVALUE);
            break;

        case (setting_type_t)Maslow_motorOffsetY:
            report_float_setting(setting, maslow_hal.settings->motorOffsetY, N_DECIMAL_SETTINGVALUE);
            break;

        case (setting_type_t)Maslow_AcorrScaling:
            report_float_setting(setting, maslow_hal.settings->XcorrScaling, N_DECIMAL_SETTINGVALUE);
            break;

        case (setting_type_t)Maslow_BcorrScaling:
            report_float_setting(setting, maslow_hal.settings->YcorrScaling, N_DECIMAL_SETTINGVALUE);
            break;
    }
}

// "Hack" for settings report until Maslow setting ids are moved to settings.h
void maslow_settings_report (setting_type_t setting)
{
    uint_fast32_t idx;
    if(setting == Setting_AxisSettingsMax + 1) {
        for(idx = Maslow_A_KP; idx < Maslow_SettingMax; idx++)
            do_maslow_settings_report((setting_type_t)idx);
    }
}

void maslow_settings_restore (void)
{
    memcpy(&driver_settings.maslow, &maslow_defaults, sizeof(maslow_settings_t));
    hal.nvs.memcpy_to_with_checksum(hal.nvs.driver_area.address, (uint8_t *)&driver_settings, sizeof(driver_settings_t));

}

void recomputeGeometry()
{
    /*
    Some variables are computed on initialization for the geometry of the machine to reduce overhead,
    calling this function regenerates those values.
    */
    machine.halfWidth = (maslow_hal.settings->machineWidth / 2.0f);
    machine.halfHeight = (maslow_hal.settings->machineHeight / 2.0f);
    machine.xCordOfMotor = (maslow_hal.settings->distBetweenMotors / 2.0f);
    machine.yCordOfMotor = (machine.halfHeight + maslow_hal.settings->motorOffsetY);
    machine.xCordOfMotor_x4 = machine.xCordOfMotor * 4.0f;
    machine.xCordOfMotor_x2_pow = powf((machine.xCordOfMotor * 2.0f), 2.0f);
}

// limit motion to stay within table (in mm)
void verifyValidTarget (float* xTarget, float* yTarget)
{
    //If the target point is beyond one of the edges of the board, the machine stops at the edge

    recomputeGeometry();
// no limits for now
//      *xTarget = (*xTarget < -halfWidth) ? -halfWidth : (*xTarget > halfWidth) ? halfWidth : *xTarget;
//      *yTarget = (*yTarget < -halfHeight) ? -halfHeight : (*yTarget > halfHeight) ? halfHeight : *yTarget;

}

// Maslow CNC calculation only. Returns x or y-axis "steps" based on Maslow motor steps.
// converts current position two-chain intersection (steps) into x / y cartesian in STEPS..
static void maslow_convert_array_steps_to_mpos (float *position, int32_t *steps)
{
    float a_len = ((float)steps[A_MOTOR] / settings.axis[A_MOTOR].steps_per_mm);
    float b_len = ((float)steps[B_MOTOR] / settings.axis[B_MOTOR].steps_per_mm);

    a_len = (machine.xCordOfMotor_x2_pow - powf(b_len, 2.0f) + powf(a_len, 2.0f)) / machine.xCordOfMotor_x4;
    position[X_AXIS] = a_len - machine.xCordOfMotor;
    a_len = maslow_hal.settings->distBetweenMotors - a_len;
    position[Y_AXIS] = machine.yCordOfMotor - sqrtf(powf(b_len, 2.0f) - powf(a_len, 2.0f));
    position[Z_AXIS] = steps[Z_AXIS] / settings.axis[Z_AXIS].steps_per_mm;

// back out any correction factor
   position[X_AXIS] /= maslow_hal.settings->XcorrScaling;
   position[Y_AXIS] /= maslow_hal.settings->YcorrScaling;
//
}

// calculate left and right (A_MOTOR/B_MOTOR) chain lengths from X-Y cartesian coordinates  (in mm)
// target is an absolute position in the frame
inline static void triangularInverse (int32_t *target_steps, float *target)
{
    //Confirm that the coordinates are on the table
//    verifyValidTarget(&xTarget, &yTarget);

    // scale target (absolute position) by any correction factor
    double xxx = (double)target[A_MOTOR] * (double)maslow_hal.settings->XcorrScaling;
    double yyy = (double)target[B_MOTOR] * (double)maslow_hal.settings->YcorrScaling;
    double yyp = pow((double)machine.yCordOfMotor - yyy, 2.0);

    //Calculate motor axes length to the bit
    target_steps[A_MOTOR] = (int32_t)lround(sqrt(pow((double)machine.xCordOfMotor + xxx, 2.0f) + yyp) * settings.axis[A_MOTOR].steps_per_mm);
    target_steps[B_MOTOR] = (int32_t)lround(sqrt(pow((double)machine.xCordOfMotor - xxx, 2.0f) + yyp) * settings.axis[B_MOTOR].steps_per_mm);
}

// Transform absolute position from cartesian coordinate system (mm) to maslow coordinate system (step)
static void maslow_target_to_steps (int32_t *target_steps, float *target)
{
    uint_fast8_t idx = N_AXIS - 1;

    do {
        target_steps[idx] = lroundf(target[idx] * settings.axis[idx].steps_per_mm);
    } while(--idx > Y_AXIS);

    triangularInverse(target_steps, target);
}

static uint_fast8_t maslow_limits_get_axis_mask (uint_fast8_t idx)
{
    return ((idx == A_MOTOR) || (idx == B_MOTOR)) ? (bit(X_AXIS) | bit(Y_AXIS)) : bit(idx);
}

// MASLOW is circular in motion, so long lines must be divided up
static bool maslow_segment_line (float *target, plan_line_data_t *pl_data, bool init)
{
    static uint_fast16_t iterations;
    static bool segmented;
    static float delta[N_AXIS], segment_target[N_AXIS];
//    static plan_line_data_t plan;

    uint_fast8_t idx = N_AXIS;

    if(init) {

        float max_delta = 0.0f;

        do {
            idx--;
            delta[idx] = target[idx] - gc_state.position[idx];
            max_delta = max(max_delta, fabsf(delta[idx]));
        } while(idx);

        if((segmented = !(pl_data->condition.rapid_motion || pl_data->condition.jog_motion) &&
                          max_delta > MAX_SEG_LENGTH_MM && !(delta[X_AXIS] == 0.0f && delta[Y_AXIS] == 0.0f))) {

            idx = N_AXIS;
            iterations = (uint_fast16_t)ceilf(max_delta / MAX_SEG_LENGTH_MM);

            memcpy(segment_target, gc_state.position, sizeof(segment_target));
//            memcpy(&plan, pl_data, sizeof(plan_line_data_t));

            do {
                delta[--idx] /= (float)iterations;
                target[idx] = gc_state.position[idx];
            } while(idx);

        } else
            iterations = 1;

        iterations++; // return at least one iteration

    } else {

        iterations--;

        if(segmented && iterations) do {
            idx--;
            segment_target[idx] += delta[idx];
            target[idx] = segment_target[idx];
//            memcpy(pl_data, &plan, sizeof(plan_line_data_t));
        } while(idx);

    }

    return iterations != 0;
}

static void maslow_limits_set_target_pos (uint_fast8_t idx) // fn name?
{
    /*
    int32_t axis_position;
    float position[3];
    maslow_convert_array_steps_to_mpos(position, sys_position);

    float aCl,bCl;    // set initial chain lengths to table center when $HOME
    void triangularInverse(float ,float , float* , float* );

    x_axis.axis_Position = 0;
    x_axis.target = 0;
    x_axis.target_PS = 0;
    x_axis.Integral = 0;
    y_axis.axis_Position = 0;
    y_axis.target = 0;
    y_axis.target_PS = 0;
    y_axis.Integral = 0;
    z_axis.axis_Position = 0;
    z_axis.target = 0;
    z_axis.target_PS = 0;
    z_axis.Integral = 0;
    set_axis_position = 0;    // force to center of table -- its a Maslow thing

    triangularInverse((float)(set_axis_position), (float)(set_axis_position), &aCl, &bCl);
    sys_position[A_MOTOR] = (int32_t) lround(aCl * settings.steps_per_mm[A_MOTOR]);
    sys_position[B_MOTOR] = (int32_t) lround(bCl * settings.steps_per_mm[B_MOTOR]);
    sys_position[Z_AXIS] = set_axis_position;

    store_current_machine_pos();    // reset all the way out to stored space
    sys.step_control = STEP_CONTROL_NORMAL_OP; // Return step control to normal operation.
    return;

  sys_position[idx] = set_axis_position;

    switch(idx) {
        case X_AXIS:
            axis_position = system_convert_maslow_to_y_axis_steps(sys_position);
            sys_position[A_MOTOR] = axis_position;
            sys_position[B_MOTOR] = -axis_position;
            break;
        case Y_AXIS:
            sys_position[A_MOTOR] = sys_position[B_MOTOR] = system_convert_maslow_to_x_axis_steps(sys_position);
            break;
        default:
            sys_position[idx] = 0;
            break;
    }
    */
}

// Set machine positions for homed limit switches. Don't update non-homed axes.
// NOTE: settings.max_travel[] is stored as a negative value.
static void maslow_limits_set_machine_positions (axes_signals_t cycle)
{
    /*
     *     uint_fast8_t idx = N_AXIS;

    if(settings.homing.flags.force_set_origin) {
        if (cycle.mask & bit(--idx)) do {
            switch(--idx) {
                case X_AXIS:
                    sys_position[A_MOTOR] = system_convert_maslow_to_y_axis_steps(sys_position);
                    sys_position[B_MOTOR] = - sys_position[A_MOTOR];
                    break;
                case Y_AXIS:
                    sys_position[A_MOTOR] = system_convert_maslow_to_x_axis_steps(sys_position);
                    sys_position[B_MOTOR] = sys_position[A_MOTOR];
                    break;
                default:
                    sys_position[idx] = 0;
                    break;
            }
        } while (idx);
    } else do {
         if (cycle.mask & bit(--idx)) {
             int32_t off_axis_position;
             int32_t set_axis_position = bit_istrue(settings.homing.dir_mask.value, bit(idx))
                                          ? lroundf((settings.max_travel[idx] + settings.homing.pulloff) * settings.steps_per_mm[idx])
                                          : lroundf(-settings.homing.pulloff * settings.steps_per_mm[idx]);
             switch(idx) {
                 case X_AXIS:
                     off_axis_position = system_convert_maslow_to_y_axis_steps(sys_position);
                     sys_position[A_MOTOR] = set_axis_position + off_axis_position;
                     sys_position[B_MOTOR] = set_axis_position - off_axis_position;
                     break;
                 case Y_AXIS:
                     off_axis_position = system_convert_maslow_to_x_axis_steps(sys_position);
                     sys_position[A_MOTOR] = off_axis_position + set_axis_position;
                     sys_position[B_MOTOR] = off_axis_position - set_axis_position;
                     break;
                 default:
                     sys_position[idx] = set_axis_position;
                     break;
             }
         }
    } while(idx);
    */
}

// TODO: format output in grbl fashion: [...]
status_code_t maslow_tuning (uint_fast16_t state, char *line, char *lcline)
{
    status_code_t retval = Status_OK;

    if(line[1] == 'M') switch(line[2]) {

        case 'C': // commit driver setting changes to non-volatile storage
            settings_dirty.is_dirty = settings_dirty.driver_settings = true;
            break;

        case 'X':
            selected_motor = A_MOTOR;
            hal.stream.write("X-Axis Selected" ASCII_EOL);
            break;

        case 'Y':
            selected_motor = B_MOTOR;
            hal.stream.write("Y-Axis Selected" ASCII_EOL);
            break;

        case 'Z':
            selected_motor = Z_AXIS;
            if(maslow_hal.get_debug_data(selected_motor))
                hal.stream.write("Z-Axis Selected" ASCII_EOL);
            else {
                selected_motor = A_MOTOR;
                hal.stream.write("Z-Axis is not PID controlled, switched to A motor" ASCII_EOL);
            }
            break;

        case 'G':
            maslow_hal.pos_enable(true);
            break;

        case 'R':   // reset current position
            maslow_hal.reset_pid(selected_motor);
            break;

        case '+':   // Move
            maslow_hal.move(selected_motor, 10000);
            break;

        case '-':   // Move
            maslow_hal.move(selected_motor, -10000);
            break;

        case '*':   // Move
            maslow_hal.move(selected_motor, 10);
            break;

        case '/':   // Move
            maslow_hal.move(selected_motor, -10);
            break;

        case 'I':
        case 'M':
        case 'D':
        case 'P':
        case 'S':
        case 'A':;
            if(line[3] == '=' && line[4] != '\0') {
                float parameter;
                uint_fast8_t counter = 4;

                if(!read_float(line, &counter, &parameter))
                    retval = Status_BadNumberFormat;

                else switch(line[2]) {

                    case 'P':
                        maslow_hal.settings->pid[selected_motor].Kp = parameter;
                        hal.stream.write("Kp == ");
                        hal.stream.write(ftoa(maslow_hal.settings->pid[selected_motor].Kp, 3));
                        hal.stream.write(ASCII_EOL);
                        break;

                    case 'D':
                        maslow_hal.settings->pid[selected_motor].Kd = parameter;
                        hal.stream.write("Kd == ");
                        hal.stream.write(ftoa(maslow_hal.settings->pid[selected_motor].Kd, 3));
                        hal.stream.write(ASCII_EOL);
                        break;

                    case 'I':
                        maslow_hal.settings->pid[selected_motor].Ki = parameter;
                        hal.stream.write("Ki == ");
                        hal.stream.write(ftoa(maslow_hal.settings->pid[selected_motor].Ki, 3));
                        hal.stream.write(ASCII_EOL);
                        maslow_hal.pid_settings_changed(selected_motor);
                        break;

                    case 'M':
                        maslow_hal.settings->pid[selected_motor].Imax = parameter;
                        hal.stream.write("Imax == ");
                        hal.stream.write(ftoa(maslow_hal.settings->pid[selected_motor].Imax, 3));
                        hal.stream.write(ASCII_EOL);
                        maslow_hal.pid_settings_changed(selected_motor);
                        break;

                    case 'S':
                        {
                            maslow_hal.tuning_enable(true);
                            int32_t sz = maslow_hal.set_step_size(selected_motor, (int32_t)parameter);
                            hal.stream.write("S == ");
                            hal.stream.write(ftoa((float)sz, 0));
                            hal.stream.write(ASCII_EOL);
                        }
                        break;

                    case 'A': // test kinematics - from X,Y mm to A,B steps back to X,Y mm
                        {
                            float xyz[N_AXIS];
                            int32_t abz[N_AXIS];
                            recomputeGeometry();
                            xyz[X_AXIS] = parameter;
                            if(line[counter++] == ',' && line[counter] != '\0') {
                                if(!read_float(line, &counter, &xyz[Y_AXIS]))
                                    retval = Status_BadNumberFormat;
                            } else
                                retval = Status_BadNumberFormat;

                            if(retval == Status_OK) {

                                triangularInverse(abz, xyz);
                                hal.stream.write("[KINEMATICSTRANSFORM: X,Y = ");
                                hal.stream.write(ftoa(xyz[X_AXIS], 3));
                                hal.stream.write(",");
                                hal.stream.write(ftoa(xyz[Y_AXIS], 3));
                                hal.stream.write(" -> A,B steps: ");
                                hal.stream.write(uitoa((uint32_t)abz[A_MOTOR]));
                                hal.stream.write(",");
                                hal.stream.write(uitoa((uint32_t)abz[B_MOTOR]));

                                maslow_convert_array_steps_to_mpos(xyz, abz);
                                hal.stream.write(" -> X,Y = ");
                                hal.stream.write(ftoa(xyz[X_AXIS], 3));
                                hal.stream.write(",");
                                hal.stream.write(ftoa(xyz[Y_AXIS], 3));
                                hal.stream.write("]" ASCII_EOL);
                            }
                        }
                        break;
                }
            } else
                retval = Status_BadNumberFormat;
            break;

        default:
            {
               maslow_debug_t *debug = maslow_hal.get_debug_data(selected_motor);

               hal.stream.write("[AXISPID:");
               hal.stream.write(axis_letter[selected_motor]);
               hal.stream.write(": Kp = ");
               hal.stream.write(ftoa(maslow_hal.settings->pid[selected_motor].Kp, 3));
               hal.stream.write(" Ki = ");
               hal.stream.write(ftoa(maslow_hal.settings->pid[selected_motor].Ki, 3));
               hal.stream.write(" Kd = ");
               hal.stream.write(ftoa(maslow_hal.settings->pid[selected_motor].Kd, 3));
               hal.stream.write(" Imax = ");
               hal.stream.write(ftoa(maslow_hal.settings->pid[selected_motor].Imax, 3));

               hal.stream.write("]\r\n[PIDDATA:err=");
               hal.stream.write(ftoa(debug->Error, 0));
               hal.stream.write("\t\ti=");
               hal.stream.write(ftoa(debug->Integral, 0));
               hal.stream.write("\tiT=");
               hal.stream.write(ftoa(debug->iterm, 0));
               hal.stream.write("\td=");
               hal.stream.write(ftoa(debug->DiffTerm, 0));
    //           hal.stream.write("\tV=");
    //           hal.stream.write(ftoa(debug->totalSpeed, 0));
               hal.stream.write("\txCMD=");
               hal.stream.write(ftoa(debug->speed, 0));
    //           hal.stream.write("\tyCMD=");
    //           hal.stream.write(uitoa(motor[Y_AXIS]->speed));
    //           hal.stream.write("\tzCMD=");
    //           hal.stream.write(uitoa(motor[Z_AXIS]->speed));
               hal.stream.write("]" ASCII_EOL);
            }
           break;
    } else
        retval = Status_Unhandled;

    return retval;
}

// Initialize API pointers & machine parameters for Maslow router kinematics
void maslow_init (void)
{
    float xy[2] = {0.0f, 0.0f};

    recomputeGeometry();
    triangularInverse(sys_position, xy);

    selected_motor = A_MOTOR;

    kinematics.limits_set_target_pos = maslow_limits_set_target_pos;
    kinematics.limits_get_axis_mask = maslow_limits_get_axis_mask;
    kinematics.limits_set_machine_positions = maslow_limits_set_machine_positions;
    kinematics.plan_target_to_steps = maslow_target_to_steps;
    kinematics.convert_array_steps_to_mpos = maslow_convert_array_steps_to_mpos;
    kinematics.segment_line = maslow_segment_line;

    grbl.on_unknown_sys_command = maslow_tuning;
}

#endif

