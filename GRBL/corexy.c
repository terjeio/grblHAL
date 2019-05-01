/*
  corexy.c - corexy kinematics implementation
  Part of Grbl

  Copyright (c) 2019 Terje Io
  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC

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

#ifdef COREXY

// CoreXY motor assignments. DO NOT ALTER.
// NOTE: If the A and B motor axis bindings are changed, this effects the CoreXY equations.
#define A_MOTOR X_AXIS // Must be X_AXIS
#define B_MOTOR Y_AXIS // Must be Y_AXIS

// Returns x or y-axis "steps" based on CoreXY motor steps.
inline static int32_t corexy_convert_to_x_axis_steps (int32_t *steps)
{
    return (steps[A_MOTOR] + steps[B_MOTOR]) >> 1;
}

inline static int32_t corexy_convert_to_y_axis_steps (int32_t *steps)
{
    return (steps[A_MOTOR] - steps[B_MOTOR]) >> 1;
}


// Returns machine position of axis 'idx'. Must be sent a 'step' array.
// NOTE: If motor steps and machine position are not in the same coordinate frame, this function
//   serves as a central place to compute the transformation.
static float corexy_convert_axis_steps_to_mpos (int32_t *steps, uint_fast8_t idx)
{
    return (float)(idx == X_AXIS ? corexy_convert_to_x_axis_steps(steps) : (idx == Y_AXIS ? corexy_convert_to_y_axis_steps(steps) : steps[idx])) / settings.steps_per_mm[idx];
}


/* Add a new linear movement to the buffer. target[N_AXIS] is the signed, absolute target position
   in millimeters. Feed rate specifies the speed of the motion. If feed rate is inverted, the feed
   rate is taken to mean "frequency" and would complete the operation in 1/feed_rate minutes.
   All position data passed to the planner must be in terms of machine position to keep the planner
   independent of any coordinate system changes and offsets, which are handled by the g-code parser.
   NOTE: Assumes buffer is available. Buffer checks are handled at a higher level by motion_control.
   In other words, the buffer head is never equal to the buffer tail.  Also the feed rate input value
   is used in three ways: as a normal feed rate if invert_feed_rate is false, as inverse time if
   invert_feed_rate is true, or as seek/rapids rate if the feed_rate value is negative (and
   invert_feed_rate always false).
   The system motion condition tells the planner to plan a motion in the always unused block buffer
   head. It avoids changing the planner state and preserves the buffer to ensure subsequent gcode
   motions are still planned correctly, while the stepper module only points to the block buffer head
   to execute the special system motion. */

static bool corexy_plan_copy_pos (bool system_motion, float *target, int32_t *position_steps, int32_t *target_steps)
{
    if (system_motion) {
        position_steps[X_AXIS] = corexy_convert_to_x_axis_steps(sys_position);
        position_steps[Y_AXIS] = corexy_convert_to_y_axis_steps(sys_position);
        position_steps[Z_AXIS] = sys_position[Z_AXIS];
    }

    target_steps[A_MOTOR] = lroundf(target[A_MOTOR] * settings.steps_per_mm[A_MOTOR]);
    target_steps[B_MOTOR] = lroundf(target[B_MOTOR] * settings.steps_per_mm[B_MOTOR]);

    return system_motion;
}


static int32_t corexy_plan_calc_pos (uint_fast8_t idx, float *target, int32_t *position_steps, int32_t *target_steps)
{
    int32_t delta_steps;
    // Calculate target position in absolute steps, number of steps for each axis, and determine max step events.
    // Also, compute individual axes distance for move and prep unit vector calculations.
    // NOTE: Computes true distance from converted step values.
    switch(idx) {
        case X_AXIS:
            delta_steps = (target_steps[X_AXIS] - position_steps[X_AXIS]) + (target_steps[Y_AXIS] - position_steps[Y_AXIS]);
            break;
        case Y_AXIS:
            delta_steps = (target_steps[X_AXIS] - position_steps[X_AXIS]) - (target_steps[Y_AXIS] - position_steps[Y_AXIS]);
            break;
        default:
            target_steps[idx] = lroundf(target[idx] * settings.steps_per_mm[idx]);
            delta_steps = target_steps[idx] - position_steps[idx];
            break;
    }
    return delta_steps;
}


// Reset the planner position vectors. Called by the system abort/initialization routine.
static void corexy_plan_sync_position (planner_t *pl)
{
  // TODO: For motor configurations not in the same coordinate frame as the machine position,
  // this function needs to be updated to accomodate the difference.
    uint_fast8_t idx = N_AXIS;
    do {
        switch(--idx) {
            case X_AXIS:
                pl->position[X_AXIS] = corexy_convert_to_x_axis_steps(sys_position);
                break;
            case Y_AXIS:
                pl->position[Y_AXIS] = corexy_convert_to_y_axis_steps(sys_position);
                break;
            default:
                pl->position[idx] = sys_position[idx];
                break;
        }
    } while (idx);
}


static uint_fast8_t corexy_limits_get_axis_mask (uint_fast8_t idx)
{
    return ((idx == A_MOTOR) || (idx == B_MOTOR)) ? (bit(X_AXIS) | bit(Y_AXIS)) : bit(idx);
}


static void corexy_limits_set_target_pos (uint_fast8_t idx) // fn name?
{
    int32_t axis_position;

    switch(idx) {
        case X_AXIS:
            axis_position = corexy_convert_to_y_axis_steps(sys_position);
            sys_position[A_MOTOR] = axis_position;
            sys_position[B_MOTOR] = -axis_position;
            break;
        case Y_AXIS:
            sys_position[A_MOTOR] = sys_position[B_MOTOR] = corexy_convert_to_x_axis_steps(sys_position);
            break;
        default:
            sys_position[idx] = 0;
            break;
    }
}


// Set machine positions for homed limit switches. Don't update non-homed axes.
// NOTE: settings.max_travel[] is stored as a negative value.
static void corexy_limits_set_machine_positions (uint8_t cycle_mask)
{
    uint_fast8_t idx = N_AXIS;

    if(settings.flags.homing_force_set_origin) {
        if (cycle_mask & bit(--idx)) do {
            switch(--idx) {
                case X_AXIS:
                    sys_position[A_MOTOR] = corexy_convert_to_y_axis_steps(sys_position);
                    sys_position[B_MOTOR] = - sys_position[A_MOTOR];
                    break;
                case Y_AXIS:
                    sys_position[A_MOTOR] = corexy_convert_to_x_axis_steps(sys_position);
                    sys_position[B_MOTOR] = sys_position[A_MOTOR];
                    break;
                default:
                    sys_position[idx] = 0;
                    break;
            }
        } while (idx);
    } else do {
         if (cycle_mask & bit(--idx)) {
             int32_t off_axis_position;
             int32_t set_axis_position = bit_istrue(settings.homing.dir_mask.value, bit(idx))
                                          ? lroundf((settings.max_travel[idx] + settings.homing.pulloff) * settings.steps_per_mm[idx])
                                          : lroundf(-settings.homing.pulloff * settings.steps_per_mm[idx]);
             switch(idx) {
                 case X_AXIS:
                     off_axis_position = corexy_convert_to_y_axis_steps(sys_position);
                     sys_position[A_MOTOR] = set_axis_position + off_axis_position;
                     sys_position[B_MOTOR] = set_axis_position - off_axis_position;
                     break;
                 case Y_AXIS:
                     off_axis_position = corexy_convert_to_x_axis_steps(sys_position);
                     sys_position[A_MOTOR] = off_axis_position + set_axis_position;
                     sys_position[B_MOTOR] = off_axis_position - set_axis_position;
                     break;
                 default:
                     sys_position[idx] = set_axis_position;
                     break;
             }
         }
    } while(idx);
}


// Initialize HAL pointers for CoreXY kinematics
void corexy_init (void)
{
    hal.kinematics.limits_set_target_pos = corexy_limits_set_target_pos;
    hal.kinematics.limits_get_axis_mask = corexy_limits_get_axis_mask;
    hal.kinematics.limits_set_machine_positions = corexy_limits_set_machine_positions;
    hal.kinematics.plan_sync_position = corexy_plan_sync_position;
    hal.kinematics.plan_copy_position = corexy_plan_copy_pos;
    hal.kinematics.plan_calc_position = corexy_plan_calc_pos;
    hal.kinematics.system_convert_axis_steps_to_mpos = corexy_convert_axis_steps_to_mpos;
}

#endif

