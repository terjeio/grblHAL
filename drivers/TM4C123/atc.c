/*
  atc.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for my ATC

  Part of GrblHAL

  Copyright (c) 2018-2019 Terje Io

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

#include "grbl/grbl.h"

typedef struct {
    float x;
    float y;
} pos_t;

static const float r1 = 10.0f, r2 = 20.0f;
static tool_data_t *current_tool = 0, *next_tool = 0;
static coord_data_t offset;

void atc_tool_select (uint8_t tool)
{
#ifdef N_TOOLS
    current_tool = &tool_table[tool];
#endif
}

void atc_tool_selected (tool_data_t *tool)
{
    next_tool = tool;
    // check if tool->tool is not 0 (undefined)? here or in gcode.c?
}

static void atc_move (coord_data_t position, plan_line_data_t *plan_data)
{
    uint_fast8_t idx = N_AXIS;

    do {
        idx--;
        position.values[idx] += offset.values[idx];
    } while(idx);

    mc_line(position.values, plan_data);
}

void atc_tool_change (parser_state_t *gc_state)
{
    if(next_tool != current_tool) {

        float angle;
        plan_line_data_t plan_data;
        coord_data_t target;

        memset(&target, 0, sizeof(target)); // Zero plan_data struct
        memset(&plan_data, 0, sizeof(plan_line_data_t)); // Zero plan_data struct
        settings_read_coord_data(8, &offset.values); // G59.3 - fail if not set?

        hal.spindle_set_state((spindle_state_t){0}, 0.0f);
        hal.coolant_set_state((coolant_state_t){0});
        mc_dwell(1.0);

        plan_data.condition.rapid_motion = On;

        // put current tool back
        angle = 0.25f * M_PI * (float)(current_tool->tool - 1);
        target.z = 15.0f;
        atc_move(target, &plan_data);
        target.x = r1 * sinf(angle);
        target.y = r1 * cosf(angle);
        atc_move(target, &plan_data);
        target.z = 10.0f;
        atc_move(target, &plan_data);
        target.x = r2 * sinf(angle);
        target.y = r2 * cosf(angle);
        atc_move(target, &plan_data);
        target.z = 15.0f;
        atc_move(target, &plan_data);

        mc_dwell(1.0);

        // set next as current and fetch it
        current_tool = next_tool;
        angle = 0.25f * M_PI * (float)(current_tool->tool - 1);
        target.z = 15.0f;
        atc_move(target, &plan_data);
        target.x = r2 * sinf(angle);
        target.y = r2 * cosf(angle);
        atc_move(target, &plan_data);
        target.z = 10.0f;
        atc_move(target, &plan_data);
        target.x = r1 * sinf(angle);
        target.y = r1 * cosf(angle);
        atc_move(target, &plan_data);
        target.z = 15.0f;
        atc_move(target, &plan_data);

        mc_dwell(1.0);

        mc_line(gc_state->position, &plan_data);

        spindle_sync(gc_state->modal.spindle, gc_state->spindle.rpm);
        coolant_sync(gc_state->modal.coolant);

        mc_dwell(1.0);

    }
}
