/*
  atc.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for my Mini Mill ATC, 8 tools arranged in a circle
  A motorized socket wrench is mounted in the center, used for opening/closing the spindle nut

  Part of grblHAL

  Copyright (c) 2018-2020 Terje Io

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

#include <msp.h>
#include <math.h>
#include <string.h>

#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/motion_control.h"

#define ATC_I2C_ADDRESS (0x4A)

typedef struct {
    float x;
    float y;
} pos_t;

typedef enum {
    CMD_Version = 0,
    CMD_Motor,
    CMD_Latch,
    CMD_SetCurrent,
    CMD_GetState
} atc_command_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t nut_locked   :1,
                nut_unlocked :1,
                spindle_locked :1,
                spindle_unlocked :1;
    };
} atc_state_t;

typedef enum {
    Motor_Off = 0,
    Motor_CW = 1,
    Motor_CCW = 2
} atc_motor_state_t;

typedef struct {
    uint8_t addr;
    volatile int16_t count;
    uint8_t *data;
    atc_command_t command;
} i2c_trans_t;

static i2c_trans_t i2c;

static uint16_t current = 100; //856;
static const float r1 = 22.0f, r2 = 31.25f, z_nut = 5.0f, z_tools = 40.0f, z_clear = 40.0f, z_base = 0.0f, z_tool_clearance = 17.0f;
static tool_data_t *current_tool = NULL, *next_tool = NULL;
static coord_data_t offset;
static driver_reset_ptr driver_reset = NULL;

static void StartI2C (bool read)
{
    bool single = i2c.count == 1;

    EUSCI_B1->I2CSA = i2c.addr;                                         // Set EEPROM address and MSB part of data address
    EUSCI_B1->IFG &= ~(EUSCI_B_IFG_TXIFG0|EUSCI_B_IFG_RXIFG0);          // Clear interrupt flags
    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TR|EUSCI_B_CTLW0_TXSTT;            // Transmit start condition and address
    while(!(EUSCI_B1->IFG & EUSCI_B_IFG_TXIFG0));                       // Wait for TX
    EUSCI_B1->TXBUF = i2c.command;                                      // Transmit data address LSB
//    EUSCI_B1->IFG &= ~EUSCI_B_IFG_TXIFG0;                               // Clear TX interrupt flag and
    while(!(EUSCI_B1->IFG & EUSCI_B_IFG_TXIFG0));                       // wait for transmit complete

    if(read) {                                                          // Read data from EEPROM:
        EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTP;                         // Transmit STOP condtition
        while (EUSCI_B1->CTLW0 & EUSCI_B_CTLW0_TXSTP);                  // and wait for it to complete
        EUSCI_B1->CTLW0 &= ~EUSCI_B_CTLW0_TR;                           // Set read mode
        if(single)                                                      // and issue
            EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTT|EUSCI_B_CTLW0_TXSTP; // restart and stop condition if single byte read
        else                                                            // else
            EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTT;                     // restart condition only

        while(i2c.count) {                                              // Read data...
            if(!single && i2c.count == 1) {
                EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
                while (EUSCI_B1->CTLW0 & EUSCI_B_CTLW0_TXSTP) {
                    while(!(EUSCI_B1->IFG & EUSCI_B_IFG_RXIFG0));
                }
            } else
                while(!(EUSCI_B1->IFG & EUSCI_B_IFG_RXIFG0));
            i2c.count--;
            *i2c.data++ = EUSCI_B1->RXBUF;
        }
    } else {                                                            // Write data to EEPROM:
        while (i2c.count--) {
            EUSCI_B1->TXBUF = *i2c.data++;
            while(!(EUSCI_B1->IFG & EUSCI_B_IFG_TXIFG0));
        }
        EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTP;                         // I2C stop condition
 //       WaitForACK();
        hal.delay_ms(5, 0);                                             // Wait a bit for the write cycle to complete
    }
    while (EUSCI_B1->CTLW0 & EUSCI_B_CTLW0_TXSTP);                      // Ensure stop condition got sent
}

static void start_motor(atc_motor_state_t state)
{
    i2c.addr = ATC_I2C_ADDRESS;
    i2c.count = 1;
    i2c.command = CMD_Motor;
    i2c.data = &state;

    StartI2C(false);
}

static void lock_spindle(bool lock)
{
    i2c.addr = ATC_I2C_ADDRESS;
    i2c.count = 1;
    i2c.command = CMD_Latch;
    i2c.data = (uint8_t *)&lock;

    StartI2C(false);
}

static atc_state_t atc_state (void)
{
    atc_state_t state;

    i2c.addr = ATC_I2C_ADDRESS;
    i2c.count = 1;
    i2c.command = CMD_GetState;
    i2c.data = &state.value;

    StartI2C(true);

    return state;
}

static void atc_reset (void)
{
    lock_spindle(false);
    start_motor(Motor_Off);

    driver_reset();
}

static bool atc_move (coord_data_t position, plan_line_data_t *plan_data)
{
    uint_fast8_t idx = N_AXIS;

    do {
        idx--;
        position.values[idx] += offset.values[idx];
    } while(idx);

    return mc_line(position.values, plan_data);
}

static bool spindle_nut (plan_line_data_t *plan_data, float zpos, bool open)
{
    coord_data_t target;

    memset(&target, 0, sizeof(target)); // Zero plan_data struct

    // move to z clearance
    target.z = zpos;
    atc_move(target, plan_data);

    // spin up spindle briefely and lock spindle
    hal.spindle.set_state((spindle_state_t){ .on = On, .ccw = Off }, 100.0f);
    hal.delay_ms(500, NULL);
    hal.spindle.set_state((spindle_state_t){0}, 0.0f);
    lock_spindle(true);
    do {
        hal.delay_ms(50, NULL);
        if(!protocol_execute_realtime())
            return false;
    } while(!atc_state().spindle_locked);

    // move to just above socket wrench
    target.z = z_nut + 5.0f;
    atc_move(target, plan_data);
    protocol_buffer_synchronize();

    // start socket wrench motor
    start_motor(open ? Motor_CW : Motor_CCW);

    // engage socket wrench
    plan_data->condition.rapid_motion = Off;

    target.z = z_nut;
    atc_move(target, plan_data);
    protocol_buffer_synchronize();

    // wait for nut open/closed event
    do {
        hal.delay_ms(50, NULL);
        if(!protocol_execute_realtime())
            return false;
    } while(atc_state().nut_locked == open);

    // unlock spindle
    lock_spindle(false);
    do {
        hal.delay_ms(50, NULL);
        if(!protocol_execute_realtime())
            return false;
    } while(atc_state().spindle_locked);

    // move out of nut, to tool clearance
    plan_data->condition.rapid_motion = On;
    target.z = z_tools;
    atc_move(target, plan_data);

    return true;
}

static void atc_tool_select (tool_data_t *tool, bool next)
{
    if(next)
        next_tool = tool;
    else
        current_tool = tool;
}

static status_code_t atc_tool_change (parser_state_t *gc_state)
{
    if(current_tool == NULL || next_tool == NULL)
        return Status_GCodeToolError;

    if(current_tool == next_tool)
        return Status_OK;

    if(!sys.homed.mask || sys.homed.mask != sys.homing.mask)
        return Status_HomingRequired;

    //good to go?
    if(atc_state().value != 0)
        return Status_GCodeToolError;

    float angle;
    plan_line_data_t plan_data = {0};
    coord_data_t target = {0}, previous;

    i2c.addr = ATC_I2C_ADDRESS;
    i2c.count = 2;
    i2c.command = CMD_SetCurrent;
    i2c.data = (uint8_t *)&current;

    StartI2C(false);

    // Save current position
    system_convert_array_steps_to_mpos(previous.values, sys.position);

    // G59.3 contains offsets to position of socket wrench center (X, Y) and spindle nut offset above ATC base plate
    settings_read_coord_data(CoordinateSystem_G59_3, &offset.values); // G59.3 - fail if not set?

    // Stop spindle and coolant
    hal.spindle.set_state((spindle_state_t){0}, 0.0f);
    hal.coolant.set_state((coolant_state_t){0});

    plan_data.feed_rate = 100.0f;
    plan_data.condition.rapid_motion = On;

    // Initial move to safe Z above socket wrench
    if(z_clear + offset.values[Z_AXIS] < previous.z) {
        target.x += offset.x;
        target.y += offset.y;
        target.z = previous.z;
    } else {
        target.x = previous.x;
        target.y = previous.y;
        target.z = z_clear + offset.values[Z_AXIS];
    }
    if(!mc_line(target.values, &plan_data))
        return Status_Reset;

    // Disengage (open) spindle nut
    if(!spindle_nut(&plan_data, z_clear, true))
        return Status_Reset;

    // put current tool back
    angle = 0.25f * M_PI * (float)(current_tool->tool - 1);

    target.z = z_tools;
    target.x = r1 * sinf(angle);
    target.y = r1 * cosf(angle);
    if(!atc_move(target, &plan_data))
        return Status_Reset;

    target.z = z_base;
    // Trinamic 2130 - monitor stepper current?
    if(!atc_move(target, &plan_data))
        return Status_Reset;

    target.x = r2 * sinf(angle);
    target.y = r2 * cosf(angle);
    if(!atc_move(target, &plan_data))
        return Status_Reset;

    target.z = z_tool_clearance;
    if(!atc_move(target, &plan_data))
        return Status_Reset;

    // set next tool as current and fetch it
    current_tool = next_tool;

    // intermediate move to center of socket wrench
    target.x = 0.0f;
    target.y = 0.0f;
    if(!atc_move(target, &plan_data))
        return Status_Reset;

    // move to tool
    angle = 0.25f * M_PI * (float)(current_tool->tool - 1);
    target.x = r2 * sinf(angle);
    target.y = r2 * cosf(angle);
    if(!atc_move(target, &plan_data))
        return Status_Reset;

    // Spin up spindle
    protocol_buffer_synchronize();
    hal.spindle.set_state((spindle_state_t){ .on = On, .ccw = Off }, 100.0f);
    hal.delay_ms(200, NULL);

    // Engage tool
    // Trinamic 2130 - monitor stepper current?
    target.z = z_tool_clearance - 5.0f;
    plan_data.condition.rapid_motion = Off;
    if(!atc_move(target, &plan_data))
        return Status_Reset;

    protocol_buffer_synchronize();
    hal.spindle.set_state((spindle_state_t){0}, 0.0f);
    hal.delay_ms(200, NULL);
    plan_data.condition.rapid_motion = On;

    target.z = z_base + 0.5f; // a bit over the base to ensure proper return
    plan_data.condition.rapid_motion = On;
    if(!atc_move(target, &plan_data))
        return Status_Reset;

    // release
    target.x = r1 * sinf(angle);
    target.y = r1 * cosf(angle);
    if(!atc_move(target, &plan_data))
        return Status_Reset;

    // and remove it
    target.z = z_tools;
    plan_data.condition.rapid_motion = On;
    if(!atc_move(target, &plan_data))
        return Status_Reset;

    protocol_buffer_synchronize();

    // Tigthen spindle nut
    if(!spindle_nut(&plan_data, z_tools, false))
        return Status_Reset;

    // probe cycle...?

    // go back to previous position

    if(z_clear + offset.values[Z_AXIS] < previous.z) {
        target.x = offset.x;
        target.y = offset.y;
        target.z = previous.z;
    } else {
        target.x = previous.x;
        target.y = previous.y;
        target.z = z_clear + offset.values[Z_AXIS];
    }
    if(!mc_line(target.values, &plan_data))
        return Status_Reset;

    if(!mc_line(previous.values, &plan_data))
        return Status_Reset;

    // Restore coolant and spindle state
    coolant_sync(gc_state->modal.coolant);
    spindle_restore(gc_state->modal.spindle, gc_state->spindle.rpm);

    return Status_OK;
}

void atc_init (void)
{
    if(driver_reset == NULL) {
        driver_reset = hal.driver_reset;
        hal.driver_reset = atc_reset;
    }
    hal.tool.select = atc_tool_select;
    hal.tool.change = atc_tool_change;
}
