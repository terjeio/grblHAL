/*
  override.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Buffer handlers for real-time override commands

  Part of Grbl

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

#include "grbl.h"

static uint8_t feed_buf[FEED_OVR_BUFSIZE], accessory_buf[FEED_OVR_BUFSIZE];
static volatile uint_fast8_t feed_head = 0, feed_tail = 0, accessory_head = 0, accessory_tail = 0;

ISR_CODE void enqueue_feed_ovr (uint8_t cmd)
{
    uint_fast8_t bptr = (feed_head + 1) & (FEED_OVR_BUFSIZE - 1);    // Get next head pointer

    if(bptr != feed_tail) {                       // If not buffer full
        feed_buf[feed_head] = cmd;                // add data to buffer
        feed_head = bptr;                         // and update pointer
    }
}

// Returns 0 if no commands enqueued
uint8_t get_feed_ovr (void)
{
    uint8_t data = 0;
    uint_fast8_t bptr = feed_tail;

    if(bptr != feed_head) {
        data = feed_buf[bptr++];                    // Get next character, increment tmp pointer
        feed_tail = bptr & (FEED_OVR_BUFSIZE - 1);  // and update pointer
    }

    return data;
}

ISR_CODE void enqueue_accessory_ovr (uint8_t cmd)
{
    uint_fast8_t bptr = (accessory_head + 1) & (FEED_OVR_BUFSIZE - 1);    // Get next head pointer

    if(bptr != accessory_tail) {                       // If not buffer full
        accessory_buf[accessory_head] = cmd;           // add data to buffer
        accessory_head = bptr;                         // and update pointer
    }
}

// Returns 0 if no commands enqueued
uint8_t get_accessory_ovr (void)
{
    uint8_t data = 0;
    uint_fast8_t bptr = accessory_tail;

    if(bptr != accessory_head) {
        data = accessory_buf[bptr++];                    // Get next character, increment tmp pointer
        accessory_tail = bptr & (FEED_OVR_BUFSIZE - 1);  // and update pointer
    }

    return data;
}

void flush_override_buffers () {
    feed_head = feed_tail = accessory_head = accessory_tail = 0;
}
