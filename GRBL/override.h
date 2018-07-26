/*
  override.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Buffer handlers for real-time override commands

  Part of Grbl

  Copyright (c) 2016-2017 Terje Io

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

#ifndef __OVERRIDE_H__
#define __OVERRIDE_H__

#define FEED_OVR_BUFSIZE 16         // must be a power of 2
#define ACCESSORY_OVR_BUFSIZE 16    // must be a power of 2

void flush_override_buffers ();
void enqueue_feed_ovr (uint8_t cmd);
uint8_t get_feed_ovr (void);
void enqueue_accessory_ovr (uint8_t cmd);
uint8_t get_accessory_ovr (void);

#endif
