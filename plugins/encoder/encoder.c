/*
  encoder.c - quadrature encoder plugin

  Part of GrblHAL

  Copyright (c) 2020 Terje Io

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

#ifdef ARDUINO
#include "../../driver.h"
#else
#include "driver.h"
#endif

#if QEI_ENABLE

#ifdef ARDUINO
#include "../grbl/grbl.h"
#include "../grbl/plugins.h"
#else
#include "grbl/grbl.h"
#include "grbl/plugins.h"
#endif

static uint32_t position = 0;
static bool mode_chg = false;
static encoder_mode_t mode = Encoder_FeedRate;

void encoder_execute_realtime (uint_fast16_t state)
{
	if(mode_chg) {

		switch(mode) {

			case Encoder_FeedRate:
				hal.stream.write("[MSG:Encoder mode feed rate]" ASCII_EOL);
				break;

			case Encoder_Spindle_RPM:
				hal.stream.write("[MSG:Encoder mode spindle RPM]" ASCII_EOL);
				break;

			default:
				break;
		}

		mode_chg = false;
	}
}

void encoder_changed (encoder_t *encoder)
{
	if(encoder->changed.select && encoder->mode == Encoder_Universal) {
        mode_chg = true;
        mode = mode == Encoder_FeedRate ? Encoder_Spindle_RPM : Encoder_FeedRate;
    }

	if(encoder->changed.position)
	  switch(encoder->mode == Encoder_Universal ? mode : encoder->mode) {

		case Encoder_FeedRate:
			hal.stream.enqueue_realtime_command(encoder->position > position ? CMD_OVERRIDE_FEED_FINE_PLUS : CMD_OVERRIDE_FEED_FINE_MINUS);
			break;

		case Encoder_Spindle_RPM:
			hal.stream.enqueue_realtime_command(encoder->position > position ? CMD_OVERRIDE_SPINDLE_FINE_PLUS : CMD_OVERRIDE_SPINDLE_FINE_MINUS);
			break;

		default:
			break;
	}

    position = encoder->position;

    encoder->changed.signals = 0;
}

#endif
