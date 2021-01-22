/*

  main.c - startup for Atmel SAMD21 ARM processor (Arduino MKRZERO version)

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

// Double tap reset to enter bootloader mode - select bootloader port for programming

#include "grbl/grbllib.h"

void setup ()
{
    grbl_enter();
}

void loop ()
{
}
