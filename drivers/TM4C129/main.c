/*
  mainc.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Startup entry point for TM4C1294NCPDT (Texas Instruments Tiva C LaunchPad)

  Part of Grbl

  Copyright (c) 2018 Terje Io

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

#include <stdint.h>
#include <stdbool.h>

#include "driverlib/sysctl.h"

#include "core/driver.h"
#include "GRBL/grbllib.h"

int main(void)
{
    grbl_enter();
}
