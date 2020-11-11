/*
  main.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Entry point for driver for Cypress PSoC 5 (CY8CKIT-059)

  Note: this must be compiled together with my HALified version of Grbl,
        either as a dependent project or directly embedded in this project

  Part of GrblHAL

  Copyright (c) 2017-2020 Terje Io

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

#include "project.h"
#include "grbl/grbllib.h"

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    grbl_enter();
    
    return 0; // will never happen
    
}

/* [] END OF FILE */
