/*
  grbl_interface.h - functions to link AVR sim to GRBL app
    stepper interrupt is called

  Part of Grbl Simulator

  Copyright (c) 2015 Adam Shelly

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

void grbl_app_init(void);  //call to setup ISRs and local tracking vars
void grbl_per_tick(void);  //call per tick to print steps
void grbl_per_byte(void);  //call per incoming byte to print block info
void grbl_app_exit(void);  //call to shutdown cleanly
