/*
  driver.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for Texas Instruments Tiva C (TM4C123GH6PM) ARM processor

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

// User defined mcode handling prototypes
#ifndef _USERMCODES_H_
#define _USERMCODES_H_

uint8_t userMCodeCheck (uint8_t mcode);
uint8_t userMCodeValidate (parser_block_t *gc_block, uint_fast16_t *value_words);
void userMCodeExecute (uint8_t state, parser_block_t *gc_block);

#endif
