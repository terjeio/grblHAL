/*
  nvs.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Peristent storage of settings in flash

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

bool nvsInit(void);
uint8_t nvsGetByte (uint32_t addr);
void nvsPutByte (uint32_t addr, uint8_t new_value);
void nvsWriteBlockWithChecksum (uint32_t destination, uint8_t *source, uint32_t size);
bool nvsReadBlockWithChecksum (uint8_t *destination, uint32_t source, uint32_t size);
