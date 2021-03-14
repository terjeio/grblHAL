/*
  mainc.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Startup entry point for TM4C1294NCPDT (Texas Instruments Tiva C LaunchPad)

  Part of grblHAL

  Copyright (c) 2018-2021 Terje Io

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
#include <stdlib.h>

#include <ti/boards/MSP_EXP432E401Y/Board.h>

#include "FreeRTOS.h"
#include "task.h"

#include "shared/base/driver.h"
#include "GRBL/grbllib.h"

#ifndef FreeRTOS
#error This configuration requires at least one FreeRTOS dependent option enabled in base/driver.h!
#endif

static void vGrblTask (void * pvParameters)
{
    grbl_enter();
}

int main(void)
{
    /* Call driver init functions */
    MSP_EXP432E401Y_initGeneral();

    xTaskCreate(vGrblTask, "Grbl", configMINIMAL_STACK_SIZE, NULL, 0, NULL);

    vTaskStartScheduler();

    while(true);
}

