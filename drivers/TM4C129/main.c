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

#include "base/driver.h"
#include "GRBL/grbllib.h"

#include "FreeRTOS.h"
#include "task.h"

uint32_t g_ui32SysClock;

static void vGrblTask (void * pvParameters)
{
    grbl_enter();
}

int main(void)
{
    g_ui32SysClock = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ|SYSCTL_OSC_MAIN|SYSCTL_USE_PLL|SYSCTL_CFG_VCO_480, configCPU_CLOCK_HZ);

    xTaskCreate (vGrblTask, "Grbl", 2048, NULL, 0, NULL);

    vTaskStartScheduler();

    while(true);
}
