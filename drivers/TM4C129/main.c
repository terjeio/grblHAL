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

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"

#include "driver.h"
#include "GRBL/grbllib.h"

#ifdef FreeRTOS

#include "drivers/pinout.h"

#include "TCPStream.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "utils/lwiplib.h"
#include "utils/locator.h"

//*****************************************************************************
//
// A flag indicating the current link status.
//
//*****************************************************************************
static volatile bool linkUp = false;

//*****************************************************************************
//
// Required by lwIP library to support any host-related timer functions.  This
// function is called periodically, from the lwIP (TCP/IP) timer task context,
// every "HOST_TMR_INTERVAL" ms (defined in lwipopts.h file).
//
//*****************************************************************************
void lwIPHostTimerHandler (void)
{
    bool isLinkUp = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0) != 0;

    if(isLinkUp != linkUp)
    {
        linkUp = isLinkUp;
        TCPStreamNotifyLinkStatus(linkUp);
    }

    TCPStreamHandler();
}

//*****************************************************************************
//
// Setup lwIP raw API services that are provided by the application.  The
// services provided in this application are - locator service and TCPStream server.
//
//*****************************************************************************
void SetupServices (void *pvArg)
{
    uint8_t MACAddress[6];

    //
    // Setup the device locator service.
    //
    LocatorInit();
    lwIPLocalMACGet(MACAddress);
    LocatorMACAddrSet(MACAddress);
    LocatorAppTitleSet("Grbl CNC Controller");

    TCPStreamInit();
    TCPStreamListen(23);
}

//*****************************************************************************
//
// Initializes the lwIP tasks.
//
//*****************************************************************************
bool lwIPTaskInit (void)
{
    uint32_t User0, User1;
    uint8_t MACAddress[6];

    //
    // Get the MAC address from the user registers.
    //
    MAP_FlashUserGet(&User0, &User1);

    if((User0 == 0xffffffff) || (User1 == 0xffffffff))
        return false;

    //
    // Convert the 24/24 split MAC address from NV ram into a 32/16 split MAC
    // address needed to program the hardware registers, then program the MAC
    // address into the Ethernet Controller registers.
    //
    MACAddress[0] = ((User0 >>  0) & 0xff);
    MACAddress[1] = ((User0 >>  8) & 0xff);
    MACAddress[2] = ((User0 >> 16) & 0xff);
    MACAddress[3] = ((User1 >>  0) & 0xff);
    MACAddress[4] = ((User1 >>  8) & 0xff);
    MACAddress[5] = ((User1 >> 16) & 0xff);

    //
    // Lower the priority of the Ethernet interrupt handler to less than
    // configMAX_SYSCALL_INTERRUPT_PRIORITY.  This is required so that the
    // interrupt handler can safely call the interrupt-safe FreeRTOS functions
    // if any.
    //
    MAP_IntPrioritySet(INT_EMAC0, 0xC0);

    //
    // Set the link status based on the LED0 signal (which defaults to link
    // status in the PHY).
    //
    linkUp = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3) != 0;

    //
    // Initialize lwIP.
    //
    lwIPInit(configCPU_CLOCK_HZ, MACAddress, 0, 0, 0, IPADDR_USE_DHCP);

    //
    // Setup the remaining services inside the TCP/IP thread's context.
    //
    tcpip_callback(SetupServices, 0);

    return true;
}

//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(true);
}

static void vGrblTask( void * pvParameters )
{
    grbl_enter();
}

#endif // FreeRTOS

int main(void)
{

#ifdef FreeRTOS

    SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ|SYSCTL_OSC_MAIN|SYSCTL_USE_PLL|SYSCTL_CFG_VCO_480, configCPU_CLOCK_HZ);
    //
    // Configure the device pins based on the selected board.
    //

    PinoutSet(true, false);

    //
    // Initialize the Ethernet peripheral and create the lwIP tasks.
    //
    if(!lwIPTaskInit())
        while(true);

    xTaskCreate (vGrblTask, "Grbl", configMINIMAL_STACK_SIZE, NULL, 0, NULL);

    vTaskStartScheduler();

    while(true);

#else
    grbl_enter();
#endif

}
