//
// enet.c - lw-IP/FreeRTOS TCP/IP stream implementation
//
// v1.0 / 2018-12-08 / Io Engineering / Terje
//

/*

Copyright (c) 2018, Terje Io
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

· Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

· Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

· Neither the name of the copyright holder nor the names of its contributors may
be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"

#include "utils/lwiplib.h"
//#include "utils/locator.h" // comment in to enable TIs locator service

#include "base/driver.h"
#include "TCPStream.h"

#define SYSTICK_INT_PRIORITY    0x80
#define ETHERNET_INT_PRIORITY   0xC0

static volatile bool linkUp = false;
static uint32_t IPAddress = 0;
static uint16_t service_port;

char *enet_ip_address (void)
{
    static char ip[16];

    if(IPAddress == 0xFFFFFFFF)
        strcpy(ip, "no link");
    else if(IPAddress)
        sprintf(ip, "%d.%d.%d.%d", IPAddress &0xFF, (IPAddress >> 8) &0xFF, (IPAddress >> 16) &0xFF, (IPAddress >> 24));
    else
        ip[0] = '\0';

    return ip;
}

void lwIPHostTimerHandler (void)
{
    bool isLinkUp = PREF(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0)) != 0;

    IPAddress = lwIPLocalIPAddrGet();

    if(isLinkUp != linkUp) {
        linkUp = isLinkUp;
        TCPStreamNotifyLinkStatus(linkUp);
    }

    TCPStreamHandler();
}

void setupServices (void *pvArg)
{
#ifdef __LOCATOR_H__
    // Setup the device locator service.

    uint8_t MACAddress[6];

    LocatorInit();
    lwIPLocalMACGet(MACAddress);
    LocatorMACAddrSet(MACAddress);
    LocatorAppTitleSet("Grbl CNC Controller");
#endif

    TCPStreamInit();
    TCPStreamListen(service_port);
}

bool lwIPTaskInit (void)
{
    uint32_t User0, User1;
    uint8_t MACAddress[6];

    // Get the MAC address from the user registers.
    PREF(FlashUserGet(&User0, &User1));

    if((User0 == 0xFFFFFFFF) || (User1 == 0xFFFFFFFF))
        return false;

    // Convert the 24/24 split MAC address from flash into a 32/16 split MAC address needed by lwIP.
    MACAddress[0] = ((User0 >>  0) & 0xFF);
    MACAddress[1] = ((User0 >>  8) & 0xFF);
    MACAddress[2] = ((User0 >> 16) & 0xFF);
    MACAddress[3] = ((User1 >>  0) & 0xFF);
    MACAddress[4] = ((User1 >>  8) & 0xFF);
    MACAddress[5] = ((User1 >> 16) & 0xFF);

    //
    // Lower the priority of the Ethernet interrupt handler to less than
    // configMAX_SYSCALL_INTERRUPT_PRIORITY.  This is required so that the
    // interrupt handler can safely call the interrupt-safe FreeRTOS functions
    // if any.
    //

    PREF(SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF));
    PREF(IntPrioritySet(INT_EMAC0, 0xC0));

    PREF(GPIOPinConfigure(GPIO_PF0_EN0LED0));
    PREF(GPIOPinConfigure(GPIO_PF4_EN0LED1));

    PREF(GPIOPinTypeEthernetLED(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_4));

    //
    // Set the link status based on the LED0 signal (which defaults to link
    // status in the PHY).
    //
    linkUp = PREF(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3)) != 0;

//    SysCtlMOSCConfigSet(SYSCTL_MOSC_HIGHFREQ);
    // Register ethernet IRQ handler here, this since the driver layer does not do that

#ifdef __MSP432E401Y__
    PREF(EMACIntRegister(EMAC0_BASE, EMAC0_IRQHandler));
    PREF(IntPrioritySet(INT_EMAC0, ETHERNET_INT_PRIORITY));
    PREF(IntPrioritySet(FAULT_SYSTICK, SYSTICK_INT_PRIORITY));
#endif

    lwIPInit(configCPU_CLOCK_HZ, MACAddress, 0, 0, 0, IPADDR_USE_DHCP);

    // Setup the remaining services inside the TCP/IP thread's context.
    tcpip_callback(setupServices, 0);

    return true;
}

//*****************************************************************************
//
// This hook is called by FreeRTOS when memory allocation failure is detected.
//
//*****************************************************************************

void vApplicationMallocFailedHook()
{
    while(true);
}

//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************

void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
    while(true);
}

bool enet_init (uint16_t port)
{
    service_port = port;

    return lwIPTaskInit();
}
