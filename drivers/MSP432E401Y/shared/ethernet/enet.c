//
// enet.c - lwIP/FreeRTOS TCP/IP stream implementation
//
// v1.3 / 2021-01-22 / Io Engineering / Terje
//

/*

Copyright (c) 2018-2021, Terje Io
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

#include "shared/base/driver.h"

#if ETHERNET_ENABLE

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"

#include "utils/lwiplib.h"
//#include "utils/locator.h" // comment in to enable TIs locator service

#include "posix/sys/socket.h"

#include "grbl/report.h"
#include "grbl/nvs_buffer.h"

#include "networking/networking.h"
#include "networking/TCPStream.h"
#include "networking/WsStream.h"

#define SYSTICK_INT_PRIORITY    0x80
#define ETHERNET_INT_PRIORITY   0xC0

// some defines for old lwIP stack

#ifndef IPADDR_STRLEN_MAX
#define IPADDR_STRLEN_MAX 16
#endif

#ifndef ip4addr_ntoa_r
#define ip4addr_ntoa_r(ipaddr, buf, buflen) ipaddr_ntoa_r(ipaddr, buf, buflen)
#endif

#ifndef ip4addr_aton
#define ip4addr_aton(cp, addr) ipaddr_aton(cp, addr)
#endif

//

static volatile bool linkUp = false;
static uint32_t IPAddress = 0;
static network_settings_t network, ethernet;
static network_services_t services = {0};
static uint32_t nvs_address;
static on_report_options_ptr on_report_options;

static char *enet_ip_address (void)
{
    static char ip[IPADDR_STRLEN_MAX];

    ip4addr_ntoa_r((const ip_addr_t *)&IPAddress, ip, IPADDR_STRLEN_MAX);

    return ip;
}

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(newopt)
        hal.stream.write("[IP:");
    else {
        hal.stream.write("[IP:");
        hal.stream.write(enet_ip_address());
        hal.stream.write("]\r\n");
    }
}

void lwIPHostTimerHandler (void)
{
    bool isLinkUp = PREF(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0)) != 0;

    IPAddress = network.ip_mode == IpMode_Static && isLinkUp ? *(uint32_t *)network.ip : lwIPLocalIPAddrGet();

    if(IPAddress == 0xffffffff)
        IPAddress = 0;

    if(isLinkUp != linkUp) {
        linkUp = isLinkUp;
        TCPStreamNotifyLinkStatus(linkUp);
    }

#if TELNET_ENABLE
    if(services.telnet)
        TCPStreamPoll();
#endif
#if WEBSOCKET_ENABLE
    if(services.websocket)
        WsStreamPoll();
#endif
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

#if TELNET_ENABLE
    if(network.services.telnet && !services.telnet) {
        TCPStreamInit();
        TCPStreamListen(network.telnet_port == 0 ? 23 : network.telnet_port);
        services.telnet = On;
    }
#endif
#if WEBSOCKET_ENABLE
    if(network.services.websocket && !services.websocket) {
        WsStreamInit();
        WsStreamListen(network.websocket_port == 0 ? 80 : network.websocket_port);
        services.websocket = On;
    }
#endif
}

bool enet_start (void)
{
    uint32_t User0, User1;
    uint8_t MACAddress[6];

    // Get the MAC address from the user registers.
    PREF(FlashUserGet(&User0, &User1));

    if(nvs_address == 0 || (User0 == 0xFFFFFFFF) || (User1 == 0xFFFFFFFF))
        return false;

    // Convert the 24/24 split MAC address from flash into a 32/16 split MAC address needed by lwIP.
    MACAddress[0] = ((User0 >>  0) & 0xFF);
    MACAddress[1] = ((User0 >>  8) & 0xFF);
    MACAddress[2] = ((User0 >> 16) & 0xFF);
    MACAddress[3] = ((User1 >>  0) & 0xFF);
    MACAddress[4] = ((User1 >>  8) & 0xFF);
    MACAddress[5] = ((User1 >> 16) & 0xFF);

    memcpy(&network, &ethernet, sizeof(network_settings_t));

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

    if(network.ip_mode == IpMode_DHCP)
        lwIPInit(configCPU_CLOCK_HZ, MACAddress, 0, 0, 0, IPADDR_USE_DHCP);
    else
        lwIPInit(configCPU_CLOCK_HZ, MACAddress, ntohl(*(uint32_t *)network.ip), ntohl(*(uint32_t *)network.mask), ntohl(*(uint32_t *)network.gateway), network.ip_mode == IpMode_Static ? IPADDR_USE_STATIC : IPADDR_USE_AUTOIP);

#if LWIP_NETIF_HOSTNAME
    extern struct netif *netif_default;
    netif_set_hostname(netif_default, network.hostname);
#endif

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

static void ethernet_settings_load (void);
static void ethernet_settings_restore (void);
static status_code_t ethernet_set_ip (setting_id_t setting, char *value);
static char *ethernet_get_ip (setting_id_t setting);

static const setting_group_detail_t ethernet_groups [] = {
    { Group_Root, Group_Networking, "Networking" }
};

#if TELNET_ENABLE && WEBSOCKET_ENABLE && HTTP_ENABLE
static const char netservices[] = "Telnet,Websocket,HTTP";
#endif
#if TELNET_ENABLE && WEBSOCKET_ENABLE && !HTTP_ENABLE
static const char netservices[] = "Telnet,Websocket";
#endif
#if TELNET_ENABLE && !WEBSOCKET_ENABLE && !HTTP_ENABLE
static const char netservices[] = "Telnet";
#endif

static const setting_detail_t ethernet_settings[] = {
#if TELNET_ENABLE
    { Setting_NetworkServices, Group_Networking, "Network Services", NULL, Format_Bitfield, netservices, NULL, NULL, Setting_NonCore, &ethernet.services.mask, NULL, NULL },
#endif
    { Setting_Hostname, Group_Networking, "Hostname", NULL, Format_String, "x(64)", NULL, "64", Setting_NonCore, ethernet.hostname, NULL, NULL },
    { Setting_IpMode, Group_Networking, "IP Mode", NULL, Format_RadioButtons, "Static,DHCP,AutoIP", NULL, NULL, Setting_NonCore, &ethernet.ip_mode, NULL, NULL },
    { Setting_IpAddress, Group_Networking, "IP Address", NULL, Format_IPv4, NULL, NULL, NULL, Setting_NonCoreFn, ethernet_set_ip, ethernet_get_ip, NULL },
    { Setting_Gateway, Group_Networking, "Gateway", NULL, Format_IPv4, NULL, NULL, NULL, Setting_NonCoreFn, ethernet_set_ip, ethernet_get_ip, NULL },
    { Setting_NetMask, Group_Networking, "Netmask", NULL, Format_IPv4, NULL, NULL, NULL, Setting_NonCoreFn, ethernet_set_ip, ethernet_get_ip, NULL },
    { Setting_TelnetPort, Group_Networking, "Telnet port", NULL, Format_Int16, "####0", "1", "65535", Setting_NonCore, &ethernet.telnet_port, NULL, NULL },
    { Setting_WebSocketPort, Group_Networking, "Websocket port", NULL, Format_Int16, "####0", "1", "65535", Setting_NonCore, &ethernet.websocket_port, NULL, NULL }
#if HTTP_ENABLE
    { Setting_HttpPort, Group_Networking, "HTTP port", NULL, Format_Int16, "####0", "1", "65535", Setting_NonCore, &ethernet.http_port, NULL, NULL },
#endif
};

static void ethernet_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&ethernet, sizeof(network_settings_t), true);
}

static setting_details_t details = {
    .groups = ethernet_groups,
    .n_groups = sizeof(ethernet_groups) / sizeof(setting_group_detail_t),
    .settings = ethernet_settings,
    .n_settings = sizeof(ethernet_settings) / sizeof(setting_detail_t),
    .save = ethernet_settings_save,
    .load = ethernet_settings_load,
    .restore = ethernet_settings_restore
};

static setting_details_t *on_get_settings (void)
{
    return &details;
}

static status_code_t ethernet_set_ip (setting_id_t setting, char *value)
{
    ip_addr_t addr;

    if(ip4addr_aton(value, &addr) != 1)
        return Status_InvalidStatement;

    status_code_t status = Status_OK;

    switch(setting) {

        case Setting_IpAddress:
            *((ip_addr_t *)ethernet.ip) = addr;
            break;

        case Setting_Gateway:
            *((ip_addr_t *)ethernet.gateway) = addr;
            break;

        case Setting_NetMask:
            *((ip_addr_t *)ethernet.mask) = addr;
            break;

        default:
            status = Status_Unhandled;
            break;
    }

    return status;
}

static char *ethernet_get_ip (setting_id_t setting)
{
    static char ip[IPADDR_STRLEN_MAX];

    switch(setting) {

        case Setting_IpAddress:
            ip4addr_ntoa_r((const ip_addr_t *)&ethernet.ip, ip, IPADDR_STRLEN_MAX);
            break;

        case Setting_Gateway:
            ip4addr_ntoa_r((const ip_addr_t *)&ethernet.gateway, ip, IPADDR_STRLEN_MAX);
            break;

        case Setting_NetMask:
            ip4addr_ntoa_r((const ip_addr_t *)&ethernet.mask, ip, IPADDR_STRLEN_MAX);
            break;

        default:
            *ip = '\0';
            break;
    }

    return ip;
}

static void ethernet_settings_restore (void)
{
    strcpy(ethernet.hostname, NETWORK_HOSTNAME);

    ip_addr_t addr;

    ethernet.ip_mode = (ip_mode_t)NETWORK_IPMODE;

    if(ip4addr_aton(NETWORK_IP, &addr) == 1)
        *((ip_addr_t *)ethernet.ip) = addr;

    if(ip4addr_aton(NETWORK_GATEWAY, &addr) == 1)
        *((ip_addr_t *)ethernet.gateway) = addr;

#if NETWORK_IPMODE == 0
    if(ip4addr_aton(NETWORK_MASK, &addr) == 1)
        *((ip_addr_t *)ethernet.mask) = addr;
#else
    if(ip4addr_aton("255.255.255.0", &addr) == 1)
        *((ip_addr_t *)ethernet.mask) = addr;
#endif

    ethernet.services.mask = 0;
    ethernet.telnet_port = NETWORK_TELNET_PORT;
    ethernet.http_port = NETWORK_HTTP_PORT;
    ethernet.websocket_port = NETWORK_WEBSOCKET_PORT;

#if TELNET_ENABLE
    ethernet.services.telnet = On;
#endif

#if HTTP_ENABLE
    ethernet.services.http = On;
#endif

#if WEBSOCKET_ENABLE
    ethernet.services.websocket = On;
#endif

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&ethernet, sizeof(network_settings_t), true);
}

static void ethernet_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&ethernet, nvs_address, sizeof(network_settings_t), true) != NVS_TransferResult_OK)
        ethernet_settings_restore();
}

bool enet_init (void)
{
    if((nvs_address = nvs_alloc(sizeof(network_settings_t)))) {

        hal.driver_cap.ethernet = On;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = report_options;

        details.on_get_settings = grbl.on_get_settings;
        grbl.on_get_settings = on_get_settings;
    }

    return nvs_address != 0;
}

#endif
