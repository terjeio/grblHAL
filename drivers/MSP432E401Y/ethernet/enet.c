//
// enet.c - lwIP/FreeRTOS TCP/IP stream implementation
//
// v1.1 / 2020-07-14 / Io Engineering / Terje
//

/*

Copyright (c) 2018-2020, Terje Io
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

#include "posix/sys/socket.h"

#include "base/driver.h"
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
static network_settings_t *network;
static network_services_t services = {0};

char *enet_ip_address (void)
{
    static char ip[IPADDR_STRLEN_MAX];

    ip4addr_ntoa_r((const ip_addr_t *)&IPAddress, ip, IPADDR_STRLEN_MAX);

    return ip;
}

void lwIPHostTimerHandler (void)
{
    bool isLinkUp = PREF(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0)) != 0;

    IPAddress = network->ip_mode == IpMode_Static && isLinkUp ? *(uint32_t *)network->ip : lwIPLocalIPAddrGet();

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
    if(network->services.telnet && !services.telnet) {
        TCPStreamInit();
        TCPStreamListen(network->telnet_port == 0 ? 23 : network->telnet_port);
        services.telnet = On;
    }
#endif
#if WEBSOCKET_ENABLE
    if(network->services.websocket && !services.websocket) {
        WsStreamInit();
        WsStreamListen(network->websocket_port == 0 ? 80 : network->websocket_port);
        services.websocket = On;
    }
#endif
}

bool lwIPTaskInit (network_settings_t *settings)
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

    network = settings;

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

    if(network->ip_mode == IpMode_DHCP)
        lwIPInit(configCPU_CLOCK_HZ, MACAddress, 0, 0, 0, IPADDR_USE_DHCP);
    else
        lwIPInit(configCPU_CLOCK_HZ, MACAddress, ntohl(*(uint32_t *)network->ip), ntohl(*(uint32_t *)network->mask), ntohl(*(uint32_t *)network->gateway), network->ip_mode == IpMode_Static ? IPADDR_USE_STATIC : IPADDR_USE_AUTOIP);

#if LWIP_NETIF_HOSTNAME
    extern struct netif *netif_default;
    netif_set_hostname(netif_default, network->hostname);
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

bool enet_init (network_settings_t *network)
{
    return lwIPTaskInit(network);
}

#if ETHERNET_ENABLE

status_code_t ethernet_setting (setting_type_t param, float value, char *svalue)
{
    status_code_t status = Status_Unhandled;

    if(svalue) switch(param) {

        case Setting_NetworkServices:
            if(isintf(value) && value >= 0.0f && value < 256.0f) {
                status = Status_OK;
                network_services_t is_available = {0};
#if TELNET_ENABLE
                is_available.telnet = On;
#endif
#if HTTP_ENABLE
                is_available.http = On;
#endif
#if WEBSOCKET_ENABLE
                is_available.websocket = On;
#endif
                driver_settings.network.services.mask = (uint8_t)value & is_available.mask;
                // TODO: fault if attempt to select services not available?
            } else
                status = Status_InvalidStatement; //out of range...
            break;

#if LWIP_NETIF_HOSTNAME
        case Setting_Hostname:
            if(strlen(svalue) < sizeof(hostname_t)) {
                strcpy(driver_settings.network.hostname, svalue);
                status = Status_OK;
            } else
                status = Status_InvalidStatement; // too long...
            break;
#endif
/* Static mode is not working...
        case Setting_IpMode:
          if(isintf(value) >= 0.0f && value <= 2.0f) {
              status = Status_OK;
              driver_settings.network.ip_mode = (ip_mode_t)(uint8_t)value;
          }
          else
              status = Status_InvalidStatement; // out of range...
          break;
*/
        case Setting_IpAddress:
          {
              ip_addr_t addr;
              if(ip4addr_aton(svalue, &addr) == 1) {
                  status = Status_OK;
                  *((ip_addr_t *)driver_settings.network.ip) = addr;
              } else
                  status = Status_InvalidStatement;
          }
          break;

        case Setting_Gateway:
          {
              ip_addr_t addr;
              if(ip4addr_aton(svalue, &addr) == 1) {
                  status = Status_OK;
                  *((ip_addr_t *)driver_settings.network.gateway) = addr;
              } else
                  status = Status_InvalidStatement;
          }
          break;

        case Setting_NetMask:
          {
              ip_addr_t addr;
              if(ip4addr_aton(svalue, &addr) == 1) {
                  status = Status_OK;
                  *((ip_addr_t *)driver_settings.network.mask) = addr;
              } else
                  status = Status_InvalidStatement;
          }
          break;

#if TELNET_ENABLE
      case Setting_TelnetPort:
          if(isintf(value) && value != NAN && value > 0.0f && value < 65536.0f) {
              status = Status_OK;
              driver_settings.network.telnet_port = (uint16_t)value;
          } else
              status = Status_InvalidStatement; //out of range...
          break;
#endif

#if WEBSOCKET_ENABLE
      case Setting_WebSocketPort:
          if(isintf(value) && value != NAN && value > 0.0f && value < 65536.0f) {
              status = Status_OK;
              driver_settings.network.websocket_port = (uint16_t)value;
          } else
              status = Status_InvalidStatement; //out of range...
          break;
#endif

#if HTTP_ENABLE
      case Setting_HttpPort:
          if((claimed = (value != NAN && value > 0.0f && value < 65536.0f) {
              status = Status_OK;
              driver_settings.network.http_port = (uint16_t)value;
          } else
              status = Status_InvalidStatement; //out of range...
          break;
#endif
    }

    return status;
}

void ethernet_settings_report (setting_type_t setting)
{
    switch(setting) {

        case Setting_NetworkServices:
            report_uint_setting(setting, driver_settings.network.services.mask);
            break;

#if LWIP_NETIF_HOSTNAME
        case Setting_Hostname:
            report_string_setting(setting, driver_settings.network.hostname);
            break;
#endif

        case Setting_IpMode:
            report_uint_setting(setting, driver_settings.network.ip_mode);
            break;

        case Setting_IpAddress:
            if(driver_settings.network.ip_mode != IpMode_DHCP) {
                char ip[IPADDR_STRLEN_MAX];
                report_string_setting(setting, ip4addr_ntoa_r((const ip_addr_t *)&driver_settings.network.ip, ip, IPADDR_STRLEN_MAX));
            }
            break;

        case Setting_Gateway:
            if(driver_settings.network.ip_mode != IpMode_DHCP) {
                char ip[IPADDR_STRLEN_MAX];
                report_string_setting(setting, ip4addr_ntoa_r((const ip_addr_t *)&driver_settings.network.gateway, ip, IPADDR_STRLEN_MAX));
            }
            break;

        case Setting_NetMask:
            if(driver_settings.network.ip_mode != IpMode_DHCP) {
                char ip[IPADDR_STRLEN_MAX];
                report_string_setting(setting, ip4addr_ntoa_r((const ip_addr_t *)&driver_settings.network.mask, ip, IPADDR_STRLEN_MAX));
            }
            break;

#if TELNET_ENABLE
        case Setting_TelnetPort:
            report_uint_setting(setting, driver_settings.network.telnet_port);
            break;
#endif

#if WEBSOCKET_ENABLE
        case Setting_WebSocketPort:
            report_uint_setting(setting, driver_settings.network.websocket_port);
            break;
#endif
    }
}

void ethernet_settings_restore (void)
{
    strcpy(driver_settings.network.hostname, NETWORK_HOSTNAME);

#if NETWORK_IPMODE_STATIC

    ip_addr_t addr;

    if(ip4addr_aton(NETWORK_IP, &addr) == 1)
        *((ip_addr_t *)driver_settings.network.ip) = addr;

    if(ip4addr_aton(NETWORK_GATEWAY, &addr) == 1)
        *((ip_addr_t *)driver_settings.network.gateway) = addr;

    if(ip4addr_aton(NETWORK_MASK, &addr) == 1)
        *((ip_addr_t *)driver_settings.network.mask) = addr;

    driver_settings.network.ip_mode = IpMode_Static;
#else
    driver_settings.network.ip_mode = IpMode_DHCP;
#endif

    driver_settings.network.telnet_port = NETWORK_TELNET_PORT;
    driver_settings.network.http_port = NETWORK_HTTP_PORT;
    driver_settings.network.websocket_port = NETWORK_WEBSOCKET_PORT;

#if TELNET_ENABLE
    driver_settings.network.services.telnet = On;
#endif

#if HTTP_ENABLE
    driver_settings.network.services.http = On;
#endif

#if WEBSOCKET_ENABLE
    driver_settings.network.services.websocket = On;
#endif
}

#endif
