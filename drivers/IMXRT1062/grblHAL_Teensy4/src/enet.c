/*
  enet.c - lwIP driver glue code for IMXRT1062 processor (on Teensy 4.1 board)

  Part of grblHAL

  Copyright (c) 2020-2021 Terje Io

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

#include "driver.h"

#if ETHERNET_ENABLE

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <lwipopts.h>
#include <lwip_k6x.h>
#include <lwip_t41.h>
#include <lwip/netif.h>
#include "lwip/dhcp.h"

#include "grbl/report.h"
#include "grbl/nvs_buffer.h"

#include "networking/TCPStream.h"
#include "networking/WsStream.h"

static volatile bool linkUp = false;
static char IPAddress[IP4ADDR_STRLEN_MAX];
static network_services_t services = {0};
static nvs_address_t nvs_address;
static network_settings_t ethernet, network;
static on_report_options_ptr on_report_options;

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(newopt)
        hal.stream.write(",ETH");
    else {
        hal.stream.write("[IP:");
        hal.stream.write(IPAddress);
        hal.stream.write("]" ASCII_EOL);
    }
}

static void link_status_callback (struct netif *netif)
{
    bool isLinkUp = netif_is_link_up(netif);

    if(isLinkUp != linkUp) {
        linkUp = isLinkUp;
        TCPStreamNotifyLinkStatus(linkUp);
    }
}

static void netif_status_callback (struct netif *netif)
{
    ip4addr_ntoa_r(netif_ip_addr4(netif), IPAddress, IP4ADDR_STRLEN_MAX);

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

void grbl_enet_poll (void)
{
    static uint32_t last_ms0, last_ms1;
    uint32_t ms;

    enet_proc_input();

    ms = millis();

    if(ms - last_ms0 > 1) {
        last_ms0 = ms;
#if TELNET_ENABLE
        if(services.telnet)
          TCPStreamPoll();
#endif
    #if WEBSOCKET_ENABLE
        if(services.websocket)
          WsStreamPoll();
#endif
    }

    if (ms - last_ms1 > 25)
    {
        last_ms1 = ms;
        enet_poll();
    }
}

bool grbl_enet_start (void)
{
    if(nvs_address != 0) {

        *IPAddress = '\0';

        memcpy(&network, &ethernet, sizeof(network_settings_t));

        if(network.ip_mode == IpMode_Static)
            enet_init((ip_addr_t *)&network.ip, (ip_addr_t *)&network.mask, (ip_addr_t *)&network.gateway);
        else
            enet_init(NULL, NULL, NULL);

        netif_set_status_callback(netif_default, netif_status_callback);
        netif_set_link_callback(netif_default, link_status_callback);

        netif_set_up(netif_default);
    #if LWIP_NETIF_HOSTNAME
        netif_set_hostname(netif_default, network.hostname);
    #endif
        if(network.ip_mode == IpMode_DHCP)
            dhcp_start(netif_default);
    }

    return nvs_address != 0;
}

static inline void set_addr (char *ip, ip4_addr_t *addr)
{
    memcpy(ip, addr, sizeof(ip4_addr_t));
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

PROGMEM static const setting_detail_t ethernet_settings[] = {
    { Setting_NetworkServices, Group_Networking, "Network Services", NULL, Format_Bitfield, netservices, NULL, NULL, Setting_NonCore, &ethernet.services.mask, NULL, NULL },
    { Setting_Hostname, Group_Networking, "Hostname", NULL, Format_String, "x(64)", NULL, "64", Setting_NonCore, ethernet.hostname, NULL, NULL },
    { Setting_IpMode, Group_Networking, "IP Mode", NULL, Format_RadioButtons, "Static,DHCP,AutoIP", NULL, NULL, Setting_NonCore, &ethernet.ip_mode, NULL, NULL },
    { Setting_IpAddress, Group_Networking, "IP Address", NULL, Format_IPv4, NULL, NULL, NULL, Setting_NonCoreFn, ethernet_set_ip, ethernet_get_ip, NULL },
    { Setting_Gateway, Group_Networking, "Gateway", NULL, Format_IPv4, NULL, NULL, NULL, Setting_NonCoreFn, ethernet_set_ip, ethernet_get_ip, NULL },
    { Setting_NetMask, Group_Networking, "Netmask", NULL, Format_IPv4, NULL, NULL, NULL, Setting_NonCoreFn, ethernet_set_ip, ethernet_get_ip, NULL },
    { Setting_TelnetPort, Group_Networking, "Telnet port", NULL, Format_Int16, "####0", "1", "65535", Setting_NonCore, &ethernet.telnet_port, NULL, NULL },
#if HTTP_ENABLE
    { Setting_HttpPort, Group_Networking, "HTTP port", NULL, Format_Int16, "####0", "1", "65535", Setting_NonCore, &ethernet.http_port, NULL, NULL },
#endif
    { Setting_WebSocketPort, Group_Networking, "Websocket port", NULL, Format_Int16, "####0", "1", "65535", Setting_NonCore, &ethernet.websocket_port, NULL, NULL }
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
            set_addr(ethernet.ip, &addr);
            break;

        case Setting_Gateway:
            set_addr(ethernet.gateway, &addr);
            break;

        case Setting_NetMask:
            set_addr(ethernet.mask, &addr);
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

void ethernet_settings_restore (void)
{
    strcpy(ethernet.hostname, NETWORK_HOSTNAME);

    ip4_addr_t addr;

    ethernet.ip_mode = (ip_mode_t)NETWORK_IPMODE;

    if(ip4addr_aton(NETWORK_IP, &addr) == 1)
        set_addr(ethernet.ip, &addr);

    if(ip4addr_aton(NETWORK_GATEWAY, &addr) == 1)
        set_addr(ethernet.gateway, &addr);

#if NETWORK_IPMODE == 0
    if(ip4addr_aton(NETWORK_MASK, &addr) == 1)
        set_addr(ethernet.mask, &addr);
#else
    if(ip4addr_aton("255.255.255.0", &addr) == 1)
        set_addr(ethernet.mask, &addr);
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

bool grbl_enet_init (network_settings_t *settings)
{
    if((nvs_address = nvs_alloc(sizeof(network_settings_t)))) {

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = report_options;

        details.on_get_settings = grbl.on_get_settings;
        grbl.on_get_settings = on_get_settings;
    }

    return nvs_address != 0;
}

#endif
