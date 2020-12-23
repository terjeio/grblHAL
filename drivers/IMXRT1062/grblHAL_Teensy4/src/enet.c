/*
  enet.c - lwIP driver glue code for IMXRT1062 processor (on Teensy 4.1 board)

  Part of grblHAL

  Copyright (c) 2020 Terje Io

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
static driver_setting_ptrs_t driver_settings;
static network_settings_t ethernet, network;
static on_report_options_ptr on_report_options;

static void reportIP (bool newopt)
{
    on_report_options(newopt);

    if(!newopt) {
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
    if(driver_settings.nvs_address != 0) {

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

    return driver_settings.nvs_address != 0;
}

static inline void set_addr (char *ip, ip4_addr_t *addr)
{
    memcpy(ip, addr, sizeof(ip4_addr_t));
}

static const setting_group_detail_t ethernet_groups [] = {
    { Group_Root, Group_Networking, "Networking" }
};

static const setting_detail_t ethernet_settings[] = {
    { Setting_NetworkServices, Group_Networking, "Network Services", NULL, Format_Bitfield, "Telnet,Websocket", NULL, NULL },
    { Setting_Hostname, Group_Networking, "Hostname", NULL, Format_String, "x(64)", NULL, "64" },
    { Setting_IpMode, Group_Networking, "IP Mode", NULL, Format_RadioButtons, "Static,DHCP,AutoIP", NULL, NULL },
    { Setting_IpAddress, Group_Networking, "IP Address", NULL, Format_IPv4, NULL, NULL, NULL },
    { Setting_Gateway, Group_Networking, "Gateway", NULL, Format_IPv4, NULL, NULL, NULL },
    { Setting_NetMask, Group_Networking, "Netmask", NULL, Format_IPv4, NULL, NULL, NULL },
    { Setting_TelnetPort, Group_Networking, "Telnet port", NULL, Format_Integer, "####0", "1", "65535" },
#if HTTP_ENABLE
    { Setting_HttpPort, Group_Networking, "HTTP port", NULL, Format_Integer, "####0", "1", "65535" },
#endif
    { Setting_WebSocketPort, Group_Networking, "Websocket port", NULL, Format_Integer, "####0", "1", "65535" }
};

static setting_details_t details = {
    .groups = ethernet_groups,
    .n_groups = sizeof(ethernet_groups) / sizeof(setting_group_detail_t),
    .settings = ethernet_settings,
    .n_settings = sizeof(ethernet_settings) / sizeof(setting_detail_t)
};

static setting_details_t *on_report_settings (void)
{
    return &details;
}

status_code_t ethernet_setting (setting_type_t setting, float value, char *svalue)
{
    status_code_t status = svalue ? Status_OK : Status_Unhandled;

    if(svalue) switch(setting) {

        case Setting_NetworkServices:
            if(isintf(value) && value >= 0.0f && value < 256.0f) {
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
                ethernet.services.mask = (uint8_t)value & is_available.mask;
                // TODO: fault if attempt to select services not available?
            } else
                status = Status_InvalidStatement; //out of range...
            break;

#if LWIP_NETIF_HOSTNAME
        case Setting_Hostname:
            if(strlen(svalue) < sizeof(hostname_t))
                strcpy(ethernet.hostname, svalue);
            else
                status = Status_InvalidStatement; // too long...
            break;
#endif

#if TELNET_ENABLE
        case Setting_TelnetPort:
          if(isintf(value) && value != NAN && value > 0.0f && value < 65536.0f)
              ethernet.telnet_port = (uint16_t)value;
          else
              status = Status_InvalidStatement; //out of range...
          break;
#endif

#if WEBSOCKET_ENABLE
        case Setting_WebSocketPort:
          if(isintf(value) && value != NAN && value > 0.0f && value < 65536.0f)
              ethernet.websocket_port = (uint16_t)value;
          else
              status = Status_InvalidStatement; //out of range...
          break;
#endif

#if HTTP_ENABLE
        case Setting_HttpPort:
          if((claimed = (value != NAN && value > 0.0f && value < 65536.0f)
              ethernet.http_port = (uint16_t)value;
          else
              status = Status_InvalidStatement; //out of range...
          break;
#endif

        case Setting_IpMode:
          if(isintf(value) && value >= 0.0f && value <= 2.0f)
              ethernet.ip_mode = (ip_mode_t)(uint8_t)value;
          else
              status = Status_InvalidStatement; // out of range...
          break;

        case Setting_IpAddress:
          {
              ip4_addr_t addr;
              if(ip4addr_aton(svalue, &addr) == 1)
                  set_addr(ethernet.ip, &addr);
              else
                  status = Status_InvalidStatement;
          }
          break;

        case Setting_Gateway:
          {
              ip4_addr_t addr;
              if(ip4addr_aton(svalue, &addr) == 1)
                  set_addr(ethernet.gateway, &addr);
              else
                  status = Status_InvalidStatement;
          }
          break;

        case Setting_NetMask:
          {
              ip4_addr_t addr;
              if(ip4addr_aton(svalue, &addr) == 1)
                  set_addr(ethernet.mask, &addr);
              else
                  status = Status_InvalidStatement;
          }
          break;

        default:
            status = Status_Unhandled;
            break;
    }

    if(status == Status_OK)
        hal.nvs.memcpy_to_nvs(driver_settings.nvs_address, (uint8_t *)&ethernet, sizeof(network_settings_t), true);

    return status == Status_Unhandled && driver_settings.set ? driver_settings.set(setting, value, svalue) : status;
}

void ethernet_settings_report (setting_type_t setting)
{
    bool reported = true;

    switch(setting) {

        case Setting_NetworkServices:
            report_uint_setting(setting, ethernet.services.mask);
            break;

#if LWIP_NETIF_HOSTNAME
        case Setting_Hostname:
            report_string_setting(setting, ethernet.hostname);
            break;
#endif

        case Setting_IpMode:
            report_uint_setting(setting, ethernet.ip_mode);
            break;

        case Setting_IpAddress:
            if(ethernet.ip_mode != IpMode_DHCP) {
                char ip[IP4ADDR_STRLEN_MAX];
                report_string_setting(setting, ip4addr_ntoa_r((ip4_addr_t *)&ethernet.ip, ip, IP4ADDR_STRLEN_MAX));
            }
            break;

        case Setting_Gateway:
            if(ethernet.ip_mode != IpMode_DHCP) {
                char ip[IP4ADDR_STRLEN_MAX];
                report_string_setting(setting, ip4addr_ntoa_r((ip4_addr_t *)&ethernet.gateway, ip, IP4ADDR_STRLEN_MAX));
            }
            break;

        case Setting_NetMask:
            if(ethernet.ip_mode != IpMode_DHCP) {
                char ip[IP4ADDR_STRLEN_MAX];
                report_string_setting(setting, ip4addr_ntoa_r((ip4_addr_t *)&ethernet.mask, ip, IP4ADDR_STRLEN_MAX));
            }
            break;

#if TELNET_ENABLE
        case Setting_TelnetPort:
            report_uint_setting(setting, ethernet.telnet_port);
            break;
#endif

#if WEBSOCKET_ENABLE
        case Setting_WebSocketPort:
            report_uint_setting(setting, ethernet.websocket_port);
            break;
#endif

        default:
            reported = false;
            break;
    }

    if(!reported && driver_settings.report)
        driver_settings.report(setting);
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

    hal.nvs.memcpy_to_nvs(driver_settings.nvs_address, (uint8_t *)&ethernet, sizeof(network_settings_t), true);

    if(driver_settings.restore)
        driver_settings.restore();
}

static void ethernet_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&ethernet, driver_settings.nvs_address, sizeof(network_settings_t), true) != NVS_TransferResult_OK)
        ethernet_settings_restore();

    if(driver_settings.load)
        driver_settings.load();
}

bool grbl_enet_init (network_settings_t *settings)
{
    if((hal.driver_settings.nvs_address = nvs_alloc(sizeof(network_settings_t)))) {
        memcpy(&driver_settings, &hal.driver_settings, sizeof(driver_setting_ptrs_t));
        hal.driver_cap.ethernet = On;
        hal.driver_settings.set = ethernet_setting;
        hal.driver_settings.report = ethernet_settings_report;
        hal.driver_settings.restore = ethernet_settings_restore;
        hal.driver_settings.load = ethernet_settings_load;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = reportIP;

        details.on_report_settings = grbl.on_report_settings;
        grbl.on_report_settings = on_report_settings;
    }

    return driver_settings.nvs_address != 0;
}

#endif
