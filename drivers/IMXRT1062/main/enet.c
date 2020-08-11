/*
  enet.c - lwIP driver glue code for IMXRT1062 processor (on Teensy 4.1 board)

  Part of GrblHAL

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

#include "src/grbl/report.h"
#include "src/networking/TCPStream.h"
#include "src/networking/WsStream.h"

static volatile bool linkUp = false;
static char IPAddress[IP4ADDR_STRLEN_MAX];
static network_settings_t network;
static network_services_t services = {0};

char *enet_ip_address (void)
{
    return IPAddress;
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
    static uint32_t last_ms;
    uint32_t ms;

    enet_proc_input();

    ms = millis();
    if (ms - last_ms > 100)
    {
        last_ms = ms;

      #if TELNET_ENABLE
        if(services.telnet)
            TCPStreamPoll();
      #endif
      #if WEBSOCKET_ENABLE
        if(services.websocket) {
            WsStreamPoll();
        }
      #endif

        enet_poll();
    }
}

bool grbl_enet_init (network_settings_t *settings)
{
    *IPAddress = '\0';

    memcpy(&network, settings, sizeof(network_settings_t));

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

    return true;
}

static inline void set_addr (char *ip, ip4_addr_t *addr)
{
    memcpy(ip, addr, sizeof(ip4_addr_t));
}

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

        case Setting_IpMode:
          if(isintf(value) && value >= 0.0f && value <= 2.0f) {
              status = Status_OK;
              driver_settings.network.ip_mode = (ip_mode_t)(uint8_t)value;
          }
          else
              status = Status_InvalidStatement; // out of range...
          break;

        case Setting_IpAddress:
          {
              ip4_addr_t addr;
              if(ip4addr_aton(svalue, &addr) == 1) {
                  status = Status_OK;
                  set_addr(driver_settings.network.ip, &addr);
              } else
                  status = Status_InvalidStatement;
          }
          break;

        case Setting_Gateway:
          {
              ip4_addr_t addr;
              if(ip4addr_aton(svalue, &addr) == 1) {
                  status = Status_OK;
                  set_addr(driver_settings.network.gateway, &addr);
              } else
                  status = Status_InvalidStatement;
          }
          break;

        case Setting_NetMask:
          {
              ip4_addr_t addr;
              if(ip4addr_aton(svalue, &addr) == 1) {
                  status = Status_OK;
                  set_addr(driver_settings.network.mask, &addr);
              } else
                  status = Status_InvalidStatement;
          }
          break;

        default:
            break;
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
                char ip[IP4ADDR_STRLEN_MAX];
                report_string_setting(setting, ip4addr_ntoa_r((ip4_addr_t *)&driver_settings.network.ip, ip, IP4ADDR_STRLEN_MAX));
            }
            break;

        case Setting_Gateway:
            if(driver_settings.network.ip_mode != IpMode_DHCP) {
                char ip[IP4ADDR_STRLEN_MAX];
                report_string_setting(setting, ip4addr_ntoa_r((ip4_addr_t *)&driver_settings.network.gateway, ip, IP4ADDR_STRLEN_MAX));
            }
            break;

        case Setting_NetMask:
            if(driver_settings.network.ip_mode != IpMode_DHCP) {
                char ip[IP4ADDR_STRLEN_MAX];
                report_string_setting(setting, ip4addr_ntoa_r((ip4_addr_t *)&driver_settings.network.mask, ip, IP4ADDR_STRLEN_MAX));
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

        default:
            break;
    }
}

void ethernet_settings_restore (void)
{
    strcpy(driver_settings.network.hostname, NETWORK_HOSTNAME);

    ip4_addr_t addr;

    driver_settings.network.ip_mode = (ip_mode_t)NETWORK_IPMODE;

    if(ip4addr_aton(NETWORK_IP, &addr) == 1)
        set_addr(driver_settings.network.ip, &addr);

    if(ip4addr_aton(NETWORK_GATEWAY, &addr) == 1)
        set_addr(driver_settings.network.gateway, &addr);

#if NETWORK_IPMODE == 0
    if(ip4addr_aton(NETWORK_MASK, &addr) == 1)
        set_addr(driver_settings.network.mask, &addr);
#else
    if(ip4addr_aton("255.255.255.0", &addr) == 1)
        set_addr(driver_settings.network.mask, &addr);
#endif

    driver_settings.network.services.mask = 0;
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
