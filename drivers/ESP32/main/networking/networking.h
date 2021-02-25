#ifndef __NETWORKING_H__
#define __NETWORKING_H__

//*****************************************************************************
//
// lwIP Options
//
//*****************************************************************************

#include "lwipopts.h"

// If no OS increase TX buffer size to hold the largest message generated and then some.
// The list settings $$ command is currently the big one.
#if NO_SYS > 0 && !defined(TX_BUFFER_SIZE)
#define TX_BUFFER_SIZE 1024 // must be a power of 2
#endif

#define SOCKET_TIMEOUT 0
#ifndef TCP_SLOW_INTERVAL
#define TCP_SLOW_INTERVAL 500
#endif

//*****************************************************************************

#ifdef ARDUINO
#include "../driver.h"
#else
#include "driver.h"
#endif

#if TELNET_ENABLE
#include "TCPStream.h"
#endif

#if WEBSOCKET_ENABLE
#include "WsStream.h"
#endif

//*****************************************************************************
//
// Ensure that AUTOIP COOP option is configured correctly.
//
//*****************************************************************************
#undef LWIP_DHCP_AUTOIP_COOP
#define LWIP_DHCP_AUTOIP_COOP   ((LWIP_DHCP) && (LWIP_AUTOIP))

//*****************************************************************************
//
// lwIP API Header Files
//
//*****************************************************************************
#include <stdint.h>
#include "lwip/api.h"
#include "lwip/netifapi.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "lwip/tcpip.h"
#include "lwip/sockets.h"
#include "lwip/mem.h"
#include "lwip/stats.h"
#include "lwip/def.h"
#include "lwip/ip_addr.h"

#if NO_SYS
#include "lwip/sys.h"
typedef uint32_t TickType_t;
#define configTICK_RATE_HZ 1000
#define xTaskGetTickCount() sys_now()
#endif

#ifndef SYS_ARCH_PROTECT
#define lev 1
#define SYS_ARCH_PROTECT(lev)
#define SYS_ARCH_UNPROTECT(lev)
#define SYS_ARCH_DECL_PROTECT(lev)

#endif

#endif
