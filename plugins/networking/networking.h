#ifndef __NETWORKING_H__
#define __NETWORKING_H__

#ifdef ARDUINO
#include "../../driver.h"
#else
#include "driver.h"
#endif

#if TELNET_ENABLE
#include "TCPSTream.h"
#endif

#if WEBSOCKET_ENABLE
#include "WsSTream.h"
#endif

//*****************************************************************************
//
// lwIP Options
//
//*****************************************************************************
#ifdef ARDUINO
#include "lwip/opt.h"
#else
#include "lwipopts.h"
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
//#include "lwip/tcp_impl.h"
//#include "lwip/timers.h"

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
