#ifndef __NETWORKING_H__
#define __NETWORKING_H__

#include "driver.h"

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
#include "lwip/opt.h"

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

#endif
