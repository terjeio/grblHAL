//
// TCPStream.c - lw-IP/FreeRTOS stream implementation
//
// v1.0 / 2018-09-21 / Io Engineering / Terje
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
#include <string.h>
#include "GRBL/grbl.h"

#include "lwipopts.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include "lwip/netifapi.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "lwip/tcpip.h"
#include "lwip/sockets.h"
#include "lwip/mem.h"
#include "lwip/stats.h"
#include "lwip/def.h"
#include "lwip/opt.h"
#include "lwip/priv/tcp_priv.h"
#include "lwip/timeouts.h"

#include "driver.h"
#include "serial.h"
#include "TCPStream.h"

#define SOCKET_TIMEOUT 0
#define BUFCOUNT(head, tail, size) ((head >= tail) ? (head - tail) : (size - tail + head))

typedef enum
{
    TCPState_Idle,
    TCPState_Listen,
    TCPState_Connected,
} TCPState_t;

typedef struct pbuf_entry
{
    struct pbuf *pbuf;
    struct pbuf_entry *next;
} pbuf_entry_t;

typedef struct
{
    uint16_t port;
    TCPState_t state;
    bool linkLost;
    uint32_t timeout;
    uint32_t timeoutMax;
    struct tcp_pcb *pcbConnect;
    struct tcp_pcb *pcbListen;
    pbuf_entry_t queue[PBUF_POOL_SIZE];
    pbuf_entry_t *rcvTail;
    pbuf_entry_t *rcvHead;
    struct pbuf *pbufHead;
    struct pbuf *pbufCurrent;
    uint32_t bufferIndex;
    TickType_t lastSendTime;
    err_t lastErr;
    uint8_t errorCount;
    uint8_t reconnectCount;
    uint8_t connectCount;
} SessionData_t;

const SessionData_t defaultSettings =
{
    .port = 23,
    .state = TCPState_Listen,
    .timeout = 0,
    .timeoutMax = SOCKET_TIMEOUT,
    .pcbConnect = NULL,
    .pcbListen = NULL,
    .pbufHead = NULL,
    .pbufCurrent = NULL,
    .bufferIndex = 0,
    .lastSendTime = 0,
    .linkLost = false,
    .connectCount = 0,
    .reconnectCount = 0,
    .errorCount = 0,
    .lastErr = ERR_OK
};

static char rxbuf[RX_BUFFER_SIZE];
static char txbuf[TX_BUFFER_SIZE];
static volatile uint16_t rx_head = 0, rx_tail = 0, rx_overflow = 0;
static volatile uint16_t tx_head = 0, tx_tail = 0;
static SessionData_t streamSession;

void TCPStreamInit (void)
{
    memcpy(&streamSession, &defaultSettings, sizeof(SessionData_t));

    // turn the packet queue array into a circular linked list
    uint_fast8_t idx;
    for(idx = 0; idx < PBUF_POOL_SIZE; idx++) {
        streamSession.queue[idx].next = &streamSession.queue[idx == PBUF_POOL_SIZE - 1 ? 0 : idx + 1];
    }

    streamSession.rcvTail = streamSession.rcvHead = &streamSession.queue[0];
}

//
// TCPStreamGetC - returns -1 if no data available
//
int16_t TCPStreamGetC (void)
{
    int16_t data;
    uint16_t bptr = rx_tail;

    if(bptr == rx_head)
        return -1; // no data available else EOF

    data = rxbuf[bptr++];                   // Get next character, increment tmp pointer
    rx_tail = bptr & (RX_BUFFER_SIZE - 1);  // and update pointer

    return data;
}

inline uint16_t TCPStreamRxCount (void)
{
    uint16_t head = rx_head, tail = rx_tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

uint16_t TCPStreamRxFree (void)
{
    return (RX_BUFFER_SIZE - 1) - TCPStreamRxCount();
}

void TCPStreamRxFlush (void)
{
    rx_tail = rx_head;
}

void TCPStreamRxCancel (void)
{
    rxbuf[rx_head] = CAN;
    rx_tail = rx_head;
    rx_head = (rx_tail + 1) & (RX_BUFFER_SIZE - 1);
}


bool TCPStreamPutC (const char c) {

    uint32_t next_head = (tx_head + 1) & (TX_BUFFER_SIZE - 1);  // Get and update head pointer

    while(tx_tail == next_head) {                               // Buffer full, block until space is available...
        if(!hal.stream_blocking_callback())
            return false;
    }

    txbuf[tx_head] = c;                                         // Add data to buffer
    tx_head = next_head;                                        // and update head pointer

    return true;
}

void TCPStreamWriteS (const char *data)
{
    char c, *ptr = (char *)data;

    while((c = *ptr++) != '\0')
        TCPStreamPutC(c);
}

void TCPStreamWriteLn (const char *data)
{
    TCPStreamWriteS(data);
    TCPStreamWriteS(EOL);
}

void TCPStreamWrite (const char *data, unsigned int length)
{
    char *ptr = (char *)data;

    while(length--)
        TCPStreamPutC(*ptr++);

}

uint16_t TCPStreamTxCount(void) {

    uint16_t head = tx_head, tail = tx_tail;

    return BUFCOUNT(head, tail, TX_BUFFER_SIZE);
}

static int16_t streamReadTXC (void)
{
    int16_t data;
    uint16_t bptr = tx_tail;

    if(bptr == tx_head)
        return -1; // no data available else EOF

    data = txbuf[bptr++];                   // Get next character, increment tmp pointer
    tx_tail = bptr & (TX_BUFFER_SIZE - 1);  // and update pointer

    return data;
}


static void streamBufferRX (char c) {

    uint16_t bptr = (rx_head + 1) & (RX_BUFFER_SIZE - 1);    // Get next head pointer

    if(bptr == rx_tail) {                           // If buffer full TODO: remove this check?
        rx_overflow = 1;                            // flag overlow
    } else {
        if(!hal.protocol_process_realtime || hal.protocol_process_realtime(c)) {
            rxbuf[rx_head] = c;   // Add data to buffer
            rx_head = bptr;                         // and update pointer
        }
    }
}

static void streamFreeBuffers (SessionData_t *streamSession)
{
    SYS_ARCH_DECL_PROTECT(lev);
    SYS_ARCH_PROTECT(lev);

    // Free any buffer chain currently beeing processed
    if(streamSession->pbufHead != NULL) {
        pbuf_free(streamSession->pbufHead);
        streamSession->pbufHead = streamSession->pbufCurrent = NULL;
        streamSession->bufferIndex = 0;
    }

    // Free any queued buffer chains
    while(streamSession->rcvTail != streamSession->rcvHead) {
        pbuf_free(streamSession->rcvTail->pbuf);
        streamSession->rcvTail = streamSession->rcvTail->next;
    }

    SYS_ARCH_UNPROTECT(lev);
}

void TCPStreamNotifyLinkStatus (bool up)
{
    if(!up)
        streamSession.linkLost = true;
}

static void streamError (void *arg, err_t err)
{
    SessionData_t *streamSession = arg;

    streamFreeBuffers(streamSession);

    streamSession->state = TCPState_Listen;
    streamSession->errorCount++;
    streamSession->lastErr = err;
    streamSession->pcbConnect = NULL;
    streamSession->timeout = 0;
    streamSession->pbufHead = streamSession->pbufCurrent = NULL;
    streamSession->bufferIndex = 0;
    streamSession->lastSendTime = 0;
    streamSession->linkLost = false;
    streamSession->rcvTail = streamSession->rcvHead;
}

static err_t streamPoll (void *arg, struct tcp_pcb *pcb)
{
    SessionData_t *streamSession = arg;

    streamSession->timeout++;

    if(streamSession->timeoutMax && streamSession->timeout > streamSession->timeoutMax)
        tcp_abort(pcb);

    return ERR_OK;
}

//
// Queue incoming packet for processing
//
static err_t streamReceive (void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
    if(err == ERR_OK) {

        SessionData_t *streamSession = arg;

        if(p) {
            // Attempt to queue data
            SYS_ARCH_DECL_PROTECT(lev);
            SYS_ARCH_PROTECT(lev);

            if(streamSession->rcvHead->next == streamSession->rcvTail) {
                // Queue full, discard
                SYS_ARCH_UNPROTECT(lev);
                pbuf_free(p);
            } else {
                streamSession->rcvHead->pbuf = p;
                streamSession->rcvHead = streamSession->rcvHead->next;
                SYS_ARCH_UNPROTECT(lev);
            }
        } else {
            // Null packet received, means close connection
            tcp_arg(pcb, NULL);
            tcp_recv(pcb, NULL);
            tcp_sent(pcb, NULL);
            tcp_err(pcb, NULL);
            tcp_poll(pcb, NULL, 1);

            tcp_close(pcb);

            streamFreeBuffers(streamSession);

            streamSession->pcbConnect = NULL;
            streamSession->state = TCPState_Listen;

            selectStream(StreamSetting_Serial);
        }
    }

    return ERR_OK;
}

static err_t streamSent (void *arg, struct tcp_pcb *pcb, u16_t ui16len)
{
    ((SessionData_t *)arg)->timeout = 0;

    return ERR_OK;
}

static err_t TCPStreamAccept (void *arg, struct tcp_pcb *pcb, err_t err)
{
    SessionData_t *streamSession = arg;

    if(streamSession->state != TCPState_Listen) {

        if(!streamSession->linkLost)
            return ERR_CONN; // Busy, refuse connection

        // Link was previously lost, abort current connection

        tcp_abort(streamSession->pcbConnect);

        streamFreeBuffers(streamSession);

        streamSession->linkLost = false;
    }

    streamSession->pcbConnect = pcb;
    streamSession->state = TCPState_Connected;

    tcp_accepted(pcb);

    TCPStreamRxFlush();

    streamSession->timeout = 0;

    tcp_setprio(pcb, TCP_PRIO_MIN);
    tcp_recv(pcb, streamReceive);
    tcp_err(pcb, streamError);
    tcp_poll(pcb, streamPoll, 1000 / TCP_SLOW_INTERVAL);
    tcp_sent(pcb, streamSent);

    selectStream(StreamSetting_WiFi);
	hal.stream_write_all("[MSG:WIFI CONNECTED]\r\n");

    return ERR_OK;
}

void TCPStreamClose (void)
{
    if(streamSession.pcbConnect != NULL) {
        tcp_arg(streamSession.pcbConnect, NULL);
        tcp_recv(streamSession.pcbConnect, NULL);
        tcp_sent(streamSession.pcbConnect, NULL);
        tcp_err(streamSession.pcbConnect, NULL);
        tcp_poll(streamSession.pcbConnect, NULL, 1);

        tcp_abort(streamSession.pcbConnect);
        streamFreeBuffers(&streamSession);
    }

    if(streamSession.pcbListen != NULL) {
        tcp_close(streamSession.pcbListen);
        streamFreeBuffers(&streamSession);
    }

    streamSession.state = TCPState_Idle;
    streamSession.pcbConnect = streamSession.pcbListen = NULL;
    streamSession.timeout = 0;
    streamSession.rcvTail = streamSession.rcvHead;
    streamSession.pbufHead = streamSession.pbufCurrent = NULL;
    streamSession.bufferIndex = 0;
    streamSession.lastSendTime = 0;
    streamSession.linkLost = false;
}

void TCPStreamListen (uint16_t port)
{
//    ASSERT(port != 0);

    streamSession.state = TCPState_Listen;
    streamSession.pcbConnect = NULL;
    streamSession.timeout = 0;
    streamSession.timeoutMax = SOCKET_TIMEOUT;
    streamSession.port = port;
    streamSession.rcvTail = streamSession.rcvHead;
    streamSession.pbufHead = streamSession.pbufCurrent = NULL;
    streamSession.bufferIndex = 0;
    streamSession.lastSendTime = 0;
    streamSession.linkLost = false;

    void *pcb = tcp_new();
    tcp_bind(pcb, IP_ADDR_ANY, port);

    streamSession.pcbListen = tcp_listen(pcb);

    tcp_arg(streamSession.pcbListen, &streamSession);
    tcp_accept(streamSession.pcbListen, TCPStreamAccept);
}

//
// Process data for streaming
//
void TCPStreamHandler (void)
{
    static uint8_t tempBuffer[PBUF_POOL_BUFSIZE];

    if(streamSession.state != TCPState_Connected)
        return;

    uint8_t *payload = streamSession.pbufCurrent ? streamSession.pbufCurrent->payload : NULL;

    SYS_ARCH_DECL_PROTECT(lev);

    // 1. Process input stream
    while(TCPStreamRxFree()) {

        // Get next pbuf chain to process
        if(streamSession.pbufHead == NULL && streamSession.rcvTail != streamSession.rcvHead) {
            SYS_ARCH_PROTECT(lev);
            streamSession.pbufCurrent = streamSession.pbufHead = streamSession.rcvTail->pbuf;
            streamSession.rcvTail = streamSession.rcvTail->next;
            streamSession.bufferIndex = 0;
            SYS_ARCH_UNPROTECT(lev);
            payload = streamSession.pbufCurrent ? streamSession.pbufCurrent->payload : NULL;
        }

        if(payload == NULL)
            break; // No more data to be processed...

        // Add data to input stream buffer
        streamBufferRX(payload[streamSession.bufferIndex++]);

        if(streamSession.bufferIndex >= streamSession.pbufCurrent->len) {
            streamSession.pbufCurrent = streamSession.pbufCurrent->next;
            streamSession.bufferIndex = 0;
            payload = streamSession.pbufCurrent ? streamSession.pbufCurrent->payload : NULL;
        }

        // ACK current pbuf chain when all data has been processed
        if((streamSession.pbufCurrent == NULL) && (streamSession.bufferIndex == 0)) {
            tcp_recved(streamSession.pcbConnect, streamSession.pbufHead->tot_len);
            pbuf_free(streamSession.pbufHead);
            streamSession.pbufCurrent = streamSession.pbufHead = NULL;
            streamSession.bufferIndex = 0;
        }
    }

  //  tcp_output(streamSession.pcbConnect);

    int32_t TXCount;

    // 2. Process output stream
    if((TXCount = TCPStreamTxCount()) && tcp_sndbuf(streamSession.pcbConnect) && streamSession.pcbConnect->snd_queuelen < TCP_SND_QUEUELEN) {

        u16_t idx;

        if(TXCount > tcp_sndbuf(streamSession.pcbConnect))
            TXCount = tcp_sndbuf(streamSession.pcbConnect);

        while(TXCount && streamSession.pcbConnect->snd_queuelen < TCP_SND_QUEUELEN)
        {
            idx = 0;

            while(TXCount && idx < sizeof(tempBuffer)) {
                tempBuffer[idx] = (uint8_t)streamReadTXC();
                idx++;
                TXCount--;
            }

            tcp_write(streamSession.pcbConnect, tempBuffer, idx, 1);
        }

        tcp_output(streamSession.pcbConnect);
        streamSession.lastSendTime = xTaskGetTickCount();
    }
}
