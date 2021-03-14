//
// TCPStream.c - lwIP stream implementation, raw "Telnet"
//
// v1.3 / 2021-03-13 / Io Engineering / Terje
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

#ifdef ARDUINO
#include "../driver.h"
#else
#include "driver.h"
#endif

#if TELNET_ENABLE

#include <assert.h>
#include <string.h>

#include "TCPStream.h"
#include "networking.h"

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
    stream_rx_buffer_t rxbuf;
    stream_tx_buffer_t txbuf;
    TickType_t lastSendTime;
    err_t lastErr;
    uint8_t errorCount;
    uint8_t reconnectCount;
    uint8_t connectCount;
} sessiondata_t;

static const sessiondata_t defaultSettings =
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
    .rxbuf = {0},
    .txbuf = {0},
    .lastSendTime = 0,
    .linkLost = false,
    .connectCount = 0,
    .reconnectCount = 0,
    .errorCount = 0,
    .lastErr = ERR_OK
};

static sessiondata_t streamSession;

void TCPStreamInit (void)
{
    memcpy(&streamSession, &defaultSettings, sizeof(sessiondata_t));

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
    uint_fast16_t bptr = streamSession.rxbuf.tail;

    if(bptr == streamSession.rxbuf.head)
        return -1; // no data available else EOF

    data = streamSession.rxbuf.data[bptr++];                // Get next character, increment tmp pointer
    streamSession.rxbuf.tail = bptr & (RX_BUFFER_SIZE - 1); // and update pointer

    return data;
}

inline uint16_t TCPStreamRxCount (void)
{
    uint_fast16_t head = streamSession.rxbuf.head, tail = streamSession.rxbuf.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

uint16_t TCPStreamRxFree (void)
{
    return (RX_BUFFER_SIZE - 1) - TCPStreamRxCount();
}

void TCPStreamRxFlush (void)
{
    streamSession.rxbuf.tail = streamSession.rxbuf.head;
}

void TCPStreamRxCancel (void)
{
    streamSession.rxbuf.data[streamSession.rxbuf.head] = ASCII_CAN;
    streamSession.rxbuf.tail = streamSession.rxbuf.head;
    streamSession.rxbuf.head = (streamSession.rxbuf.tail + 1) & (RX_BUFFER_SIZE - 1);
}

bool TCPStreamSuspendInput (bool suspend)
{
    return stream_rx_suspend(&streamSession.rxbuf, suspend);
}

static bool streamBufferRX (char c)
{
    // discard input if MPG has taken over...
    if(hal.stream.type != StreamType_MPG) {

        uint_fast16_t bptr = (streamSession.rxbuf.head + 1) & (RX_BUFFER_SIZE - 1); // Get next head pointer

        if(bptr == streamSession.rxbuf.tail)                        // If buffer full
            streamSession.rxbuf.overflow = true;                    // flag overflow
        else if(c == CMD_TOOL_ACK && !streamSession.rxbuf.backup) {
            stream_rx_backup(&streamSession.rxbuf);
            hal.stream.read = TCPStreamGetC; // restore normal input
        } else if(!hal.stream.enqueue_realtime_command(c)) {        // If not a real time command
            streamSession.rxbuf.data[streamSession.rxbuf.head] = c; // add data to buffer
            streamSession.rxbuf.head = bptr;                        // and update pointer
        }
    }

    return !streamSession.rxbuf.overflow;
}

bool TCPStreamPutC (const char c)
{
    uint32_t next_head = (streamSession.txbuf.head + 1) & (TX_BUFFER_SIZE - 1);  // Get and update head pointer

    while(streamSession.txbuf.tail == next_head) {                               // Buffer full, block until space is available...
        if(!hal.stream_blocking_callback())
            return false;
    }

    streamSession.txbuf.data[streamSession.txbuf.head] = c;                     // Add data to buffer
    streamSession.txbuf.head = next_head;                                       // and update head pointer

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
    TCPStreamWriteS(ASCII_EOL);
}

void TCPStreamWrite (const char *data, unsigned int length)
{
    char *ptr = (char *)data;

    while(length--)
        TCPStreamPutC(*ptr++);
}

uint16_t TCPStreamTxCount(void) {

    uint_fast16_t head = streamSession.txbuf.head, tail = streamSession.txbuf.tail;

    return BUFCOUNT(head, tail, TX_BUFFER_SIZE);
}

static int16_t streamReadTXC (void)
{
    int16_t data;
    uint_fast16_t bptr = streamSession.txbuf.tail;

    if(bptr == streamSession.txbuf.head)
        return -1; // no data available else EOF

    data = streamSession.txbuf.data[bptr++];                 // Get next character, increment tmp pointer
    streamSession.txbuf.tail = bptr & (TX_BUFFER_SIZE - 1);  // and update pointer

    return data;
}

void TCPStreamTxFlush (void)
{
    streamSession.txbuf.tail = streamSession.txbuf.head;
}

static void streamFreeBuffers (sessiondata_t *session)
{
    SYS_ARCH_DECL_PROTECT(lev);
    SYS_ARCH_PROTECT(lev);

    // Free any buffer chain currently beeing processed
    if(session->pbufHead != NULL) {
        pbuf_free(session->pbufHead);
        session->pbufHead = session->pbufCurrent = NULL;
        session->bufferIndex = 0;
    }

    // Free any queued buffer chains
    while(session->rcvTail != session->rcvHead) {
        pbuf_free(session->rcvTail->pbuf);
        session->rcvTail = session->rcvTail->next;
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
    sessiondata_t *session = arg;

    streamFreeBuffers(session);

    session->state = TCPState_Listen;
    session->errorCount++;
    session->lastErr = err;
    session->pcbConnect = NULL;
    session->timeout = 0;
    session->pbufHead = session->pbufCurrent = NULL;
    session->bufferIndex = 0;
    session->lastSendTime = 0;
    session->linkLost = false;
    session->rcvTail = session->rcvHead;
}

static err_t streamPoll (void *arg, struct tcp_pcb *pcb)
{
    sessiondata_t *session = arg;

    session->timeout++;

    if(session->timeoutMax && session->timeout > session->timeoutMax)
        tcp_abort(pcb);

    return ERR_OK;
}

static void closeSocket (sessiondata_t *session, struct tcp_pcb *pcb)
{
    tcp_arg(pcb, NULL);
    tcp_recv(pcb, NULL);
    tcp_sent(pcb, NULL);
    tcp_err(pcb, NULL);
    tcp_poll(pcb, NULL, 1);

    tcp_close(pcb);

    streamFreeBuffers(session);

    session->pcbConnect = NULL;
    session->state = TCPState_Listen;

    // Switch grbl I/O stream back to UART
    selectStream(StreamType_Serial);
}

//
// Queue incoming packet for processing
//
static err_t streamReceive (void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
    if(err == ERR_OK) {

        sessiondata_t *session = arg;

        if(p) {
            // Attempt to queue data
            SYS_ARCH_DECL_PROTECT(lev);
            SYS_ARCH_PROTECT(lev);

            if(session->rcvHead->next == session->rcvTail) {
                // Queue full, discard
                SYS_ARCH_UNPROTECT(lev);
                pbuf_free(p);
            } else {
                session->rcvHead->pbuf = p;
                session->rcvHead = session->rcvHead->next;
                SYS_ARCH_UNPROTECT(lev);
            }
        } else // Null packet received, means close connection
            closeSocket(session, pcb);;
    }

    return ERR_OK;
}

static err_t streamSent (void *arg, struct tcp_pcb *pcb, u16_t ui16len)
{
    ((sessiondata_t *)arg)->timeout = 0;

    return ERR_OK;
}

static err_t TCPStreamAccept (void *arg, struct tcp_pcb *pcb, err_t err)
{
    sessiondata_t *session = arg;

    if(session->state != TCPState_Listen) {

        if(!session->linkLost)
            return ERR_CONN; // Busy, refuse connection

        // Link was previously lost, abort current connection

        tcp_abort(session->pcbConnect);

        streamFreeBuffers(session);

        session->linkLost = false;
    }

    session->pcbConnect = pcb;
    session->state = TCPState_Connected;

    TCPStreamRxFlush();
    TCPStreamTxFlush();

    tcp_accepted(pcb);

    session->timeout = 0;

    tcp_setprio(pcb, TCP_PRIO_MIN);
    tcp_recv(pcb, streamReceive);
    tcp_err(pcb, streamError);
    tcp_poll(pcb, streamPoll, 1000 / TCP_SLOW_INTERVAL);
    tcp_sent(pcb, streamSent);

    // Switch grbl I/O stream to TCP/IP connection
    selectStream(StreamType_Telnet);

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

    // Switch grbl I/O stream back to UART
    selectStream(StreamType_Serial);
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
void TCPStreamPoll (void)
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

//    tcp_output(streamSession.pcbConnect);

    int_fast16_t TXCount;

    // 2. Process output stream
    if((TXCount = TCPStreamTxCount()) && tcp_sndbuf(streamSession.pcbConnect) && streamSession.pcbConnect->snd_queuelen < TCP_SND_QUEUELEN) {

        uint_fast16_t idx;

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

            tcp_write(streamSession.pcbConnect, tempBuffer, (u16_t)idx, 1);
        }

        tcp_output(streamSession.pcbConnect);
        streamSession.lastSendTime = xTaskGetTickCount();
    }
}

#endif

