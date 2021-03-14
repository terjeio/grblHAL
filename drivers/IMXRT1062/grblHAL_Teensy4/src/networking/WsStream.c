//
// WsStream.c - lwIP websocket stream implementation
//
// v1.3 / 2021-03-13 / Io Engineering / Terje
//

/*

Copyright (c) 2019-2021, Terje Io
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

#if WEBSOCKET_ENABLE

#include <assert.h>
#include <stdlib.h>
#include <string.h>

#include "networking.h"
#include "WsStream.h"
#include "base64.h"
#include "sha1.h"
#include "utils.h"
#include "strutils.h"

#include "grbl/grbl.h"

//#define WSDEBUG

#define CRLF "\r\n"
#define SOCKET_TIMEOUT 0
#define MAX_HTTP_HEADER_SIZE 512
#define FRAME_NONE 0xFF

static const char WS_GUID[] = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
static const char WS_KEY[] = "Sec-WebSocket-Key: ";
static const char WS_PROT[] = "Sec-WebSocket-Protocol: ";
static const char WS_RSP[] = "HTTP/1.1 101 Switching Protocols" CRLF \
                             "Upgrade: websocket" CRLF \
                             "Connection: Upgrade" CRLF \
                             "Sec-WebSocket-Accept: ";
static const char HTTP_400[] = "HTTP/1.1 400" CRLF \
                               "Status: 400 Bad Request" CRLF CRLF;
static const char HTTP_500[] = "HTTP/1.1 500" CRLF \
                               "Status: 500 Internal Server Error" CRLF CRLF;

typedef enum {
    WsOpcode_Continuation = 0x00,
    WsOpcode_Text = 0x1,
    WsOpcode_Binary = 0x2,
    WsOpcode_Close = 0x8,
    WsOpcode_Ping = 0x9,
    WsOpcode_Pong = 0xA
} websocket_opcode_t;

typedef enum
{
    WsState_Idle,
    WsState_Listen,
    WsState_Connected,
    WsStateClosing
} websocket_state_t;

typedef union {
    uint8_t token;
    struct {
        uint8_t opcode :4,
                rsv3   :1,
                rsv2   :1,
                rsv1   :1,
                fin    :1;
    };
} ws_frame_start_t;

typedef struct {
    uint32_t idx;
    uint32_t payload_len;
    uint32_t payload_rem;
    uint32_t rx_index;
    uint8_t *frame;
    uint32_t mask;
    bool masked;
    bool complete;
    uint8_t data[13];
} frame_header_t;

typedef struct pbuf_entry
{
    struct pbuf *pbuf;
    struct pbuf_entry *next;
} pbuf_entry_t;

typedef struct ws_sessiondata
{
    uint16_t port;
    websocket_state_t state;
    ws_frame_start_t ftype;
    websocket_opcode_t fragment_opcode;
    ws_frame_start_t start;
    frame_header_t header;
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
    uint8_t pingCount;
    char *http_request;
    uint32_t hdrsize;
    void (*traffic_handler)(struct ws_sessiondata *session);
} ws_sessiondata_t;

static void WsConnectionHandler (ws_sessiondata_t *session);
static void WsStreamHandler (ws_sessiondata_t *session);

static const ws_frame_start_t wshdr_txt = {
  .fin    = true,
  .opcode = WsOpcode_Text
};

static const ws_frame_start_t wshdr_bin = {
  .fin    = true,
  .opcode = WsOpcode_Binary
};

static const ws_frame_start_t wshdr_ping = {
  .fin    = true,
  .opcode = WsOpcode_Ping
};

static const ws_sessiondata_t defaultSettings =
{
    .port = 80,
    .state = WsState_Listen,
    .fragment_opcode = WsOpcode_Continuation,
    .start.token = FRAME_NONE,
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
    .pingCount = 0,
    .lastErr = ERR_OK,
    .http_request = NULL,
    .hdrsize = MAX_HTTP_HEADER_SIZE,
    .traffic_handler = WsConnectionHandler
};

static ws_sessiondata_t streamSession;

void WsStreamInit (void)
{
    memcpy(&streamSession, &defaultSettings, sizeof(ws_sessiondata_t));

    // turn the packet queue array into a circular linked list
    uint_fast8_t idx;
    for(idx = 0; idx < PBUF_POOL_SIZE; idx++) {
        streamSession.queue[idx].next = &streamSession.queue[idx == PBUF_POOL_SIZE - 1 ? 0 : idx + 1];
    }

    streamSession.rcvTail = streamSession.rcvHead = &streamSession.queue[0];
}

//
// WsStreamGetC - returns -1 if no data available
//
int16_t WsStreamGetC (void)
{
    int16_t data;
    uint_fast16_t bptr = streamSession.rxbuf.tail;

    if(bptr == streamSession.rxbuf.head)
        return -1; // no data available else EOF

    data = streamSession.rxbuf.data[bptr++];                // Get next character, increment tmp pointer
    streamSession.rxbuf.tail = bptr & (RX_BUFFER_SIZE - 1); // and update pointer

    return data;
}

inline uint16_t WsStreamRxCount (void)
{
    uint_fast16_t head = streamSession.rxbuf.head, tail = streamSession.rxbuf.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

uint16_t WsStreamRxFree (void)
{
    return (RX_BUFFER_SIZE - 1) - WsStreamRxCount();
}

void WsStreamRxFlush (void)
{
    streamSession.rxbuf.tail = streamSession.rxbuf.head;
}

void WsStreamRxCancel (void)
{
    streamSession.rxbuf.data[streamSession.rxbuf.head] = ASCII_CAN;
    streamSession.rxbuf.tail = streamSession.rxbuf.head;
    streamSession.rxbuf.head = (streamSession.rxbuf.tail + 1) & (RX_BUFFER_SIZE - 1);
}

bool WsStreamSuspendInput (bool suspend)
{
    return stream_rx_suspend(&streamSession.rxbuf, suspend);
}

bool WsStreamRxInsert (char c)
{
    // discard input if MPG has taken over...
    if(hal.stream.type != StreamType_MPG) {

        uint_fast16_t bptr = (streamSession.rxbuf.head + 1) & (RX_BUFFER_SIZE - 1); // Get next head pointer

        if(bptr == streamSession.rxbuf.tail)                        // If buffer full
            streamSession.rxbuf.overflow = true;                    // flag overflow
        else if(c == CMD_TOOL_ACK && !streamSession.rxbuf.backup) {
            stream_rx_backup(&streamSession.rxbuf);
            hal.stream.read = WsStreamGetC; // restore normal input
        } if(!hal.stream.enqueue_realtime_command(c)) {             // If not a real time command
            streamSession.rxbuf.data[streamSession.rxbuf.head] = c; // add data to buffer
            streamSession.rxbuf.head = bptr;                        // and update pointer
        }
    }

    return !streamSession.rxbuf.overflow;
}

bool WsStreamPutC (const char c) {

    uint32_t next_head = (streamSession.txbuf.head + 1) & (TX_BUFFER_SIZE - 1);  // Get and update head pointer

    while(streamSession.txbuf.tail == next_head) {                               // Buffer full, block until space is available...
        if(!hal.stream_blocking_callback())
            return false;
    }

    streamSession.txbuf.data[streamSession.txbuf.head] = c;                     // Add data to buffer
    streamSession.txbuf.head = next_head;                                       // and update head pointer

    return true;
}

void WsStreamWriteS (const char *data)
{
    char c, *ptr = (char *)data;

    while((c = *ptr++) != '\0')
        WsStreamPutC(c);
}

void WsStreamWriteLn (const char *data)
{
    WsStreamWriteS(data);
    WsStreamWriteS(ASCII_EOL);
}

void WsStreamWrite (const char *data, unsigned int length)
{
    char *ptr = (char *)data;

    while(length--)
        WsStreamPutC(*ptr++);
}

uint16_t WsStreamTxCount(void) {

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

void WsStreamTxFlush (void)
{
    streamSession.txbuf.tail = streamSession.txbuf.head;
}

static void streamFreeBuffers (ws_sessiondata_t *session)
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

    // Free any http request currently beeing processed
    if(session->http_request) {
        free(session->http_request);
        session->http_request = NULL;
        session->hdrsize = MAX_HTTP_HEADER_SIZE;
    }

    if(session->header.frame)
        free(session->header.frame);

    SYS_ARCH_UNPROTECT(lev);
}

void WsStreamNotifyLinkStatus (bool up)
{
    if(!up)
        streamSession.linkLost = true;
}

static void streamError (void *arg, err_t err)
{
    ws_sessiondata_t *streamSession = arg;

    streamFreeBuffers(streamSession);

    streamSession->state = WsState_Listen;
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
    ws_sessiondata_t *streamSession = arg;

    streamSession->timeout++;

    if(streamSession->timeoutMax && streamSession->timeout > streamSession->timeoutMax)
        tcp_abort(pcb);

    return ERR_OK;
}

static void closeSocket (ws_sessiondata_t *session, struct tcp_pcb *pcb)
{
    tcp_arg(pcb, NULL);
    tcp_recv(pcb, NULL);
    tcp_sent(pcb, NULL);
    tcp_err(pcb, NULL);
    tcp_poll(pcb, NULL, 1);

    tcp_close(pcb);

    streamFreeBuffers(session);

    session->pcbConnect = NULL;
    session->state = WsState_Listen;
    session->traffic_handler = WsConnectionHandler;

    // Switch grbl I/O stream back to UART
    selectStream(StreamType_Serial);
}

//
// Queue incoming packet for processing
//
static err_t streamReceive (void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
    if(err == ERR_OK) {

        ws_sessiondata_t *session = arg;

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
            closeSocket(session, pcb);
    }

    return ERR_OK;
}

static err_t streamSent (void *arg, struct tcp_pcb *pcb, u16_t ui16len)
{
    ((ws_sessiondata_t *)arg)->timeout = 0;

    return ERR_OK;
}

static err_t WsStreamAccept (void *arg, struct tcp_pcb *pcb, err_t err)
{
    ws_sessiondata_t *session = arg;

    if(session->state != WsState_Listen) {

        if(!session->linkLost)
            return ERR_CONN; // Busy, refuse connection

        // Link was previously lost, abort current connection

        tcp_abort(session->pcbConnect);

        streamFreeBuffers(session);

        session->linkLost = false;
    }

    session->ftype = wshdr_txt;
    session->pcbConnect = pcb;
    session->state = WsState_Connected;
    session->fragment_opcode = WsOpcode_Continuation;
    session->start.token = FRAME_NONE;
    memset(&session->header, 0, sizeof(frame_header_t));

    session->traffic_handler = WsConnectionHandler;
    session->pingCount = 0;

    WsStreamRxFlush();
    WsStreamTxFlush();

    tcp_accepted(pcb);

    session->timeout = 0;

    tcp_setprio(pcb, TCP_PRIO_MIN);
    tcp_recv(pcb, streamReceive);
    tcp_err(pcb, streamError);
    tcp_poll(pcb, streamPoll, 1000 / TCP_SLOW_INTERVAL);
    tcp_sent(pcb, streamSent);

    return ERR_OK;
}

void WsStreamClose (void)
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

    streamSession.state = WsState_Idle;
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

void WsStreamListen (uint16_t port)
{
//    ASSERT(port != 0);

    streamSession.state = WsState_Listen;
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
    tcp_accept(streamSession.pcbListen, WsStreamAccept);
}


/** Call tcp_write() in a loop trying smaller and smaller length
 *
 * @param pcb tcp_pcb to send
 * @param ptr Data to send
 * @param length Length of data to send (in/out: on return, contains the
 *        amount of data sent)
 * @param apiflags directly passed to tcp_write
 * @return the return value of tcp_write
 */
static err_t http_write(struct tcp_pcb *pcb, const void* ptr, u16_t *length, u8_t apiflags)
{
    u16_t len;
    err_t err;

    LWIP_ASSERT("length != NULL", length != NULL);

    len = *length;

    if (len == 0)
        return ERR_OK;

    do {
        err = tcp_write(pcb, ptr, len, apiflags);
        if (err == ERR_MEM) {
            if (tcp_sndbuf(pcb) == 0 || tcp_sndqueuelen(pcb) >= TCP_SND_QUEUELEN)
        /* no need to try smaller sizes */
                len = 1;
            else
                len /= 2;
        }
    } while (err == ERR_MEM && len > 1);

    *length = len;

    return err;
}

static void http_write_error (ws_sessiondata_t *session, const char *status)
{
    uint16_t len = strlen(status);
    http_write(session->pcbConnect, status, &len, 1);
    session->state = WsStateClosing;
}

//
// Process connection handshake
//
static void WsConnectionHandler (ws_sessiondata_t *session)
{
    bool hdr_ok;
    static uint32_t ptr = 0;

    if(session->http_request == NULL) {
        ptr = 0;
        if((session->http_request = malloc(session->hdrsize)) == NULL) {
            http_write_error(session, HTTP_500);
            return;
        }
    }

    uint8_t *payload = session->pbufCurrent ? session->pbufCurrent->payload : NULL;

    SYS_ARCH_DECL_PROTECT(lev);

    uint32_t hdrsize = session->hdrsize - 1;

    // 1. Process input
    while(true) {

        if(ptr == hdrsize) {
            session->hdrsize += 128;
            if((session->http_request = realloc(session->http_request, session->hdrsize)) == NULL) {
                http_write_error(session, HTTP_500);
                return;
            }
            hdrsize = session->hdrsize - 1;
        }

        // Get next pbuf chain to process
        if(session->pbufHead == NULL && session->rcvTail != session->rcvHead) {
            SYS_ARCH_PROTECT(lev);
            session->pbufCurrent = session->pbufHead = session->rcvTail->pbuf;
            session->rcvTail = session->rcvTail->next;
            session->bufferIndex = 0;
            SYS_ARCH_UNPROTECT(lev);
            payload = session->pbufCurrent ? session->pbufCurrent->payload : NULL;
        }

        if(payload == NULL)
            break; // No more data to be processed...

        // Add data to http request header
        session->http_request[ptr++] = payload[session->bufferIndex++];

        if(session->bufferIndex >= session->pbufCurrent->len) {
            session->pbufCurrent = session->pbufCurrent->next;
            session->bufferIndex = 0;
            payload = session->pbufCurrent ? session->pbufCurrent->payload : NULL;
        }

        // ACK current pbuf chain when all data has been processed
        if((session->pbufCurrent == NULL) && (session->bufferIndex == 0)) {
            tcp_recved(session->pcbConnect, session->pbufHead->tot_len);
            pbuf_free(session->pbufHead);
            session->pbufCurrent = session->pbufHead = NULL;
            session->bufferIndex = 0;
        }
    }

    session->http_request[ptr] = '\0';

    if((hdr_ok = strstr(session->http_request, "\r\n\r\n"))) {

#ifdef WSDEBUG
    DEBUG_PRINT(session->http_request);
#endif

        char *keyp, *key_hdr;

        if((key_hdr = stristr(session->http_request, WS_KEY))) {

            keyp = key_hdr + sizeof(WS_KEY) - 1;

            if((key_hdr = strstr(keyp, "\r\n"))) {

                char key[64];
                char rsp[150];

                *key_hdr = '\0';

                // Trim leading spaces from key
                while(*keyp == ' ')
                    keyp++;

                // Trim trailing spaces from key
                while(*--key_hdr == ' ')
                    key_hdr = '\0';

                // Copy base response header to response buffer
                char *response = memcpy(rsp /*session->http_request*/, WS_RSP, sizeof(WS_RSP) - 1);

                // Concatenate keys
                strcpy(key, keyp);
                strcat(key, WS_GUID);

                // Get SHA1 of keys
                BYTE sha1sum[SHA1_BLOCK_SIZE];
                SHA1_CTX ctx;
                sha1_init(&ctx);
                sha1_update(&ctx, (BYTE *)key, strlen(key));
                sha1_final(&ctx, sha1sum);

                // Base64 encode SHA1
                size_t olen = base64_encode((BYTE *)sha1sum, (BYTE *)&response[sizeof(WS_RSP) - 1], SHA1_BLOCK_SIZE, 0);

                // Upgrade...
                if (olen) {
                    response[olen + sizeof(WS_RSP) - 1] = '\0';
                    strcat(response, CRLF CRLF);
#ifdef WSDEBUG
    DEBUG_PRINT(response);
#endif
                    u16_t len = strlen(response);
                    http_write(session->pcbConnect, response, (u16_t *)&len, 1);
                    session->traffic_handler = WsStreamHandler;
                    session->lastSendTime = xTaskGetTickCount();
                    selectStream(StreamType_WebSocket);
                }
            }
        }

        if((key_hdr = stristr(session->http_request, WS_PROT))) {

            keyp = key_hdr + sizeof(WS_PROT) - 1;

            if((key_hdr = strstr(keyp, "\r\n"))) {

                *key_hdr = '\0';

                // Trim leading spaces from key
                while(*keyp == ' ')
                    keyp++;

                // Trim trailing spaces from key
                while(*--key_hdr == ' ')
                    key_hdr = '\0';

                // Switch to binary frames if protocol is: arduino
                if(!strcmp(keyp, "arduino"))
                    session->ftype = wshdr_bin;
            }
        }

        free(session->http_request);
        session->http_request = NULL;
        session->hdrsize = MAX_HTTP_HEADER_SIZE;
    }

    // Bad request?
    if(hdr_ok ? session->traffic_handler != WsStreamHandler : ptr > (MAX_HTTP_HEADER_SIZE * 2)) {
        http_write_error(session, HTTP_400);
        if(session->http_request) {
            free(session->http_request);
            session->http_request = NULL;
            session->hdrsize = MAX_HTTP_HEADER_SIZE;
        }
    }
}

//
// Process data for streaming
//

static bool WsCollectFrame (frame_header_t *header, uint8_t *payload, uint32_t len)
{
    if(header->payload_rem > len && header->payload_rem == header->payload_len) {
        if((header->frame = malloc(header->payload_len + header->idx)))
            memcpy(header->frame, &header->data, header->idx);
    }

    header->payload_rem -= len;

    if(header->frame)
        memcpy(header->frame + header->idx + header->payload_len - header->payload_rem - 1, payload, len);

    return header->frame != NULL;
}

static uint32_t WsParse (ws_sessiondata_t *session, uint8_t *payload, uint32_t len)
{
    bool frame_done = false;
    uint32_t plen = len;

    // Collect frame header
    while(!session->header.complete && plen) {

        session->header.data[session->header.idx++] = *payload++;

        if(session->header.idx == 2) {
            session->header.masked      = session->header.data[1] & 0x80; // always true from client
            session->header.payload_len = session->header.data[1] & 0x7F;
        }

        if(session->header.idx >= 6) {
            if((session->header.complete = (session->header.idx == (session->header.payload_len == 126 ? 8 : 6)))) {
                if(session->header.payload_len == 126) {
                    session->header.payload_len = (session->header.data[2] << 8) | session->header.data[3];
                    memcpy(&session->header.mask, &session->header.data[4], sizeof(uint32_t));
                } else
                    memcpy(&session->header.mask, &session->header.data[2], sizeof(uint32_t));
                session->header.payload_rem = session->header.payload_len;
            }
        }

        plen--;
    }

//    if(session->start.token != FRAME_NONE)
//        DEBUG_PRINT("\r\n!span\r\n");

    // Process frame
    if (session->header.complete && (plen || session->header.payload_rem == 0)) {

        ws_frame_start_t fs = (ws_frame_start_t)session->header.data[0];

        if (!fs.fin && (websocket_opcode_t)fs.opcode != WsOpcode_Continuation)
            session->fragment_opcode = (websocket_opcode_t)fs.opcode;

        if((websocket_opcode_t)fs.opcode == WsOpcode_Continuation)
            fs.opcode = session->fragment_opcode;

        switch ((websocket_opcode_t)fs.opcode) {

            case WsOpcode_Continuation:
                // Something went wrong, exit fragment handling mode
                session->fragment_opcode = WsOpcode_Continuation;
                break;

            case WsOpcode_Binary:
//              session->ftype = wshdr_bin; // Switch to binary responses if client talks binary to us
                //  No break
            case WsOpcode_Text:

                if (fs.fin)
                    session->fragment_opcode = WsOpcode_Continuation;

                if (session->header.payload_rem) {

                    uint8_t *mask = (uint8_t *)&session->header.mask;
                    uint_fast16_t payload_len = session->header.payload_rem > plen ? plen : session->header.payload_rem;

                    session->start.token = session->header.payload_rem > plen ? fs.token : FRAME_NONE;
/*
                    if(session->start.token != FRAME_NONE)
                        DEBUG_PRINT("\r\n!span!\r\n");
                    if(session->rxbuf.overflow)
                        DEBUG_PRINT("\r\n!overflow\r\n");
                    DEBUG_PRINT("\r\nPLEN:");
                    DEBUG_PRINT(uitoa(session->header.payload_rem));
                    DEBUG_PRINT(" ");
                    DEBUG_PRINT(uitoa(payload_len));
                    DEBUG_PRINT("\r\n");
*/
                    // Unmask and add data to input buffer
                    uint_fast16_t i = session->header.rx_index;
                    session->rxbuf.overflow = false;

                    while (payload_len--) {
                        if(!WsStreamRxInsert(*payload++ ^ mask[i % 4]))
                            break; // If overflow pend buffering rest of data until next polling
                        plen--;
                        i++;
                    }

                    session->header.rx_index = i;
                    frame_done = (session->header.payload_rem = session->header.payload_len - session->header.rx_index) == 0;
                }
                break;

            case WsOpcode_Close:
                if((frame_done = plen >= session->header.payload_rem)) {
                    plen -= session->header.payload_rem;
                    if(WsCollectFrame(&session->header, payload, session->header.payload_rem))
                        payload = session->header.frame;
                    tcp_write(session->pcbConnect, payload, session->header.payload_len, 1);
                    tcp_output(session->pcbConnect);
                    session->state = WsStateClosing;
                } else {
                    WsCollectFrame(&session->header, payload, plen);
                    plen = 0;
                }
                break;

            case WsOpcode_Ping:
                if((frame_done = plen >= session->header.payload_rem)) {
                    if(streamSession.state != WsStateClosing) {
                        plen -= session->header.payload_rem;
                        if(WsCollectFrame(&session->header, payload, session->header.payload_rem))
                            payload = session->header.frame;
                        fs.opcode = WsOpcode_Pong;
                        payload[0] = fs.token;
                        tcp_write(session->pcbConnect, payload, session->header.payload_len, 1);
                        tcp_output(session->pcbConnect);
                    }
                } else {
                    WsCollectFrame(&session->header, payload, plen);
                    plen = 0;
                }
                break;

            case WsOpcode_Pong:
                if((frame_done = plen >= session->header.payload_rem)) {
                    session->pingCount = 0;
                    plen -= session->header.payload_rem;
                } else {
                    session->header.payload_rem -= plen;
                    plen = 0;
                }
                break;

            default:
                // Unsupported/undefined opcode - ditch any payload(?)
                if((frame_done = plen >= session->header.payload_rem))
                    plen -= session->header.payload_rem;
                else {
                    session->header.payload_rem -= plen;
                    plen = 0;
                }
                break;
        }

        if(frame_done) {
            if(session->header.frame)
                free(session->header.frame);
            memset(&session->header, 0, sizeof(frame_header_t));
        }
    }

    return len - plen;
}

static void WsStreamHandler (ws_sessiondata_t *session)
{
    static uint8_t tempBuffer[PBUF_POOL_BUFSIZE];

    uint8_t *payload = session->pbufCurrent ? session->pbufCurrent->payload : NULL;

    SYS_ARCH_DECL_PROTECT(lev);

    // 1. Process input stream
    while(WsStreamRxFree()) {

        // Get next pbuf chain to process
        if(session->pbufHead == NULL && session->rcvTail != session->rcvHead) {
            SYS_ARCH_PROTECT(lev);
            session->pbufCurrent = session->pbufHead = session->rcvTail->pbuf;
            session->rcvTail = session->rcvTail->next;
            session->bufferIndex = 0;
            SYS_ARCH_UNPROTECT(lev);
            payload = session->pbufCurrent ? session->pbufCurrent->payload : NULL;
        }

        if(payload == NULL)
            break; // No more data to be processed...

        // Add data to input stream buffer
        session->bufferIndex += WsParse(session, &payload[session->bufferIndex], session->pbufCurrent->len - session->bufferIndex);

        if(session->rxbuf.overflow) // Failed to buffer all data, try again on next polling
            break;
/*
        DEBUG_PRINT("\r\nBLEN:");
        DEBUG_PRINT(uitoa(session->pbufCurrent->len));
        DEBUG_PRINT(" ");
        DEBUG_PRINT(uitoa(session->bufferIndex));
        DEBUG_PRINT("\r\n");
*/
        if(session->bufferIndex >= session->pbufCurrent->len) {
            session->pbufCurrent = session->pbufCurrent->next;
            session->bufferIndex = 0;
            payload = session->pbufCurrent ? session->pbufCurrent->payload : NULL;
        }

        // ACK current pbuf chain when all data has been processed
        if((session->pbufCurrent == NULL) && (session->bufferIndex == 0)) {
            tcp_recved(session->pcbConnect, session->pbufHead->tot_len);
            pbuf_free(session->pbufHead);
            session->pbufCurrent = session->pbufHead = NULL;
            session->bufferIndex = 0;
        }
    }

//    tcp_output(session->pcbConnect);

    uint_fast16_t TXCount;

    // 2. Process output stream
    if((TXCount = WsStreamTxCount()) && tcp_sndbuf(session->pcbConnect) > 4) {

        int16_t c;
        uint_fast16_t idx = 0;

        if(TXCount > tcp_sndbuf(session->pcbConnect) - 4)
            TXCount = tcp_sndbuf(session->pcbConnect) - 4;

        if(TXCount > sizeof(tempBuffer) - 4)
            TXCount = sizeof(tempBuffer) - 4;

        tempBuffer[idx++] = session->ftype.token;
        tempBuffer[idx++] = TXCount < 126 ? TXCount : 126;
        if(TXCount >= 126) {
            tempBuffer[idx++] = (TXCount >> 8) & 0xFF;
            tempBuffer[idx++] = TXCount & 0xFF;
        }

        while(TXCount) {
            if((c = (uint8_t)streamReadTXC()) == -1)
                break;
            tempBuffer[idx++] = (uint8_t)c;
            TXCount--;
        }

#ifdef WSDEBUG
    DEBUG_PRINT(uitoa(tempBuffer[1]));
    DEBUG_PRINT(" - ");
    DEBUG_PRINT(uitoa(idx));
    DEBUG_PRINT(" - ");
    DEBUG_PRINT(uitoa(plen));
    DEBUG_PRINT("\r\n");
#endif

        tcp_write(session->pcbConnect, tempBuffer, (u16_t)idx, 1);
        tcp_output(session->pcbConnect);

        session->lastSendTime = xTaskGetTickCount();
    }

    // Send ping every 3 seconds if no outgoing traffic.
    // Disconnect session after 3 failed pings (9 seconds).
    if(session->pingCount > 3)
        streamSession.state = WsStateClosing;
    else if(streamSession.state != WsStateClosing && (xTaskGetTickCount() - session->lastSendTime) > (3 * configTICK_RATE_HZ)) {
        if(tcp_sndbuf(session->pcbConnect) > 4) {
            tempBuffer[0] = wshdr_ping.token;
            tempBuffer[1] = 2;
            strcpy((char *)&tempBuffer[2], "Hi");
            tcp_write(session->pcbConnect, tempBuffer, 4, 1);
            tcp_output(session->pcbConnect);
            session->lastSendTime = xTaskGetTickCount();
            session->pingCount++;
        }
    }
}

//
// Process data for streaming
//
void WsStreamPoll (void)
{
    if(streamSession.state == WsState_Connected)
        streamSession.traffic_handler(&streamSession);
    else if(streamSession.state == WsStateClosing)
        closeSocket(&streamSession, streamSession.pcbConnect);
}

#endif
