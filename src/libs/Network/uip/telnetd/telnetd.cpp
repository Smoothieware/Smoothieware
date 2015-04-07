/*
 * Copyright (c) 2003, Adam Dunkels.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This file is part of the uIP TCP/IP stack
 *
 * $Id: telnetd.c,v 1.2 2006/06/07 09:43:54 adam Exp $
 *
 */

#include "uip.h"
#include "telnetd.h"
#include "shell.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define ISO_nl       0x0a
#define ISO_cr       0x0d

#define STATE_NORMAL 0
#define STATE_IAC    1
#define STATE_WILL   2
#define STATE_WONT   3
#define STATE_DO     4
#define STATE_DONT   5
#define STATE_CLOSE  6

#define TELNET_IAC   255
#define TELNET_WILL  251
#define TELNET_WONT  252
#define TELNET_DO    253
#define TELNET_DONT  254

#define TELNET_LINEMODE 0x22
#define TELNET_GA       0x03
#define TELNET_X_PROMPT 0x55

//#define DEBUG_PRINTF(...)
#define DEBUG_PRINTF printf

static char *alloc_line(int size)
{
    return (char *)malloc(size);
}

static void dealloc_line(char *line)
{
    free(line);
}

void Telnetd::close()
{
    state = STATE_CLOSE;
}

int Telnetd::sendline(char *line)
{
    int i;
    for (i = 0; i < TELNETD_CONF_NUMLINES; ++i) {
        if (lines[i] == NULL) {
            lines[i] = line;
            return i;
        }
    }
    if (i == TELNETD_CONF_NUMLINES) {
        dealloc_line(line);
    }
    return TELNETD_CONF_NUMLINES;
}

void Telnetd::output_prompt(const char *str)
{
    if(prompt) output(str);
}

int Telnetd::output(const char *str)
{
    if(state == STATE_CLOSE) return -1;

    unsigned chunk = 256; // small chunk size so we don't allocate huge blocks, and must be less than mss
    unsigned len = strlen(str);
    char *line;
    if (len < chunk) {
        // can be sent in one tcp buffer
        line = alloc_line(len + 1);
        if (line != NULL) {
            strcpy(line, str);
            return sendline(line);
        }else{
            // out of memory treat like full
            return TELNETD_CONF_NUMLINES;
        }
    } else {
        // need to split line over multiple send lines
        int size = chunk; // size to copy
        int off = 0;
        int n= 0;
        while (len >= chunk) {
            line = alloc_line(chunk + 1);
            if (line != NULL) {
                memcpy(line, str + off, size);
                line[size] = 0;
                n= sendline(line);
                len -= size;
                off += size;
            }else{
                // out of memory treat like full
                return TELNETD_CONF_NUMLINES;
            }
        }
        if (len > 0) {
            // send rest
            line = alloc_line(len + 1);
            if (line != NULL) {
                strcpy(line, str + off);
                n= sendline(line);
            }else{
                // out of memory treat like full
                return TELNETD_CONF_NUMLINES;
            }
        }
        return n;
    }
}

// check if we can queue or if queue is full
int Telnetd::can_output()
{
    if(state == STATE_CLOSE) return -1;

    int i;
    int cnt = 0;
    for (i = 0; i < TELNETD_CONF_NUMLINES; ++i) {
        if (lines[i] == NULL) cnt++;
    }
    return cnt < 4 ? 0 : 1;
}

void Telnetd::acked(void)
{
    while (numsent > 0) {
        dealloc_line(lines[0]);
        for (int i = 1; i < TELNETD_CONF_NUMLINES; ++i) {
            lines[i - 1] = lines[i];
        }
        lines[TELNETD_CONF_NUMLINES - 1] = NULL;
        --numsent;
    }
}

void Telnetd::senddata(void)
{
    // NOTE this sends as many lines as it can fit in one tcp frame
    // we need to keep the lines under the size of the tcp frame
    char *bufptr, *lineptr;
    int buflen, linelen;

    bufptr = (char *)uip_appdata;
    buflen = 0;
    for (numsent = 0; numsent < TELNETD_CONF_NUMLINES && lines[numsent] != NULL ; ++numsent) {
        lineptr = lines[numsent];
        linelen = strlen(lineptr);
        if (buflen + linelen < uip_mss()) {
            memcpy(bufptr, lineptr, linelen);
            bufptr += linelen;
            buflen += linelen;
        } else {
            break;
        }
    }
    uip_send(uip_appdata, buflen);
}

void Telnetd::get_char(u8_t c)
{
    if (c == ISO_cr) {
        return;
    }

    buf[(int)bufptr] = c;
    if (buf[(int)bufptr] == ISO_nl || bufptr == sizeof(buf) - 1) {
        if (bufptr > 0) {
            buf[(int)bufptr] = 0;
        }
        shell->input(buf);
        bufptr = 0;

    } else {
        ++bufptr;
    }
}

// static void sendopt(u8_t option, u8_t value)
// {
//     char *line;
//     line = alloc_line(4);
//     if (line != NULL) {
//         line[0] = TELNET_IAC;
//         line[1] = option;
//         line[2] = value;
//         line[3] = 0;
//         sendline(line);
//     }
// }

void Telnetd::newdata(void)
{
    u16_t len;
    u8_t c;
    char *dataptr;

    len = uip_datalen();
    dataptr = (char *)uip_appdata;

    while (len > 0 && bufptr < sizeof(buf)) {
        c = *dataptr;
        ++dataptr;
        --len;
        switch (state) {
            case STATE_IAC:
                if (c == TELNET_IAC) {
                    get_char(c);
                    state = STATE_NORMAL;
                } else {
                    switch (c) {
                        case TELNET_WILL:
                            state = STATE_WILL;
                            break;
                        case TELNET_WONT:
                            state = STATE_WONT;
                            break;
                        case TELNET_DO:
                            state = STATE_DO;
                            break;
                        case TELNET_DONT:
                            state = STATE_DONT;
                            break;
                        default:
                            state = STATE_NORMAL;
                            break;
                    }
                }
                break;
            case STATE_WILL:
                if (c == TELNET_LINEMODE) {
                    //sendopt(TELNET_DO, c);
                }
                state = STATE_NORMAL;
                break;

            case STATE_WONT:
                /* Reply with a DONT */
                //sendopt(TELNET_DONT, c);
                state = STATE_NORMAL;
                break;
            case STATE_DO:
               if (c == TELNET_X_PROMPT) {
                    prompt= true;
                }else if (c == TELNET_GA) {
                    // enable prompt if telnet client running
                    prompt= true;
                    shell->setConsole(); // tell shell we are a console, as this is sent be telnet clients
                }else{
                     /* Reply with a WONT */
                    //sendopt(TELNET_WONT, c);
                }
                state = STATE_NORMAL;
                break;
            case STATE_DONT:
                if (c == TELNET_X_PROMPT) {
                    prompt= false;
                }else{
                    /* Reply with a WONT */
                    //sendopt(TELNET_WONT, c);
                }
                state = STATE_NORMAL;
                break;
            case STATE_NORMAL:
                if (c == TELNET_IAC) {
                    state = STATE_IAC;
                } else {
                    get_char(c);
                }
                break;
        }
    }

    // if the command queue is getting too big we stop TCP
    if(shell->queue_size() > 20) {
        DEBUG_PRINTF("Telnet: stopped: %d\n", shell->queue_size());
        uip_stop();
    }
}

void Telnetd::poll()
{
    if(first_time) {
        first_time= false;
        shell->start();
        senddata();
    }
}

Telnetd::Telnetd()
{
    DEBUG_PRINTF("Telnetd: ctor %p\n", this);
    for (int i = 0; i < TELNETD_CONF_NUMLINES; ++i) {
        lines[i] = NULL;
    }

    first_time= true;
    bufptr = 0;
    state = STATE_NORMAL;
    prompt= false;
    shell= new Shell(this);
}

Telnetd::~Telnetd()
{
    DEBUG_PRINTF("Telnetd: dtor %p\n", this);
    for (int i = 0; i < TELNETD_CONF_NUMLINES; ++i) {
        if (lines[i] != NULL) dealloc_line(lines[i]);
    }
    delete shell;
}

// static
void Telnetd::appcall(void)
{
    Telnetd *instance= reinterpret_cast<Telnetd *>(uip_conn->appstate);

    if (uip_connected()) {
        // create a new telnet class instance
        instance= new Telnetd;
        DEBUG_PRINTF("Telnetd new instance: %p\n", instance);
        uip_conn->appstate= instance; // and store it in the appstate of the connection
        instance->rport= uip_conn->rport;
    }

    if (uip_closed() || uip_aborted() || uip_timedout()) {
        DEBUG_PRINTF("Telnetd: closed: %p\n", instance);
        if(instance != NULL) {
            delete instance;
            uip_conn->appstate= NULL;
        }
        return;
    }

    // sanity check
    if(instance == NULL || instance->rport != uip_conn->rport) {
        DEBUG_PRINTF("Telnetd: ERROR Null instance or rport is wrong: %p - %u, %d\n", instance, HTONS(uip_conn->rport), uip_flags);
        uip_abort();
        return;
    }

    if (instance->state == STATE_CLOSE) {
        uip_close();
    }


    if (uip_acked()) {
        instance->acked();
    }

    if (uip_newdata()) {
        instance->newdata();
    }

    if (uip_rexmit() || uip_newdata() || uip_acked() || uip_connected() || uip_poll()) {
        instance->senddata();
    }

    if(uip_poll() && uip_stopped(uip_conn) && instance->shell->queue_size() < 5) {
        DEBUG_PRINTF("restarted %d - %p\n", instance->shell->queue_size(), instance);
        uip_restart();
    }

    if(uip_poll()) {
        instance->poll();
    }
}

// static
void Telnetd::init(void)
{
    uip_listen(HTONS(23));
}
