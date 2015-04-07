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
* This file is part of the uIP TCP/IP stack.
*
* $Id: shell.c,v 1.1 2006/06/07 09:43:54 adam Exp $
*
*/

#include "Kernel.h"
#include "stdlib.h"
#include "shell.h"
#include "uip.h"
#include <string.h>
#include "checksumm.h"
#include "utils.h"
#include "stdio.h"
#include "stdlib.h"
#include "telnetd.h"
#include "CallbackStream.h"
#include "StreamOutputPool.h"
#include "CommandQueue.h"

//#define DEBUG_PRINTF(...)
#define DEBUG_PRINTF printf

struct ptentry {
    const char *command;
    void (* pfunc)(char *str, Shell *sh);
};

#define SHELL_PROMPT "> "

/*---------------------------------------------------------------------------*/
bool Shell::parse(register char *str, const struct ptentry *t)
{
    const struct ptentry *p;
    for (p = t; p->command != 0; ++p) {
        if (strncasecmp(str, p->command, strlen(p->command)) == 0) {
            break;
        }
    }

    p->pfunc(str, this);

    return p->command != 0;
}
/*---------------------------------------------------------------------------*/
static void help(char *str, Shell *sh)
{
    sh->output("Available commands: All others are passed on\n");
    sh->output("netstat     - show network info\n");
    sh->output("?           - show network help\n");
    sh->output("help        - show command help\n");
    sh->output("exit, quit  - exit shell\n");
}

/*---------------------------------------------------------------------------*/
static const char *states[] = {
    "CLOSED",
    "SYN_RCVD",
    "SYN_SENT",
    "ESTABLISHED",
    "FIN_WAIT_1",
    "FIN_WAIT_2",
    "CLOSING",
    "TIME_WAIT",
    "LAST_ACK",
    "NONE",
    "RUNNING",
    "CALLED"
};
static void connections(char *str, Shell *sh)
{
    char istr[128];
    struct uip_conn *connr;
    snprintf(istr, sizeof(istr), "Initial MSS: %d, MSS: %d\n", uip_initialmss(), uip_mss());
    sh->output(istr);
    sh->output("Current connections: \n");

    for (connr = &uip_conns[0]; connr <= &uip_conns[UIP_CONNS - 1]; ++connr) {
        if(connr->tcpstateflags != UIP_CLOSED) {
            snprintf(istr, sizeof(istr), "%d, %u.%u.%u.%u:%u, %s, %u, %u, %c %c\n",
                     HTONS(connr->lport),
                     uip_ipaddr1(connr->ripaddr), uip_ipaddr2(connr->ripaddr),  uip_ipaddr3(connr->ripaddr), uip_ipaddr4(connr->ripaddr),
                     HTONS(connr->rport),
                     states[connr->tcpstateflags & UIP_TS_MASK],
                     connr->nrtx,
                     connr->timer,
                     (uip_outstanding(connr)) ? '*' : ' ',
                     (uip_stopped(connr)) ? '!' : ' ');

            sh->output(istr);
        }
    }
}

static void quit(char *str, Shell *sh)
{
    sh->close();
}

//#include "clock.h"
static void test(char *str, Shell *sh)
{
    printf("In Test\n");

    // struct timer t;
    // u16_t ticks=  CLOCK_SECOND*5;
    // timer_set(&t, ticks);
    // printf("Wait....\n");
    // while(!timer_expired(&t)) {

    // }
    // printf("Done\n");
    /*
        const char *fn= "/sd/test6.txt";
        uint16_t *buf= (uint16_t *)malloc(200*2);
        int cnt= 0;
        FILE *fp;
        for(int i=0;i<10;i++) {
            fp= fopen(fn, i == 0 ? "w" : "a");
            if(fp == NULL) {
                printf("failed to open file\n");
                return;
            }
            for (int x = 0; x < 200; ++x) {
                buf[x]= x+cnt;
            }
            cnt+=200;
            int n= fwrite(buf, 2, 200, fp);
            printf("wrote %d, %d\n", i, n);
            fclose(fp);
        }

        fp= fopen(fn, "r");
        if(fp == NULL) {
            printf("failed to open file for read\n");
            return;
        }
        printf("Opened file %s for read\n", fn);
        do {
            int n= fread(buf, 2, 200, fp);
            if(n <= 0) break;
            for(int x=0;x<n;x++) {
                printf("%04X, ", buf[x]);
            }
        }while(1);
        fclose(fp);
        free(buf);
        */
}

/*---------------------------------------------------------------------------*/

static void unknown(char *str, Shell *sh)
{
    // its some other command, so queue it for mainloop to find
    if (strlen(str) > 0) {
        CommandQueue::getInstance()->add(str, sh->getStream());
    }
}
/*---------------------------------------------------------------------------*/
static const struct ptentry parsetab[] = {
    {"netstat", connections},
    {"exit", quit},
    {"quit", quit},
    {"test", test},
    {"?", help},

    /* Default action */
    {0, unknown}
};
/*---------------------------------------------------------------------------*/
// this callback gets the results of a command, line by line
// NULL means command completed
// static
int Shell::command_result(const char *str, void *p)
{
    // FIXME problem when shell is deleted and this gets called from slow command
    // need a way to know this shell was closed or deleted
    Shell *sh = (Shell *)p;
    if (str == NULL) {
        // indicates command is complete
        // only prompt when command is completed
        sh->telnet->output_prompt(SHELL_PROMPT);
        return 0;

    } else {
        if (sh->telnet->can_output()) {
            if (sh->telnet->output(str) == -1) return -1; // connection was closed
            return 1;
        }
        // we are stalled
        return 0;
    }
}

/*---------------------------------------------------------------------------*/
void Shell::start()
{
    telnet->output("Smoothie command shell\r\n> ");
}

int Shell::queue_size()
{
    return CommandQueue::getInstance()->size();
}
/*---------------------------------------------------------------------------*/
void Shell::input(char *cmd)
{
    if (parse(cmd, parsetab)) {
        telnet->output_prompt(SHELL_PROMPT);
    }
}
/*---------------------------------------------------------------------------*/

int Shell::output(const char *str)
{
    return telnet->output(str);
}

void Shell::close()
{
    telnet->close();
}

void Shell::setConsole()
{
    // add it to the kernels output stream if we are a console
    // TODO do we do this for all connections? so pronterface will get file done when playing from M24?
    // then we need to turn it off for the streaming app
    DEBUG_PRINTF("Shell: Adding stream to kernel streams\n");
    THEKERNEL->streams->append_stream(pstream);
    isConsole= true;
}

Shell::Shell(Telnetd *telnet)
{
    DEBUG_PRINTF("Shell: ctor %p - %p\n", this, telnet);
    this->telnet= telnet;
    // create a callback StreamOutput for this connection
    pstream = new CallbackStream(command_result, this);
    isConsole= false;
}

Shell::~Shell()
{
    if(isConsole) {
        DEBUG_PRINTF("Shell: Removing stream from kernel streams\n");
        THEKERNEL->streams->remove_stream(pstream);
    }
    // we cannot delete this stream until it is no longer in any command queue entries
    // so mark it as closed, and allow it to delete itself when it is no longer being used
    static_cast<CallbackStream*>(pstream)->mark_closed(); // mark the stream as closed so we do not get any callbacks
    DEBUG_PRINTF("Shell: dtor %p\n", this);
}
