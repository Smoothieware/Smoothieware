/*
 * Copyright (c) 2005, Swedish Institute of Computer Science
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the uIP TCP/IP stack
 *
 * @(#)$Id: dhcpc.c,v 1.2 2006/06/11 21:46:37 adam Exp $
 */

#include <stdio.h>
#include <string.h>

#include "uip.h"
#include "dhcpc.h"
#include "timer.h"
#include "pt.h"

#if UIP_CONF_UDP

#define STATE_INITIAL         0
#define STATE_SENDING         1
#define STATE_OFFER_RECEIVED  2
#define STATE_CONFIG_RECEIVED 3

#define ntohl(a) ((((a) >> 24) & 0x000000FF) | (((a) >> 8) & 0x0000FF00) | (((a) << 8) & 0x00FF0000) | (((a) << 24) & 0xFF000000))
static struct dhcpc_state s __attribute__ ((section ("AHBSRAM1")));
//#define UIP_CONF_DHCP_LIGHT

struct dhcp_msg {
    u8_t op, htype, hlen, hops;
    u8_t xid[4];
    u16_t secs, flags;
    u8_t ciaddr[4];
    u8_t yiaddr[4];
    u8_t siaddr[4];
    u8_t giaddr[4];
    u8_t chaddr[16];
#ifndef UIP_CONF_DHCP_LIGHT
    u8_t sname[64];
    u8_t file[128];
#endif
    u8_t options[312];
};

#define BOOTP_BROADCAST 0x8000

#define DHCP_REQUEST        1
#define DHCP_REPLY          2
#define DHCP_HTYPE_ETHERNET 1
#define DHCP_HLEN_ETHERNET  6
#define DHCP_MSG_LEN      236

#define DHCPC_SERVER_PORT  67
#define DHCPC_CLIENT_PORT  68

#define DHCPDISCOVER  1
#define DHCPOFFER     2
#define DHCPREQUEST   3
#define DHCPDECLINE   4
#define DHCPACK       5
#define DHCPNAK       6
#define DHCPRELEASE   7

#define DHCP_OPTION_SUBNET_MASK   1
#define DHCP_OPTION_ROUTER        3
#define DHCP_OPTION_DNS_SERVER    6
#define DHCP_OPTION_HOSTNAME     12
#define DHCP_OPTION_REQ_IPADDR   50
#define DHCP_OPTION_LEASE_TIME   51
#define DHCP_OPTION_MSG_TYPE     53
#define DHCP_OPTION_SERVER_ID    54
#define DHCP_OPTION_REQ_LIST     55
#define DHCP_OPTION_END         255

static uint32_t xid= 0x00112233;

static const u8_t magic_cookie[4] = {99, 130, 83, 99};
/*---------------------------------------------------------------------------*/
static u8_t *
add_msg_type(u8_t *optptr, u8_t type)
{
    *optptr++ = DHCP_OPTION_MSG_TYPE;
    *optptr++ = 1;
    *optptr++ = type;
    return optptr;
}
/*---------------------------------------------------------------------------*/
static u8_t *
add_server_id(u8_t *optptr)
{
    *optptr++ = DHCP_OPTION_SERVER_ID;
    *optptr++ = 4;
    memcpy(optptr, &s.serverid, 4);
    return optptr + 4;
}
/*---------------------------------------------------------------------------*/
static u8_t *
add_req_ipaddr(u8_t *optptr)
{
    *optptr++ = DHCP_OPTION_REQ_IPADDR;
    *optptr++ = 4;
    memcpy(optptr, &s.ipaddr, 4);
    return optptr + 4;
}
/*---------------------------------------------------------------------------*/
static u8_t *
add_hostname(u8_t *optptr)
{
    if (s.hostname == NULL) {
        return optptr;
    }
    const u8_t l = strlen(s.hostname);
    *optptr++ = DHCP_OPTION_HOSTNAME;
    *optptr++ = l;
    memcpy(optptr, s.hostname, l);
    return optptr + l;
}
/*---------------------------------------------------------------------------*/
static u8_t *
add_req_options(u8_t *optptr)
{
    *optptr++ = DHCP_OPTION_REQ_LIST;
    *optptr++ = s.hostname == NULL ? 3 : 4;
    *optptr++ = DHCP_OPTION_SUBNET_MASK;
    *optptr++ = DHCP_OPTION_ROUTER;
    *optptr++ = DHCP_OPTION_DNS_SERVER;
    if (s.hostname != NULL) {
        *optptr++ = DHCP_OPTION_HOSTNAME;
    }
    return optptr;
}
/*---------------------------------------------------------------------------*/
static u8_t *
add_end(u8_t *optptr)
{
    *optptr++ = DHCP_OPTION_END;
    return optptr;
}
/*---------------------------------------------------------------------------*/
static void
create_msg(register struct dhcp_msg *m, int rea)
{
    m->op = DHCP_REQUEST;
    m->htype = DHCP_HTYPE_ETHERNET;
    m->hlen = s.mac_len;
    m->hops = 0;
    memcpy(m->xid, &xid, sizeof(m->xid));
    m->secs = 0;
    m->flags = HTONS(BOOTP_BROADCAST); /*  Broadcast bit. */
    /*  uip_ipaddr_copy(m->ciaddr, uip_hostaddr);*/
    if(rea == 0 )  memcpy(m->ciaddr, uip_hostaddr, sizeof(m->ciaddr));
    else memset(m->ciaddr, 0, sizeof(m->ciaddr));
    memset(m->yiaddr, 0, sizeof(m->yiaddr));
    memset(m->siaddr, 0, sizeof(m->siaddr));
    memset(m->giaddr, 0, sizeof(m->giaddr));
    memcpy(m->chaddr, s.mac_addr, s.mac_len);
    memset(&m->chaddr[s.mac_len], 0, sizeof(m->chaddr) - s.mac_len);
#ifndef UIP_CONF_DHCP_LIGHT
    memset(m->sname, 0, sizeof(m->sname));
    memset(m->file, 0, sizeof(m->file));
#endif

    memcpy(m->options, magic_cookie, sizeof(magic_cookie));
}
/*---------------------------------------------------------------------------*/
static void
send_discover(void)
{
    u8_t *end;
    struct dhcp_msg *m = (struct dhcp_msg *)uip_appdata;

    create_msg(m, 0);

    end = add_msg_type(&m->options[4], DHCPDISCOVER);
    end = add_req_options(end);
    end = add_end(end);

    uip_send(uip_appdata, end - (u8_t *)uip_appdata);
}
/*---------------------------------------------------------------------------*/
static void
send_request(int rea)
{
    u8_t *end;
    struct dhcp_msg *m = (struct dhcp_msg *)uip_appdata;

    create_msg(m, rea);

    end = add_msg_type(&m->options[4], DHCPREQUEST);
    end = add_server_id(end);
    end = add_req_ipaddr(end);
    end = add_hostname(end);
    end = add_end(end);

    uip_send(uip_appdata, end - (u8_t *)uip_appdata);
}
/*---------------------------------------------------------------------------*/
static u8_t
parse_options(u8_t *optptr, int len)
{
    u8_t *end = optptr + len;
    u8_t type = 0;

    while (optptr < end) {
        switch (*optptr) {
            case DHCP_OPTION_SUBNET_MASK:
                memcpy(&s.netmask, optptr + 2, 4);
                break;
            case DHCP_OPTION_ROUTER:
                memcpy(&s.default_router, optptr + 2, 4);
                break;
            case DHCP_OPTION_DNS_SERVER:
                memcpy(&s.dnsaddr, optptr + 2, 4);
                break;
            case DHCP_OPTION_MSG_TYPE:
                type = *(optptr + 2);
                break;
            case DHCP_OPTION_SERVER_ID:
                memcpy(s.serverid, optptr + 2, 4);
                break;
            case DHCP_OPTION_LEASE_TIME:
                memcpy(&s.lease_time, optptr + 2, 4);
                break;
            case DHCP_OPTION_END:
                return type;
        }

        optptr += optptr[1] + 2;
    }
    return type;
}
/*---------------------------------------------------------------------------*/
u8_t
parse_msg(void)
{
    struct dhcp_msg *m = (struct dhcp_msg *)uip_appdata;

    if (m->op == DHCP_REPLY &&
        memcmp(m->xid, &xid, sizeof(xid)) == 0 &&
        memcmp(m->chaddr, s.mac_addr, s.mac_len) == 0) {
        memcpy(&s.ipaddr, m->yiaddr, 4);
        return parse_options(&m->options[4], uip_datalen());
    }
    return 0;
}
/*---------------------------------------------------------------------------*/
static
PT_THREAD(handle_dhcp(void))
{
    PT_BEGIN(&s.pt);

    /* try_again:*/
    s.state = STATE_SENDING;
    s.ticks = CLOCK_SECOND;
    xid++;

    send_discover();
    do {
        timer_set(&s.timer, s.ticks);
        PT_WAIT_UNTIL(&s.pt, uip_newdata() || timer_expired(&s.timer));
        // if we timed out then increase time out and send discover again
        if (timer_expired(&s.timer)) {
            if (s.ticks < CLOCK_SECOND * 60) {
                s.ticks *= 2;
            }
            send_discover();
        }else{
            // we may have gotten some other UDP packet in which case just wait some more for the right packet
            if (uip_newdata() && parse_msg() == DHCPOFFER) {
                s.state = STATE_OFFER_RECEIVED;
                break;
            }
        }
        PT_YIELD(&s.pt);

    } while (s.state != STATE_OFFER_RECEIVED);

    s.ticks = CLOCK_SECOND;
    xid++;

    send_request(0);
    do {
        timer_set(&s.timer, s.ticks);
        PT_WAIT_UNTIL(&s.pt, uip_newdata() || timer_expired(&s.timer));

        if (timer_expired(&s.timer)) {
            if (s.ticks <= CLOCK_SECOND * 10) {
                s.ticks += CLOCK_SECOND;
                send_request(0); // resend only on timeout
            } else {
                PT_RESTART(&s.pt);
            }
        }else{
            if (uip_newdata() && parse_msg() == DHCPACK) {
                s.state = STATE_CONFIG_RECEIVED;
                break;
            }
        }
        PT_YIELD(&s.pt);

    } while (s.state != STATE_CONFIG_RECEIVED);

    dhcpc_configured(&s);

    // now we wait for close to expiration and renew the lease
    do {
        // we should reacquire expired leases here.
        timer_set(&s.timer, (ntohl(s.lease_time) * 0.5)*CLOCK_SECOND); // half of lease expire time
        PT_WAIT_UNTIL(&s.pt, timer_expired(&s.timer));

        uip_log("reaquire dhcp lease");

        // spec says send request direct to server that gave it to us, but seems to be unecessary
        //uip_ipaddr_copy(&s.conn->ripaddr, s.serverid);

        s.ticks = CLOCK_SECOND;
        xid++;
        send_request(0);
        do {
            timer_set(&s.timer, s.ticks);
            PT_WAIT_UNTIL(&s.pt, uip_newdata() || timer_expired(&s.timer));

            if (timer_expired(&s.timer)) {
                if (s.ticks <= CLOCK_SECOND * 10) {
                    s.ticks += CLOCK_SECOND;
                    send_request(0); // resend only on timeout
                } else {
                    // give up
                    // TODO probably need to deal with upstream apps and stop them then reinit them
                    PT_RESTART(&s.pt);
                }
            }else{
                if (parse_msg() == DHCPACK) {
                    uip_log("dhcp lease renewed");
                    break;
                }
            }
            PT_YIELD(&s.pt);
        }while(1);

    }while(1);

    PT_END(&s.pt);
}
/*---------------------------------------------------------------------------*/
void
dhcpc_init(const void *mac_addr, int mac_len, char *hostname)
{
    uip_ipaddr_t addr;

    s.mac_addr = mac_addr;
    s.mac_len  = mac_len;
    s.hostname = hostname;

    s.state = STATE_INITIAL;
    uip_ipaddr(addr, 255, 255, 255, 255);
    s.conn = uip_udp_new(&addr, HTONS(DHCPC_SERVER_PORT));
    if (s.conn != NULL) {
        uip_udp_bind(s.conn, HTONS(DHCPC_CLIENT_PORT));
    }
    PT_INIT(&s.pt);
}
/*---------------------------------------------------------------------------*/
void
dhcpc_appcall(void)
{
    handle_dhcp();
}
/*---------------------------------------------------------------------------*/
void
dhcpc_request(void)
{
    u16_t ipaddr[2];

    if (s.state == STATE_INITIAL) {
        uip_ipaddr(ipaddr, 0, 0, 0, 0);
        uip_sethostaddr(ipaddr);
        /*    handle_dhcp(PROCESS_EVENT_NONE, NULL);*/
    }
}
/*---------------------------------------------------------------------------*/

#endif
