#ifndef _DHCP_H
#define _DHCP_H

#include <cstdint>

#include "net_util.h"

typedef struct {
    uint8_t  op;
    uint8_t  htype;
    uint8_t  hlen;
    uint8_t  hops;
    uint32_t xid;
    uint16_t secs;
    uint16_t flags;
    uint32_t ciaddr;
    uint32_t yiaddr;
    uint32_t siaddr;
    uint32_t giaddr;
    union
    {
//         struct
//         {
//             uint8_t chaddr[16];
//             uint8_t sname[64];
//             uint8_t file[128];
//         };
        uint8_t options[];
    };
} DHCP_Packet;

#define OP_BOOTREQUEST 1
#define OP_BOOTREPLY   2

#define FLAG_BROADCAST 1

#define HLEN_ETHERNET  6

#define DHCP_MAGIC_COOKIE 0x63825363

#define OPTION_SUBNETMASK    1
#define OPTION_GATEWAY       3
#define OPTION_TIME          4
#define OPTION_DNSSERVER     6
#define OPTION_HOSTNAME     12
#define OPTION_DOMAINNAME   15
#define OPTION_NTP          42
#define OPTION_LEASETIME    51
#define OPTION_DHCP_MSGTYPE 53

#define MSGTYPE_DHCPDISCOVER 1
#define MSGTYPE_DHCPOFFER    2
#define MSGTYPE_DHCPREQUEST  3
#define MSGTYPE_DHCPDECLINE  4
#define MSGTYPE_DHCPACK      5
#define MSGTYPE_DHCPNAK      6
#define MSGTYPE_DHCPRELEASE  7
#define MSGTYPE_DHCPINFORM   8

class DHCP : public Encapsulated
{
    DHCP(Encapsulator*);

    int receive(NetworkInterface*, void*, int);
    int construct(NetworkInterface*, void*, int);

    int set_option(NetworkInterface* ni, DHCP_Packet* packet, int option, int optionsize, uint8_t* optiondata);

    Encapsulator* parent;
};

#endif /* _DHCP_H */
