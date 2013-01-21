#ifndef _IP_H
#define _IP_H

#include <cstdint>

#include "net_util.h"

#include "TCP.h"
#include "UDP.h"
#include "ICMP.h"

class NetworkInterface;

typedef enum {
    IP_PROTO_ICMP       =  1,
    IP_PROTO_IPV4       =  4,
    IP_PROTO_TCP        =  6,
    IP_PROTO_UDP        = 17,
    IP_PROTO_IPV6       = 41,
    IP_PROTO_IPV6_ROUTE = 43,
    IP_PROTO_IPV6_FRAG  = 44,
    IP_PROTO_IPV6_ICMP  = 58,
    IP_PROTO_IPV6_NONXT = 59,
    IP_PROTO_IPV6_OPTS  = 60,
} IP_PROTOCOLS;

typedef struct __attribute__ ((packed)) {
    uint8_t version:4;
    uint8_t ihl:4;

    uint8_t dscp:6;
    uint8_t ecn:2;

    uint16_t total_length;
    uint16_t identification;

    uint16_t flags:3;
    uint16_t fragment_offset:13;

    uint8_t ttl;
    uint8_t protocol;
    uint16_t header_checksum;
    uint32_t srcip;
    uint32_t dstip;
    uint8_t payload[];
} ip_frame;

class IP : public Encapsulator, public Period_receiver
{
public:
    IP(Encapsulator*);

    int   receive(   NetworkInterface*, NET_PACKET, int);

    int   construct( NetworkInterface*, NET_PACKET, int);
    int   construct( NetworkInterface*, NET_PACKET, int, uint32_t, IP_PROTOCOLS);

    void  set_ttl(NET_PACKET, int);

    NET_PACKET get_new_packet_buffer(NetworkInterface*);
    NET_PAYLOAD get_payload_buffer(NET_PACKET);
    void  set_payload_length(NET_PACKET, int);

    int   get_length(NET_PACKET);

    int   periodical(int, NetworkInterface*, NET_PACKET, int);

    void  set_dest_ip(NET_PACKET, IP_ADDR);

protected:
    Encapsulator* parent;

    ICMP icmp;
    TCP tcp;
    UDP udp;
};

#endif /* _IP_H */
