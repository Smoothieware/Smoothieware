#ifndef _IP_H
#define _IP_H

#include <cstdint>

#include "net_util.h"

#include "TCP.h"
#include "UDP.h"
#include "ICMP.h"

class network_interface;

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

class IP : public Encapsulator
{
public:
    IP();

    int   receive(   network_interface*, void*, int);

    int   construct( network_interface*, void*, int);
    int   construct( network_interface*, void*, int, uint32_t, IP_PROTOCOLS);

    void  set_ttl(void*, int);

    void* get_payload_buffer(void*);
    void  set_payload_length(void*, int);

    int   get_length(void*);

protected:
    ICMP icmp;
    TCP tcp;
    UDP udp;
};

#endif /* _IP_H */
