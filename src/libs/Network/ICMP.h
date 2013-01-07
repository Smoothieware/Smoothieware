#ifndef _ICMP_H
#define _ICMP_H

#include "net_util.h"

typedef enum {
    ICMP_TYPE_ECHO_REPLY        =  0,
    ICMP_TYPE_DEST_UNREACHABLE  =  3,
    ICMP_TYPE_SOURCE_QUENCH     =  4,
    ICMP_TYPE_REDIRECT_MESSAGE  =  5,
    ICMP_TYPE_ALTERNAT_HOST     =  6,
    ICMP_TYPE_ECHO_REQUEST      =  8,
    ICMP_TYPE_ROUTER_ADVERT     =  9,
    ICMP_TYPE_ROUTER_SOLICIT    = 10,
    ICMP_TYPE_TIME_EXCEEDED     = 11,
    ICMP_TYPE_BAD_IP_HEADER     = 12,
    ICMP_TYPE_TIMESTAMP         = 13,
    ICMP_TYPE_TIMESTAMP_REPLY   = 14,
    ICMP_TYPE_INFO_REQUEST      = 15,
    ICMP_TYPE_INFO_REPLY        = 16,
    ICMP_TYPE_ADDRMASK_REQUEST  = 17,
    ICMP_TYPE_ADDRMASK_REPLY    = 18,
    ICMP_TYPE_TRACEROUTE        = 30,
} ICMP_TYPES;

typedef enum {
    ICMP_DU_NETWORK             =  0,
    ICMP_DU_HOST                =  1,
    ICMP_DU_PROTOCOL            =  2,
    ICMP_DU_PORT                =  3,
    ICMP_DU_FRAG                =  4,
    ICMP_DU_SRC_ROUTE           =  5,
    ICMP_DU_DEST_NET_UNKNOWN    =  6,
    ICMP_DU_DEST_HOST_UNKNOWN   =  7,
    ICMP_DU_SRC_ISOLATED        =  8,
    ICMP_DU_NET_ADMIN_PROHIBIT  =  9,
    ICMP_DU_HOST_ADMIN_PROHIBIT = 10,
    ICMP_DU_NET_UNREACHABLE     = 11,
    ICMP_DU_HOST_UNREACHABLE    = 12,
    ICMP_DU_COMM_ADMIN_PROHIBIT = 13,
    ICMP_DU_HOST_PRECEDENCE     = 14,
    ICMP_DU_PRECEDENCE_CUTOFF   = 15
} ICMP_DEST_UNREACHABLE_CODES;

typedef struct __attribute__ ((packed)) {
    uint8_t type;
    uint8_t code;
    uint16_t checksum;
    uint32_t data;
    uint8_t payload[];
} icmp_frame;

class ICMP : public Encapsulator
{
public:
    ICMP();

    int   receive(   network_interface*, void*, int );

    int   construct( network_interface*, void*, int );
    int   construct( network_interface*, void*, int, int, int, int );

    void* get_payload_buffer(void*);
    void  set_payload_length(void*, int);
};

#endif /* _ICMP_H */
