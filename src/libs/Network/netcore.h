#ifndef _NETCORE_H
#define _NETCORE_H

#include <cstdint>

class network_interface;

#include "IP.h"
#include "ARP.h"

#define MAX_INTERFACES 2

typedef enum {
    FRAME_TYPE_IPV4     = 0x0800,
    FRAME_TYPE_ARP      = 0x0806,
    FRAME_TYPE_IPV6     = 0x86DD,
} FRAME_TYPES;

typedef struct __attribute__ ((packed)) {
    uint8_t  dest_mac[6];
    uint8_t  src_mac[6];
    uint16_t e_type;
    uint8_t  payload[];
} eth_frame;

class netcore : Period_receiver
{
public:
    netcore();

    bool add_interface( network_interface* );
    int receive_packet( network_interface*, uint8_t*, int);
    int periodical( int, network_interface*, void*, int);

protected:
    IP ip;
    ARP arp;

    network_interface* interfaces[MAX_INTERFACES];
    int n_interfaces;
};

#endif /* _NETCORE_H */
