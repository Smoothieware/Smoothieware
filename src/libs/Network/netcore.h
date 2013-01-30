#ifndef _NETCORE_H
#define _NETCORE_H

#include <cstdint>

class NetworkInterface;

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

class netcore;

class netcore : public Period_receiver, public Encapsulator
{
public:
    netcore();

    bool add_interface( NetworkInterface* );
    int receive_packet( NetworkInterface*, NET_PACKET, int);
    int periodical( int, NetworkInterface*, NET_PACKET, int);

    int set_interface_status(NetworkInterface*, bool);

    NET_PACKET get_new_packet_buffer(NetworkInterface*);
    NET_PAYLOAD get_payload_buffer(NET_PACKET);
    void        set_payload_length(NET_PACKET, int);

    int   receive(NetworkInterface*, NET_PACKET, int);
    int   construct(NetworkInterface*, NET_PACKET, int);

    void  set_dest_mac(NET_PACKET, uint8_t*);
    void  set_type(NET_PACKET, int);

    NetworkInterface* route(IP_ADDR ip);

    static netcore* instance;

    // protected:
    NetworkInterface* interfaces[MAX_INTERFACES];
    int n_interfaces;

    IP ip;
    ARP arp;
};

#endif /* _NETCORE_H */
