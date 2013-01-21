#ifndef _UDP_H
#define _UDP_H

#include "net_util.h"

typedef struct {
    int length;
    uint8_t payload[];
} udp_frame;

class UDP : public Encapsulator
{
public:
    UDP(Encapsulator*);
    int   receive(   NetworkInterface*, NET_PACKET, int);
    int   construct( NetworkInterface*, NET_PACKET, int);

    NET_PACKET get_new_packet_buffer(NetworkInterface*);
    NET_PAYLOAD get_payload_buffer(NET_PACKET);
    void  set_payload_length(NET_PACKET, int);

    int   periodical( int, NetworkInterface*, NET_PACKET, int );

    Encapsulator* parent;
};

#endif /* _UDP_H */
