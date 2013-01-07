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
    UDP();
    int   receive(   network_interface*, void*, int);
    int   construct( network_interface*, void*, int);

    void* get_payload_buffer(void*);
    void  set_payload_length(void*, int);

    int   periodical( int, network_interface*, void*, int );
};

#endif /* _UDP_H */
