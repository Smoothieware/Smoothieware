#ifndef _TCP_H
#define _TCP_H

#include <cstdint>

#include "net_util.h"

typedef struct {
    int ip;
    int length;

    uint8_t payload[];
} tcp_frame;

class TCP : public Encapsulator
{
public:
    TCP();

    int   receive(   network_interface*, void*, int );
    int   construct( network_interface*, void*, int );

    void* get_payload_buffer( void* );
    void  set_payload_length( void*, int );

    int   get_length( void* );

protected:
    struct {
        uint32_t sip;
        uint32_t dip;
        uint8_t reserved;
        uint8_t protocol;
        uint16_t length;
    } TCP_pseudo_header;
};

#endif /* _TCP_H */
