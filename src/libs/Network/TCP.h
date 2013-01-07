#ifndef _TCP_H
#define _TCP_H

#include <cstdint>

#include "net_util.h"

typedef struct {
    uint16_t sport;

    uint16_t dport;

    uint32_t seq_num;

    uint32_t ack_num;

    uint16_t data_offset:4;
    uint16_t reserved:3;
    uint16_t NS:1;
    uint16_t CWR:1;
    uint16_t ECE:1;
    uint16_t URG:1;
    uint16_t ACK:1;
    uint16_t PSH:1;
    uint16_t RST:1;
    uint16_t SYN:1;
    uint16_t FIN:1;

    uint16_t win;

    uint16_t checksum;

    uint16_t urg;

    uint8_t payload[];
} tcp_frame;

class TCP : public Encapsulator, public Period_receiver
{
public:
    TCP();

    int   receive(   network_interface*, void*, int );
    int   construct( network_interface*, void*, int );

    void* get_payload_buffer( void* );
    void  set_payload_length( void*, int );

    int   get_length( void* );

    int   periodical(int, network_interface*, void*, int);

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
