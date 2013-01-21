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

class TCPSocket : public Socket
{
    TCPSocket();
    TCPSocket(int, int);

    int connect(int, int);
    int listen(int);
};

class TCP : public Encapsulator, public Period_receiver
{
public:
    TCP(Encapsulator*);

    int   receive(   NetworkInterface*, NET_PACKET, int );
    int   construct( NetworkInterface*, NET_PACKET, int );

    NET_PAYLOAD get_payload_buffer( NET_PACKET );
    void  set_payload_length( NET_PACKET, int );

    NET_PACKET get_new_packet_buffer(NetworkInterface*);

    int   periodical(int, NetworkInterface*, NET_PACKET, int);

protected:
    struct {
        uint32_t sip;
        uint32_t dip;
        uint8_t reserved;
        uint8_t protocol;
        uint16_t length;
    } TCP_pseudo_header;

    Encapsulator* parent;
};

#endif /* _TCP_H */
