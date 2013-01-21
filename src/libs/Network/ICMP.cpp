#include "ICMP.h"

#include "net_util.h"

ICMP::ICMP(Encapsulator* new_parent)
{
    parent = new_parent;
}

int ICMP::receive( NetworkInterface* interface, NET_PACKET packet, int length)
{
    icmp_frame* icf = (icmp_frame*) parent->get_payload_buffer(packet);

    int txlength = 0;

    printf("ICMP: got a %d:%d, %d bytes\n", icf->type, icf->code, length);

    switch (icf->type)
    {
        case ICMP_TYPE_ECHO_REQUEST: {
            icf->type = ICMP_TYPE_ECHO_REPLY;
            icf->checksum = 0;
            icf->checksum = checksum16((uint8_t*) packet, length, 0);
            printf("Replying! %d\n", length);
            return length;
        }
    }

    return txlength;
}

int ICMP::construct( NetworkInterface* interface, NET_PACKET packet, int length)
{
    icmp_frame* icf = (icmp_frame*) parent->get_payload_buffer(packet);

    bzero(icf, sizeof(icmp_frame));

    return sizeof(icmp_frame);
}

int ICMP::construct( NetworkInterface* interface, NET_PACKET packet, int length, int type, int code, int data)
{
    icmp_frame* icf = (icmp_frame*) parent->get_payload_buffer(packet);

    parent->construct(interface, packet, length);

    icf->type = type;
    icf->code = code;
    icf->checksum = 0;
    icf->data = data;

    icf->checksum = checksum16((uint8_t*) packet, sizeof(icmp_frame), 0);

    return sizeof(icmp_frame);
}

NET_PACKET ICMP::get_new_packet_buffer(NetworkInterface* ni)
{
    NET_PACKET packet = parent->get_new_packet_buffer(ni);
    icmp_frame* icf = (icmp_frame*) packet;

    return packet;
}

NET_PAYLOAD ICMP::get_payload_buffer(NET_PACKET packet)
{
    icmp_frame* icf = (icmp_frame*) parent->get_payload_buffer(packet);

    return icf->payload;
}

void ICMP::set_payload_length(NET_PACKET packet, int length)
{
    icmp_frame* icf = (icmp_frame*) parent->get_payload_buffer(packet);

    icf->checksum = 0;
    icf->checksum = checksum16((uint8_t*) packet, sizeof(icmp_frame) + length, 0);
}

int ICMP::periodical(int milliseconds, NetworkInterface* interface, void* buffer, int bufferlen)
{
    return 0;
}
