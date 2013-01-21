#include "UDP.h"

UDP::UDP(Encapsulator* new_parent)
{
    parent = new_parent;
}

int UDP::receive( NetworkInterface* interface, NET_PACKET packet, int length)
{
    udp_frame* udf = (udp_frame*) parent->get_payload_buffer(packet);

    int txlength = 0;

    return txlength;
}

int UDP::construct( NetworkInterface* interface, NET_PACKET packet, int length)
{
    udp_frame* udf = (udp_frame*) parent->get_payload_buffer(packet);

    parent->construct(interface, packet, length);

    return sizeof(udp_frame);
}

NET_PACKET UDP::get_new_packet_buffer(NetworkInterface* ni)
{
    return parent->get_new_packet_buffer(ni);
}

NET_PAYLOAD UDP::get_payload_buffer(NET_PACKET packet)
{
    udp_frame* udf = (udp_frame*) parent->get_payload_buffer(packet);

    return udf->payload;
}

void  UDP::set_payload_length(NET_PACKET packet, int length)
{
    udp_frame* udf = (udp_frame*) parent->get_payload_buffer(packet);

    udf->length = sizeof(udp_frame) + length;

    parent->set_payload_length(packet, udf->length);
}

int UDP::periodical(int milliseconds, NetworkInterface* interface, NET_PACKET buffer, int bufferlen)
{
    return 0;
}
