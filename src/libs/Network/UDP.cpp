#include "UDP.h"

UDP::UDP()
{
}

int UDP::receive( network_interface* interface, void* packet, int length)
{
    int txlength = 0;

    udp_frame* udf = (udp_frame*) packet;

    return txlength;
}

int UDP::construct( network_interface* interface, void* packet, int length)
{
    udp_frame* udf = (udp_frame*) packet;

    return sizeof(udp_frame);
}

void* UDP::get_payload_buffer(void* packet)
{
    udp_frame* udf = (udp_frame*) packet;

    return udf->payload;
}

void  UDP::set_payload_length(void* packet, int length)
{
    udp_frame* udf = (udp_frame*) packet;

    udf->length = sizeof(udp_frame) + length;
}

int UDP::periodical(int milliseconds, network_interface* interface, void* buffer, int bufferlen)
{
    return 0;
}
