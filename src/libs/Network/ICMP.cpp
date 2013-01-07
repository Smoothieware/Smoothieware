#include "ICMP.h"

#include "net_util.h"

ICMP::ICMP()
{
}

int ICMP::receive( network_interface* interface, void* packet, int length)
{
    int txlength = 0;

    icmp_frame* icf = (icmp_frame*) packet;

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

int ICMP::construct( network_interface* interface, void* packet, int length)
{
    icmp_frame* icf = (icmp_frame*) packet;

    bzero(icf, sizeof(icmp_frame));

    return sizeof(icmp_frame);
}

int ICMP::construct( network_interface* interface, void* packet, int length, int type, int code, int data)
{
    icmp_frame* icf = (icmp_frame*) packet;

    icf->type = type;
    icf->code = code;
    icf->checksum = 0;
    icf->data = data;

    icf->checksum = checksum16((uint8_t*) packet, sizeof(icmp_frame), 0);

    return sizeof(icmp_frame);
}

void* ICMP::get_payload_buffer(void* packet)
{
    icmp_frame* icf = (icmp_frame*) packet;

    return icf->payload;
}

void ICMP::set_payload_length(void* packet, int length)
{
    icmp_frame* icf = (icmp_frame*) packet;

    icf->checksum = 0;
    icf->checksum = checksum16((uint8_t*) packet, sizeof(icmp_frame) + length, 0);
}
