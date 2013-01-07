#include "IP.h"

#include "netcore.h"
#include "net_util.h"

#include <cstdio>
#include <cstring>

static int calc_checksum(ip_frame* ipf)
{
    int _c = ipf->header_checksum;
    ipf->header_checksum = 0;
    int calc = checksum16((uint8_t*) ipf, sizeof(ip_frame), 0);
    ipf->header_checksum = _c;
    return calc;
}

IP::IP()
{
}

int IP::receive(network_interface* interface, void* payload, int length)
{
    int txlength = 0;

    ip_frame* ipf = (ip_frame*) payload;

    uint8_t sip[IP_STR_LEN];
    uint8_t dip[IP_STR_LEN];

    format_ip(ntohl(ipf->srcip), sip);
    format_ip(ntohl(ipf->dstip), dip);

    printf("IP packet from %s to %s: protocol 0x%02X, %d byte payload (%d)\n", sip, dip, ipf->protocol, ntohs(ipf->total_length), length);

    int ch = calc_checksum(ipf);

//     printf("checksum read %04X calc %04X\n", ipf->header_checksum, ch);

    // discard packets with bad checksum
    if (ipf->header_checksum != ch)
        return 0;

    if (ntohl(ipf->dstip) == interface->ip_address)
    {
        printf("It's for me!\n");
        switch (ipf->protocol)
        {
            case IP_PROTO_ICMP:
                txlength = icmp.receive(interface, ipf->payload, ntohs(ipf->total_length) - sizeof(ip_frame));
                break;
            case IP_PROTO_IPV4:
                break;
            case IP_PROTO_TCP:
                txlength = tcp.receive(interface, ipf->payload, ntohs(ipf->total_length) - sizeof(ip_frame));
                break;
            case IP_PROTO_UDP:
                txlength = udp.receive(interface, ipf->payload, ntohs(ipf->total_length) - sizeof(ip_frame));
                break;
        }
    }
    else if ((ntohl(ipf->dstip) & ~interface->ip_mask) == ~interface->ip_mask)
    {
        printf("It's a broadcast on my network segment!\n");
        switch (ipf->protocol)
        {
            case IP_PROTO_ICMP:
                txlength = icmp.receive(interface, ipf->payload, ntohs(ipf->total_length) - sizeof(ip_frame));
                break;
            case IP_PROTO_IPV4:
                break;
            case IP_PROTO_UDP:
                txlength = udp.receive(interface, ipf->payload, ntohs(ipf->total_length) - sizeof(ip_frame));
                break;
        }
    }

    if (txlength)
    {
        printf("IP: adding %d to frame, %d -> %d\n", sizeof(ip_frame), txlength, txlength + sizeof(ip_frame));
        txlength += sizeof(ip_frame);
        ipf->dstip = ipf->srcip;
        ipf->srcip = htonl(interface->ip_address);
    }

    return txlength;
}

int IP::construct( network_interface* interface, void* payload, int length)
{
    ip_frame* ipf = (ip_frame*) payload;

    ipf->version = 4;
    ipf->ihl = 0;
    ipf->dscp = 0;
    ipf->ecn = 0;
    ipf->total_length = 0;
    ipf->identification = 0;
    ipf->flags = 0;
    ipf->fragment_offset = 0;
    ipf->ttl = 0;
    ipf->protocol = 0;
    ipf->header_checksum = 0;
    ipf->srcip = interface->ip_address;
    ipf->dstip = 0;

    return sizeof(ip_frame);
}

int IP::construct( network_interface* interface, void* payload, int length, uint32_t dest_ip, IP_PROTOCOLS protocol)
{
    ip_frame* ipf = (ip_frame*) payload;

    construct(interface, payload, length);

    ipf->protocol = protocol;
    ipf->dstip = dest_ip;

    return sizeof(ip_frame);
}

void IP::set_ttl(void* payload, int ttl)
{
    ip_frame* ipf = (ip_frame*) payload;
    ipf->ttl = ttl;
}

void* IP::get_payload_buffer(void* packet)
{
    ip_frame* ipf = (ip_frame*) packet;
    return ipf->payload;
}

void IP::set_payload_length(void* packet, int length)
{
    ip_frame* ipf = (ip_frame*) packet;

    ipf->total_length = htons(sizeof(ip_frame) + length);
}

int IP::get_length(void* packet)
{
    ip_frame* ipf = (ip_frame*) packet;

    return ntohs(ipf->total_length);
}