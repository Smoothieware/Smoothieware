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

IP::IP(Encapsulator* upstream) : icmp(this), tcp(this), udp(this)
{
    parent = upstream;
}

int IP::receive(NetworkInterface* interface, NET_PACKET packet, int length)
{
    int txlength = 0;

    ip_frame* ipf = (ip_frame*) parent->get_payload_buffer(packet);

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
                txlength = icmp.receive(interface, packet, ntohs(ipf->total_length) - sizeof(ip_frame));
                break;
            case IP_PROTO_IPV4:
                break;
            case IP_PROTO_TCP:
                txlength = tcp.receive(interface, packet, ntohs(ipf->total_length) - sizeof(ip_frame));
                break;
            case IP_PROTO_UDP:
                txlength = udp.receive(interface, packet, ntohs(ipf->total_length) - sizeof(ip_frame));
                break;
        }
    }
    else if ((ntohl(ipf->dstip) & ~interface->ip_mask) == ~interface->ip_mask)
    {
        printf("It's a broadcast on my network segment!\n");
        switch (ipf->protocol)
        {
            case IP_PROTO_ICMP:
                txlength = icmp.receive(interface, packet, ntohs(ipf->total_length) - sizeof(ip_frame));
                break;
            case IP_PROTO_IPV4:
                break;
            case IP_PROTO_UDP:
                txlength = udp.receive(interface, packet, ntohs(ipf->total_length) - sizeof(ip_frame));
                break;
        }
    }

    if (txlength)
    {
//         printf("IP: adding %d to frame, %d -> %d\n", sizeof(ip_frame), txlength, txlength + sizeof(ip_frame));
        txlength += sizeof(ip_frame);
        ipf->dstip = ipf->srcip;
        ipf->srcip = htonl(interface->ip_address);
    }

    return txlength;
}

int IP::construct( NetworkInterface* interface, NET_PACKET packet, int length)
{
    ip_frame* ipf = (ip_frame*) parent->get_payload_buffer(packet);

    parent->construct(interface, packet, length);

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

int IP::construct( NetworkInterface* interface, NET_PACKET payload, int length, uint32_t dest_ip, IP_PROTOCOLS protocol)
{
    ip_frame* ipf = (ip_frame*) parent->get_payload_buffer(payload);

    construct(interface, payload, length);

    ipf->protocol = protocol;
    ipf->dstip = dest_ip;

    return sizeof(ip_frame);
}

void IP::set_ttl(NET_PACKET payload, int ttl)
{
    ip_frame* ipf = (ip_frame*) parent->get_payload_buffer(payload);
    ipf->ttl = ttl;
}

NET_PACKET IP::get_new_packet_buffer(NetworkInterface* ni)
{
    return parent->get_new_packet_buffer(ni);
}

NET_PAYLOAD IP::get_payload_buffer(NET_PACKET packet)
{
    ip_frame* ipf = (ip_frame*) parent->get_payload_buffer(packet);
    return ipf->payload;
}

void IP::set_payload_length(NET_PACKET packet, int length)
{
    ip_frame* ipf = (ip_frame*) parent->get_payload_buffer(packet);

    ipf->total_length = htons(sizeof(ip_frame) + length);
}

int IP::get_length(NET_PACKET packet)
{
    ip_frame* ipf = (ip_frame*) parent->get_payload_buffer(packet);

    return ntohs(ipf->total_length);
}

int IP::periodical(int milliseconds, NetworkInterface* interface, NET_PACKET buffer, int buflen)
{
    int txlength = icmp.periodical(milliseconds, interface, buffer, buflen);
    if (txlength == 0)
        txlength = tcp.periodical(milliseconds, interface, buffer, buflen);
    if (txlength == 0)
        txlength = udp.periodical(milliseconds, interface, buffer, buflen);
    if (txlength == 0)
    {
        // TODO: IP periodicals
    }
    return txlength;
}

void IP::set_dest_ip(NET_PACKET packet, IP_ADDR ip)
{
    ip_frame* p = (ip_frame*) parent->get_payload_buffer(packet);
    p->dstip = ip;
//     uint8_t mac[6];
//     int r = parent->arp->resolve_address(ip, mac, NULL, packet, 1536);
//     if (r == 0)
//         parent->set_dest_mac(packet, mac);
//     else if (r > 0)
//
}
