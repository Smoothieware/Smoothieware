#include "netcore.h"

#include <cstdio>
#include <cstring>

#include "net_util.h"

netcore* netcore::instance = NULL;

netcore::netcore() : ip(this), arp(this)
{
    instance = this;
}

bool netcore::add_interface(NetworkInterface* interface)
{
    if (n_interfaces >= MAX_INTERFACES)
        return false;

    interfaces[n_interfaces] = interface;
    interface->provide_net(this);
    n_interfaces++;
    return true;
}

int netcore::receive_packet(NetworkInterface* interface, NET_PACKET packet, int length)
{
    int txlength = 0;

    eth_frame* e = (eth_frame*) packet;
    uint8_t dmac[18];
    uint8_t smac[18];

    format_mac(e->dest_mac, dmac);
    format_mac(e->src_mac, smac);

    printf("ETH: %d bytes To %s From %s Type 0x%04X: ", length, dmac, smac, ntohs(e->e_type));
    for (int i = 12; i < length; i++)
    {
        int c = e->payload[i];
        if ((c >= 32) && (c <= 126))
            printf("%c", c);
        else
            printf("\\x%02X", c);
    }
    printf("\n");

    if (compare_mac(e->dest_mac, broadcast, NULL) || compare_mac(e->dest_mac, interface->mac_address, NULL))
    {
        switch (ntohs(e->e_type))
        {
            case FRAME_TYPE_IPV4:
                txlength = ip.receive(interface, packet, length - sizeof(eth_frame));
                break;
            case FRAME_TYPE_ARP:
                txlength = arp.receive(interface, packet, length - sizeof(eth_frame));
                break;
            case FRAME_TYPE_IPV6:
                break;
        }
    }

    if (txlength)
    {
        memcpy(e->dest_mac, e->src_mac, 6);
        memcpy(e->src_mac, interface->mac_address, 6);
        txlength += sizeof(eth_frame);
    }

    printf("return %d\n", txlength);

    return txlength;
}

int netcore::periodical( int milliseconds, NetworkInterface* interface, NET_PACKET buffer, int bufsize )
{
    int txlength = 0;

    txlength = ip.periodical(milliseconds, interface, buffer, bufsize);

    if (txlength == 0)
        txlength = arp.periodical(milliseconds, interface, buffer, bufsize);

    return txlength;
}

NetworkInterface* netcore::route(IP_ADDR ip)
{
    for (int i = 0; i < n_interfaces; i++)
    {
        NetworkInterface* ni = interfaces[i];
        if ((ni->ip_address & ni->ip_mask) == (ip & ni->ip_mask))
            return ni;
    }
    return NULL;
}

NET_PACKET netcore::get_new_packet_buffer(NetworkInterface* ni)
{
    NET_PACKET packet = (NET_PACKET) ni->request_packet_buffer();
    eth_frame* f = (eth_frame*) packet;
    memcpy(f->src_mac, ni->mac_address, SIZEOF_MAC);
    f->e_type = FRAME_TYPE_IPV4;
    return packet;
}

void netcore::set_dest_mac(NET_PACKET packet, uint8_t* mac)
{
    eth_frame* f = (eth_frame*) packet;
    memcpy(f->dest_mac, mac, SIZEOF_MAC);
}

void netcore::set_type(NET_PACKET packet, int type)
{
    eth_frame* f = (eth_frame*) packet;
    f->e_type = type;
}
