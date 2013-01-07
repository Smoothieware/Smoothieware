#include "netcore.h"

#include <cstdio>
#include <cstring>

#include "net_util.h"

netcore::netcore()
{
}

bool netcore::add_interface(network_interface* interface)
{
    if (n_interfaces >= MAX_INTERFACES)
        return false;

    interfaces[n_interfaces] = interface;
    interface->provide_net(this);
    n_interfaces++;
    return true;
}

int netcore::receive_packet(network_interface* interface, uint8_t* payload, int length)
{
    int txlength = 0;

    eth_frame* e = (eth_frame*) payload;
    uint8_t dmac[18];
    uint8_t smac[18];

    format_mac(e->dest_mac, dmac);
    format_mac(e->src_mac, smac);

    printf("ETH: %d bytes To %s From %s Type 0x%04X: ", length, dmac, smac, ntohs(e->e_type));
    for (int i = 12; i < length; i++)
    {
        int c = payload[i];
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
                txlength = ip.receive(interface, e->payload, length - sizeof(eth_frame));
                break;
            case FRAME_TYPE_ARP:
                txlength = arp.receive(interface, e->payload, length - sizeof(eth_frame));
                break;
            case FRAME_TYPE_IPV6:
                break;
        }
    }

    if (txlength)
    {
        memcpy(e->dest_mac, e->src_mac, 6);
        memcpy(e->src_mac, interface->mac_address, 6);
        txlength += e->payload - payload;
    }

    printf("return %d\n", txlength);

    return txlength;
}

int netcore::periodical(network_interface* interface, uint8_t* buffer, int bufsize)
{
    int txlength = 0;

    return txlength;
}
