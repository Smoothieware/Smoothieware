#include <TCP.h>

#include <stdio.h>
#include <cstring>

TCP::TCP()
{
}

int TCP::receive( network_interface* interface, void* packet, int length)
{
    int txlength = 0;

    return txlength;
}

int TCP::construct( network_interface* interface, void* packet, int length)
{
    tcp_frame* tcf = (tcp_frame*) packet;

    return sizeof(tcp_frame);
}

void* TCP::get_payload_buffer(void* packet)
{
    tcp_frame* tcf = (tcp_frame*) packet;

    return tcf->payload;
}

void TCP::set_payload_length(void* packet, int length)
{
    tcp_frame* tcf = (tcp_frame*) packet;

    // TODO: recalculate checksum
}

int TCP::periodical(int milliseconds, network_interface* interface, void* buffer, int bufferlen)
{
    return 0;
}

// bool TCP::add_interface(network_interface* interface)
// {
//     if (n_interfaces >= MAX_INTERFACES)
//         return false;
//
//     interfaces[n_interfaces] = interface;
//     interface->provide_tcp(this);
//     n_interfaces++;
//     return true;
// }



// int TCP::receive_IPV4(network_interface* interface, uint8_t* buf, uint8_t* payload, int length)
// {
//     int txlength = 0;
//
//     ip_frame* ipf = (ip_frame*) payload;
//
//     uint8_t sip[IP_STR_LEN];
//     uint8_t dip[IP_STR_LEN];
//
//     format_ip(ntohl(ipf->srcip), sip);
//     format_ip(ntohl(ipf->dstip), dip);
//
//     printf("IP packet from %s to %s: protocol 0x%02X, %d byte payload\n", sip, dip, ipf->protocol, ntohs(ipf->total_length));
//
//     printf("checksum read %04X calc %04X\n", ipf->header_checksum, checksum16(payload, length));
//
//     if (ntohl(ipf->dstip) == interface->ip_address)
//     {
//         printf("It's for me!\n");
//         switch (ipf->protocol)
//         {
//             case IP_PROTO_ICMP:
// //                 txlength = receive_ICMP(interface, buf, ipf->payload, ipf->total_length);
//             case IP_PROTO_IPV4:
//             case IP_PROTO_TCP:
//             case IP_PROTO_UDP:
//                 break;
//         }
//     }
//     else if ((ntohl(ipf->dstip) & ~interface->ip_mask) == ~interface->ip_mask)
//     {
//         printf("It's a broadcast on my network segment!\n");
//         switch (ipf->protocol)
//         {
//             case IP_PROTO_ICMP:
// //                 txlength = receive_ICMP(interface, buf, ipf->payload, ipf->total_length);
//             case IP_PROTO_IPV4:
//             case IP_PROTO_UDP:
//                 break;
//         }
//     }
//
//     if (txlength)
//     {
//         txlength += sizeof(ip_frame);
//         ipf->dstip = ipf->srcip;
//         ipf->srcip = htonl(interface->ip_address);
//     }
//
//     return txlength;
// }

// int TCP::receive_packet(network_interface* interface, uint8_t* buf, int length)
// {
//     int txlength = 0;
//
//     eth_frame* e = (eth_frame*) buf;
//     uint8_t dmac[18];
//     uint8_t smac[18];
//
//     format_mac(e->dest_mac, dmac);
//     format_mac(e->src_mac, smac);
//
//     printf("ETH: %d bytes To %s From %s Type 0x%04X: ", length, dmac, smac, ntohs(e->e_type));
//     for (int i = 12; i < length; i++)
//     {
//         int c = buf[i];
//         if ((c >= 32) && (c <= 126))
//             printf("%c", c);
//         else
//             printf("\\x%02X", c);
//     }
//     printf("\n");
//
//     if (compare_mac(e->dest_mac, broadcast, NULL) || compare_mac(e->dest_mac, interface->mac_address, NULL))
//     {
//         switch (ntohs(e->e_type))
//         {
//             case FRAME_TYPE_IPV4:
//                 txlength = receive_IPV4(interface, buf, e->payload, length);
//                 break;
//             case FRAME_TYPE_ARP:
//                 txlength = receive_ARP(interface, buf, e->payload, length);
//                 break;
//             case FRAME_TYPE_IPV6:
//                 break;
//         }
//     }
//
//     if (txlength)
//     {
//         memcpy(e->dest_mac, e->src_mac, 6);
//         memcpy(e->src_mac, interface->mac_address, 6);
//         txlength += e->payload - buf;
//     }
//
//     printf("return %d\n", txlength);
//
//     return txlength;
// }

// int TCP::periodical(network_interface* interface, uint8_t* buf)
// {
//     return 0;
// }
