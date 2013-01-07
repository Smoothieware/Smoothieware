#include "ARP.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>

#include "net_util.h"
#include "netcore.h"

ARP::ARP()
{
    arp_cache = NULL;
}

bool ARP::store_mac_ip(network_interface* interface, uint8_t* mac, uint32_t ip)
{
    arp_cache_entry* n = (arp_cache_entry*) malloc(sizeof(arp_cache_entry));
    n->interface = interface;
    memcpy(n->mac, mac, SIZEOF_MAC);
    n->ip = ip;
    n->next = 0;

    if (arp_cache)
    {
        arp_cache_entry* head = arp_cache;
        while (head->next)
            head = (arp_cache_entry*) head->next;
        head->next = n;
    }
    else
        arp_cache = n;

    return true;
}

bool ARP::get_ip(uint32_t ip, network_interface** interface, uint8_t** mac)
{
    arp_cache_entry* head = arp_cache;

    if (head == NULL)
        return false;

    do {
        if (head->ip == ip)
        {
            if (interface)
                *interface = head->interface;
            if (mac)
                *mac = head->mac;
            return true;
        }
        head = (arp_cache_entry*) head->next;
    } while (head);

    return false;
}

bool ARP::get_mac(uint8_t* mac, network_interface** interface, uint32_t* ip)
{
    arp_cache_entry* head = arp_cache;

    if (head == NULL)
        return false;

    do {
        if (compare_mac(head->mac, mac, NULL))
        {
            if (interface)
                *interface = head->interface;
            if (ip)
                *ip = head->ip;
            return true;
        }
        head = (arp_cache_entry*) head->next;
    } while (head);

    return false;
}

int ARP::receive(network_interface* interface, void* payload, int length)
{
    arp_frame* a = (arp_frame*) payload;

    if (ntohs(a->htype) != HARDWARE_TYPE_ETHERNET)
        return 0;
    if (ntohs(a->ptype) != FRAME_TYPE_IPV4)
        return 0;
    if (a->hlen != SIZEOF_MAC)
        return 0;
    if (a->plen != SIZEOF_IP)
        return 0;

    switch (ntohs(a->operation))
    {
        case ARP_OPERATION_REQUEST: {
            uint8_t sip[IP_STR_LEN];
            uint8_t smac[MAC_STR_LEN];
            uint8_t dip[IP_STR_LEN];
            uint8_t lip[IP_STR_LEN];

            format_mac((uint8_t*) a->sha, smac);
            format_ip(ntohl(a->spa), sip);
            format_ip(ntohl(a->tpa), dip);
            format_ip(interface->ip_address, lip);
            printf("ARP Request: %s(%s) asks Who Has %s?\n", sip, smac, dip);
            printf("%s has %s\n", interface->get_name(), lip);
            if (interface->ip_address == ntohl(a->tpa))
            {
                printf("that's me! arp frame is %d long\n", sizeof(arp_frame));
                // TODO: construct an ARP reply
                a->operation = htons(ARP_OPERATION_REPLY);
                memcpy(a->tha, a->sha, 6);
                a->tpa = a->spa;

                memcpy(a->sha, interface->mac_address, 6);
                a->spa = htonl(interface->ip_address);
                return sizeof(arp_frame);
            }
            break;
        }
        case ARP_OPERATION_REPLY: {
            uint8_t sip[IP_STR_LEN];
            uint8_t smac[MAC_STR_LEN];
            uint8_t dip[IP_STR_LEN];
            format_mac((uint8_t*) a->sha, smac);
            format_ip(a->spa, sip);
            format_ip(a->tpa, dip);
            printf("ARP Reply to %s: %s has %s\n", dip, smac, sip);
            break;
        }
    }
    return 0;
}

int   ARP::construct(network_interface* interface, void* packet, int length)
{
    arp_frame* arf = (arp_frame*) packet;

    bzero(arf->sha, sizeof(arp_frame));

    arf->htype = HARDWARE_TYPE_ETHERNET;
    arf->ptype = FRAME_TYPE_IPV4;

    arf->hlen  = SIZEOF_MAC;
    arf->plen  = SIZEOF_IP;

    return sizeof(arp_frame);
}
