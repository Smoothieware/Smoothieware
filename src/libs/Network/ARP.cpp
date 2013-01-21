#include "ARP.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>

#include "net_util.h"
#include "netcore.h"

ARP::ARP(netcore* upstream)
{
    arp_cache = NULL;
    parent = upstream;
}

bool ARP::store_mac_ip(NetworkInterface* interface, uint8_t* mac, uint32_t ip)
{
    if (cache_search(ip, NULL) == NULL)
    {
        arp_cache_entry* n = (arp_cache_entry*) malloc(sizeof(arp_cache_entry));
        if (n == NULL)
            return false;

        n->interface = interface;
        memcpy(n->mac, mac, SIZEOF_MAC);
        n->ip = ip;

        // add this entry to BEGINNING of LL
        n->next = arp_cache;
        arp_cache = n;
    }

    return true;
}

int ARP::receive(NetworkInterface* interface, NET_PACKET packet, int length)
{
    arp_frame* a = (arp_frame*) parent->get_payload_buffer(packet);

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

int   ARP::construct(NetworkInterface* interface, NET_PACKET packet, int length)
{
    arp_frame* arf = (arp_frame*) parent->get_payload_buffer(packet);

    parent->construct(interface, packet, length);

    bzero(arf->sha, sizeof(arp_frame));

    arf->htype = HARDWARE_TYPE_ETHERNET;
    arf->ptype = FRAME_TYPE_IPV4;

    arf->hlen  = SIZEOF_MAC;
    arf->plen  = SIZEOF_IP;

    memcpy(arf->sha, interface->mac_address, SIZEOF_MAC);
    arf->spa = interface->ip_address;

    return sizeof(arp_frame);
}

uint8_t* ARP::cache_search(IP_ADDR ip, NetworkInterface** interface)
{
    arp_cache_entry* entry = arp_cache;

    while (entry)
    {
        if (entry->ip == ip)
        {
            if (interface)
                *interface = entry->interface;
            return entry->mac;
        }
        entry = entry->next;
    }

    return NULL;
}

int   ARP::resolve_address(IP_ADDR ip, uint8_t* mac, NetworkInterface** interface, NET_PACKET buffer, int bufferlen)
{
    arp_cache_entry* entry = arp_cache;

    while (entry)
    {
        if (entry->ip == ip)
        {
            memcpy(mac, entry->mac, SIZEOF_MAC);
            if (interface)
                *interface = entry->interface;
            return 0;
        }
        entry = entry->next;
    }

    NetworkInterface* ni = parent->route(ip);
    if (ni)
    {
        arp_frame* arf = (arp_frame*) parent->get_payload_buffer(buffer);

        construct(ni, buffer, bufferlen);

        arf->tpa = ip;
        memset(arf->tha, 0xFF, SIZEOF_MAC);

        return sizeof(arp_frame);
    }

    return -1;
}

IP_ADDR ARP::rarp(uint8_t* mac, NetworkInterface** interface)
{
    arp_cache_entry* entry = arp_cache;

    while (entry)
    {
        if (memcmp(entry->mac, mac, 6) == 0)
        {
            if (interface)
                *interface = entry->interface;
            return entry->ip;
        }
        entry = entry->next;
    }
    return 0;
}

int   ARP::periodical(int milliseconds, NetworkInterface* interface, NET_PACKET buffer, int bufferlen)
{
    return 0;
}

void ARP::dump_arp_table(StreamOutput* out)
{
    arp_cache_entry* entry = arp_cache;

    out->printf("ARP table:\n");
    out->printf("\t%15s %18s %5s", "IP address", "MAC address", "Name");

    int i = 0;

    while (entry)
    {
        out->printf("\t%3d.%3d.%3d.%3d %02X:%02X:%02X:%02X:%02X:%02X %5s", (entry->ip >> 24) & 0xFF, (entry->ip >> 16) & 0xFF, (entry->ip >> 8) & 0xFF, entry->ip & 0xFF, entry->mac[0], entry->mac[1], entry->mac[2], entry->mac[3], entry->mac[4], entry->mac[5], entry->interface->get_name());
        i++;
        entry = entry->next;
    }
    out->printf("ARP table: %i entries\n");
}

