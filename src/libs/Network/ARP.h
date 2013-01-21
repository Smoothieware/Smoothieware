#ifndef _ARP_H
#define _ARP_H

#include <cstdint>

#include "net_util.h"
#include "StreamOutput.h"

class ARP;
class NetworkInterface;

#define ARP_OPERATION_REQUEST   1
#define ARP_OPERATION_REPLY     2

typedef struct __attribute__ ((packed)) {
    uint16_t htype;
    uint16_t ptype;
    uint8_t hlen;
    uint8_t plen;
    uint16_t operation;
    uint16_t sha[3];
    uint32_t spa;
    uint16_t tha[3];
    uint32_t tpa;
} arp_frame;

struct _arp_cache_entry {
    NetworkInterface* interface;
    uint32_t ip;
    uint8_t mac[6];

    struct _arp_cache_entry* next;
};

typedef struct _arp_cache_entry arp_cache_entry;

class ARP : public Encapsulated, public Period_receiver
{
public:
    ARP(netcore*);

    int   receive(NetworkInterface*, NET_PACKET, int);

    int   construct(NetworkInterface*, NET_PACKET, int);

    NET_PAYLOAD get_payload_buffer(NET_PACKET);
    void  set_payload_length(NET_PACKET, int);

    uint8_t* cache_search(IP_ADDR, NetworkInterface**);

    int   resolve_address(IP_ADDR, uint8_t*, NetworkInterface**, NET_PACKET, int);
    IP_ADDR rarp(uint8_t*, NetworkInterface**);

    bool  store_mac_ip(NetworkInterface*, uint8_t*, uint32_t);

    int   periodical(int, NetworkInterface*, NET_PACKET, int);

    void  dump_arp_table(StreamOutput*);

protected:
    arp_cache_entry* arp_cache;
    netcore*         parent;
};

#endif /* _ARP_H */
