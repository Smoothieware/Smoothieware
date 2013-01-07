#ifndef _ARP_H
#define _ARP_H

#include <cstdint>

#include "net_util.h"

class ARP;
class network_interface;

#define HARDWARE_TYPE_ETHERNET 1

#define SIZEOF_MAC 6
#define SIZEOF_IP  4

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

typedef struct {
    network_interface* interface;
    uint32_t ip;
    uint8_t mac[6];

    void* next;
} arp_cache_entry;

class ARP : public Encapsulated, public Period_receiver
{
public:
    ARP();

    int   receive(network_interface*, void*, int);

    int   construct(network_interface*, void*, int);

    void* get_payload_buffer(void*);
    void  set_payload_length(void*, int);

    bool  store_mac_ip(network_interface*, uint8_t*, uint32_t);

    bool  get_ip(uint32_t, network_interface**, uint8_t**);
    bool  get_mac(uint8_t*, network_interface**, uint32_t*);

    int   periodical(int, network_interface*, void*, int);

protected:
    arp_cache_entry* arp_cache;
};

#endif /* _ARP_H */
