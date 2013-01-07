#ifndef _NET_UTIL_H
#define _NET_UTIL_H

#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cstring>

#define MAC_STR_LEN     18
#define IP_STR_LEN      16

// uint16_t htons(uint16_t v);
#define htons(a) ((((a) >> 8) & 0xFF) | (((a) << 8) & 0xFF00))
#define ntohs(a) htons(a)

// uint32_t htonl(uint32_t v);
#define htonl(a) ((((a) >> 24) & 0x000000FF) | (((a) >> 8) & 0x0000FF00) | (((a) << 8) & 0x00FF0000) | (((a) << 24) & 0xFF000000))
#define ntohl(a) htonl(a)

#define compare_ip(ip1, ip2, mask) (((ip1) & (mask)) == ((ip2) & (mask)))

class netcore;

class network_interface {
public:
    virtual const uint8_t* get_name(void) { return interface_name; };

    inline void provide_net(netcore* n){ net = n; }

    netcore* net;
    uint8_t* interface_name;

    uint32_t ip_address;
    uint32_t ip_mask;

    uint8_t mac_address[6];
};

class Encapsulated
{
    virtual int   receive(network_interface*, void*, int) = 0;
    virtual int   construct(network_interface*, void*, int) = 0;
};

class Encapsulator : public Encapsulated
{
    virtual void* get_payload_buffer(void*) = 0;
    virtual void  set_payload_length(void*, int) = 0;
};

class Period_receiver
{
    virtual int periodical(int, network_interface*, void*, int) = 0;
};

extern const uint8_t broadcast[6];

bool compare_mac(const uint8_t*, const uint8_t*, const uint8_t*);
int format_mac(uint8_t*, uint8_t*);
int format_ip(uint32_t, uint8_t*);
int checksum16(uint8_t*, int, int);

#endif /* _NET_UTIL_H */
