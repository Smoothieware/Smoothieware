#ifndef _NET_UTIL_H
#define _NET_UTIL_H

#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cstring>

#define HARDWARE_TYPE_ETHERNET 1

#define SIZEOF_MAC 6
#define SIZEOF_IP  4

#define MAC_STR_LEN     18
#define IP_STR_LEN      16

// uint16_t htons(uint16_t v);
#define htons(a) ((((a) >> 8) & 0xFF) | (((a) << 8) & 0xFF00))
#define ntohs(a) htons(a)

// uint32_t htonl(uint32_t v);
#define htonl(a) ((((a) >> 24) & 0x000000FF) | (((a) >> 8) & 0x0000FF00) | (((a) << 8) & 0x00FF0000) | (((a) << 24) & 0xFF000000))
#define ntohl(a) htonl(a)

#define compare_ip(ip1, ip2, mask) (((ip1) & (mask)) == ((ip2) & (mask)))

#define IPA(a, b, c, d) (((a) << 24) | ((b) << 16) | ((c) << 8) | (d))

typedef uint32_t  IP_ADDR;
typedef uint32_t* NET_PACKET;
typedef uint8_t*  NET_PAYLOAD;

// class netcore;
class Encapsulated;
class Encapsulator;
class NetworkInterface;

class Encapsulated
{
public:
    virtual int   receive(NetworkInterface*, NET_PACKET, int) = 0;
    virtual int   construct(NetworkInterface*, NET_PACKET, int) = 0;
};

class Encapsulator : public Encapsulated
{
public:
    virtual NET_PACKET  get_new_packet_buffer(NetworkInterface*)    = 0;
    virtual NET_PAYLOAD get_payload_buffer(NET_PACKET)      = 0;
    virtual void        set_payload_length(NET_PACKET, int) = 0;
};

class Period_receiver
{
public:
    virtual int periodical(int, NetworkInterface*, NET_PACKET, int) = 0;
};

class NetworkInterface : public Encapsulator {
public:
    virtual const uint8_t* get_name(void) { return interface_name; };
//     virtual void provide_net(netcore* n){ net = n; }

//     virtual bool if_up(void) = 0;

    virtual bool can_read_packet(void) = 0;
    virtual int read_packet(uint8_t**) = 0;
    void release_read_packet(uint8_t*);

    virtual bool can_write_packet(void) = 0;
    virtual int write_packet(uint8_t *, int) = 0;

    virtual void* request_packet_buffer(void) = 0;

    virtual void set_ip(uint32_t new_ip)     { ip_address = new_ip; };
    virtual void set_mac(uint8_t new_mac[6]) { memcpy(mac_address, new_mac, 6); };

    bool isUp() { return up; }

//     netcore* net;
    uint8_t* interface_name;

    IP_ADDR ip_address;
    IP_ADDR ip_mask;

    uint8_t mac_address[6];

    bool up;
};

extern const uint8_t broadcast[6];

bool compare_mac(const uint8_t*, const uint8_t*, const uint8_t*);
int format_mac(uint8_t*, uint8_t*);
int format_ip(uint32_t, uint8_t*);
int checksum16(uint8_t*, int, int);
uint32_t crc32(uint8_t* buf, int length);

#endif /* _NET_UTIL_H */
