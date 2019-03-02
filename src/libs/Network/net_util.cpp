#include "net_util.h"

#include <cstdio>
#include <cstring>

static uint8_t hexalpha[] = "0123456789ABCDEF";
const uint8_t broadcast[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

bool compare_mac(const uint8_t* mac1, const uint8_t* mac2, const uint8_t* mask)
{
    uint8_t m;
    for (int i = 0; i < 6; i++)
    {
        m = 0xFF;
        if (mask)
            m = mask[i];
        if ((mac1[i] & m) != (mac2[i] & m))
            return false;
    }
    return true;
}

uint32_t unaligned_u32(uint8_t* buf)
{
    return buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
}

uint16_t unaligned_u16(uint8_t* buf)
{
    return buf[0] | (buf[1] << 8);
}

int format_ip(uint32_t ip, uint8_t* buf)
{
    uint8_t *p = (uint8_t*) &ip;
    return snprintf((char*) buf, IP_STR_LEN, "%d.%d.%d.%d", p[3], p[2], p[1], p[0]);
}

int format_mac(uint8_t mac[6], uint8_t buf[MAC_STR_LEN])
{
    if (compare_mac(mac, broadcast, broadcast))
    {
        memcpy(buf, "[Broadcast      ]", MAC_STR_LEN);
        return MAC_STR_LEN - 1;
    }

    int i;
    for (i = 0; i < 12; i++)
    {
        buf[i + (i >> 1)] = hexalpha[(mac[i >> 1] >> ((i & 1)?0:4)) & 0x0F];
        if (i < 5)
            buf[(i * 3) + 2] = ':';
    }
    buf[MAC_STR_LEN - 1] = 0;
    return MAC_STR_LEN - 1;
}

int checksum16(uint8_t* buf, int count, int start)
{
    /* Compute Internet Checksum for "count" bytes
     *         beginning at location "addr".
     */
    register uint32_t sum = start;

    while( count > 1 )  {
        /*  This is the inner loop */
        sum += unaligned_u16(buf);
        buf += 2;
        count -= 2;
    }

    /*  Add left-over byte, if any */
    if( count > 0 )
        sum += * (uint8_t *) buf;

    /*  Fold 32-bit sum to 16 bits */
    while (sum & 0xFFFF0000)
        sum = (sum & 0xFFFF) + (sum >> 16);

    return (~sum) & 0xFFFF;
}

uint32_t crc32(uint8_t* buf, int length)
{
    static const uint32_t crc32_table[] =
    {
        0x4DBDF21C, 0x500AE278, 0x76D3D2D4, 0x6B64C2B0,
        0x3B61B38C, 0x26D6A3E8, 0x000F9344, 0x1DB88320,
        0xA005713C, 0xBDB26158, 0x9B6B51F4, 0x86DC4190,
        0xD6D930AC, 0xCB6E20C8, 0xEDB71064, 0xF0000000
    };

    int n;
    uint32_t crc=0;

    for (n = 0; n < length; n++)
    {
        crc = (crc >> 4) ^ crc32_table[(crc ^ (buf[n] >> 0)) & 0x0F];  /* lower nibble */
        crc = (crc >> 4) ^ crc32_table[(crc ^ (buf[n] >> 4)) & 0x0F];  /* upper nibble */
    }

    return crc;
}
