#include "DHCP.h"

DHCP::DHCP(Encapsulator* new_parent)
{
    parent = new_parent;
}

static uint8_t* find_options(DHCP_Packet* packet, int size, int* optionsize)
{
    uint8_t* x = packet->options;
    uint32_t m = ntohl(DHCP_MAGIC_COOKIE);
    while (x < (((uint8_t*) packet) + size))
    {
        if (memcmp(x, &m, 4) == 0)
        {
            if (optionsize)
                *optionsize = (((uint8_t*) packet) + size) - ((uint8_t*) x);
            return x;
        }
        x++;
    }
    return 0;
}

static int get_option(int option, uint8_t* options, int optionsize, void** location)
{
    for (int i = 0; i < optionsize; i++)
    {
        if (options[i])
        {
            int opt = options[i];
            int size = options[i + 1];
            uint8_t* payload = &(options[i + 2]);
            i += size + 1;

            if (opt == option)
            {
                if (location)
                    *location = payload;
                if (size == 1)
                    return *payload;
                if (size == 2)
                    return ((payload[0] << 8) | payload[1]);
                // if (size >= 4)
                return ((payload[0] << 24) | (payload[1] << 16) | (payload[2] << 8) | payload[3]);
            }
        }
    }
    return -1;
}

int DHCP::receive(NetworkInterface* ni, void* packet, int size)
{
    DHCP_Packet* dp = (DHCP_Packet*) packet;
    if (dp->op == OP_BOOTREPLY)
    {
        int optionsize = 0;
        uint8_t* opt = find_options(dp, size, &optionsize);
        if (opt)
        {
            int msgtype = get_option(OPTION_DHCP_MSGTYPE, opt, optionsize, NULL);
            switch (msgtype)
            {
                case MSGTYPE_DHCPOFFER:
                    printf("DHCP: Offered an address\n");
                case MSGTYPE_DHCPACK:
                    printf("DHCP: Offer acceptance acknowledged\n");
                case MSGTYPE_DHCPDECLINE:
                    printf("DHCP: Request Declined\n");
            }
        }
    }
    return 0;
}

int DHCP::construct(NetworkInterface* ni, void* packet, int size)
{
    DHCP_Packet* dp = (DHCP_Packet*) packet;
    dp->op     = OP_BOOTREQUEST;
    dp->htype  = HARDWARE_TYPE_ETHERNET;
    dp->hlen   = SIZEOF_MAC;
    dp->hops   = 0;
    dp->xid    = 0x12345678; // TODO: random number
    dp->secs   = 0;
    dp->flags  = 0;
    dp->ciaddr = ni->ip_address;
    dp->yiaddr = 0;
    dp->siaddr = 0;
    dp->giaddr = 0;

    uint32_t m = htonl(DHCP_MAGIC_COOKIE);
    memcpy(dp->options, &m, 4);

    dp->options[4] = 0;

    return sizeof(DHCP_Packet);
}

int DHCP::set_option(NetworkInterface* ni, DHCP_Packet* packet, int option, int optionsize, uint8_t* optiondata)
{
    uint8_t* o = packet->options;
    o += 4; // skip magic
    while ((*o) && (*o != option)) {
        o += o[1] + 2;
    }

    o[0] = option;
    o[1] = optionsize;
    memcpy(&(o[2]), optiondata, optionsize);
    o[optionsize + 2] = 0;
}
