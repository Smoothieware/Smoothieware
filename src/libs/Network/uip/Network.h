#ifndef _NETWORK_H
#define _NETWORK_H

#include "timer.h"
#include "LPC17XX_Ethernet.h"
#include "Module.h"


class Network : public Module
{
public:
    Network();
    virtual ~Network();

    void on_module_loaded();
    void on_idle(void* argument);
    void on_main_loop(void* argument);
    void on_get_public_data(void* argument);
    void dhcpc_configured(uint32_t ipaddr, uint32_t ipmask, uint32_t ipgw);
    static Network *getInstance() { return instance;}
    void tapdev_send(void *pPacket, unsigned int size);

private:
    void init();
    uint32_t tick(uint32_t dummy);
    void handlePacket();

    static Network *instance;

    LPC17XX_Ethernet *ethernet;

    struct timer periodic_timer, arp_timer;
    uint8_t mac_address[6];
    uint8_t ipaddr[4];
    uint8_t ipmask[4];
    uint8_t ipgw[4];
    volatile uint32_t tickcnt;

};

#endif
