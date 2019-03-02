#ifndef _LPC17XX_ETHERNET_H
#define _LPC17XX_ETHERNET_H

#include "lpc17xx_emac.h"

#include "Module.h"
#include "net_util.h"

#define EMAC_SMSC_8720A 0x0007C0F0

// SMSC 8720A special control/status register
#define EMAC_PHY_REG_SCSR 0x1F

#define LPC17XX_MAX_PACKET 600
#define LPC17XX_TXBUFS     4
#define LPC17XX_RXBUFS     4

typedef struct {
    void* packet;
    uint32_t control;
} packet_desc;

typedef struct {
    uint8_t buf[LPC17XX_RXBUFS][LPC17XX_MAX_PACKET];
    RX_Stat rxstat[LPC17XX_RXBUFS];
    packet_desc rxdesc[LPC17XX_RXBUFS];
} _rxbuf_t;

typedef struct {
    uint8_t buf[LPC17XX_TXBUFS][LPC17XX_MAX_PACKET];
    TX_Stat txstat[LPC17XX_TXBUFS];
    packet_desc txdesc[LPC17XX_TXBUFS];
} _txbuf_t;

class LPC17XX_Ethernet;

class LPC17XX_Ethernet : public Module, public NetworkInterface
{
public:
    LPC17XX_Ethernet();

    void on_module_loaded();
    void on_idle(void*);
    void on_second_tick(void*);

    void emac_init(void) __attribute__ ((optimize("O0")));

    void set_mac(uint8_t*);

    void irq(void);

    bool _receive_frame(void *packet, int* size);

    // NetworkInterface methods
//     void provide_net(netcore* n);
    bool can_read_packet(void);
    int read_packet(uint8_t**);
    void release_read_packet(uint8_t*);
    void periodical(int);

    bool can_write_packet(void);
    int write_packet(uint8_t *, int);

    void* request_packet_buffer(void);

    // Encapsulator methods
    int receive(NetworkInterface* ni, NET_PACKET, int);
    int construct(NetworkInterface* ni, NET_PACKET, int);
    NET_PACKET  get_new_packet_buffer(NetworkInterface*);
    NET_PAYLOAD get_payload_buffer(NET_PACKET);
    void        set_payload_length(NET_PACKET, int);

    static LPC17XX_Ethernet* instance;

private:
    static _rxbuf_t rxbuf;
    static _txbuf_t txbuf;

    void check_interface();
};

#endif /* _LPC17XX_ETHERNET_H */
