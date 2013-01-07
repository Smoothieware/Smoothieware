#ifndef _USBETHERNET_H
#define _USBETHERNET_H

#include "USB.h"
#include "descriptor_cdc.h"

#include "Module.h"
#include "TCP.h"

typedef usbdesc_string_l(12) macstr_t;

#define N_BUFFERS 4

class USBEthernet : public USB_Endpoint_Receiver, public Module, public network_interface {
public:
    USBEthernet(USB*);

    bool USBEvent_Request(CONTROL_TRANSFER&);
    bool USBEvent_RequestComplete(CONTROL_TRANSFER&, uint8_t*, uint32_t);

    bool USBEvent_EPIn(uint8_t, uint8_t);
    bool USBEvent_EPOut(uint8_t, uint8_t);

    void on_module_loaded(void);
    void on_main_loop(void*);
    uint32_t on_slow_ticker(uint32_t);

    bool can_read_packet(void);
    int read_packet(uint8_t**);

    bool can_write_packet(void);
    int write_packet(uint8_t *, int);


    USB* usb;

    usbdesc_iad             iad;
    usbdesc_interface       if0;
    usbcdc_header           cdcheader;
    usbcdc_union            cdcunion;
    usbcdc_ether            cdcether;
    usbdesc_endpoint        notifyEP;
    usbdesc_interface       ifnop;
    usbdesc_interface       ifdata;
    usbdesc_endpoint        OutEP;
    usbdesc_endpoint        InEP;

    usbdesc_string_l(17)         iFunction;
    macstr_t                macaddr;

    uint8_t bAlternate;

protected:
    volatile uint8_t pb_head;
    volatile uint8_t pb_tail;
    volatile uint8_t pb_count;
    int pr_index;
    int* packetbuffer[N_BUFFERS];

    uint8_t* buf_push(int);
    uint8_t* buf_pop(int*);

    uint8_t* txpacket;
    int      txindex;
    int      txlength;

    static int interface_count;
};

#endif /* _USBETHERNET_H */
