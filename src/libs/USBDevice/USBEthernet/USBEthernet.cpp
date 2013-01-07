#include "USBEthernet.h"

#include <cstring>

#include <ahbmalloc.h>

#include "Pin.h"
#include "Kernel.h"
#include "SlowTicker.h"

#include "netcore.h"

#define BUFSIZE 1536

int USBEthernet::interface_count = 0;

USBEthernet::USBEthernet(USB* u)
{
    this->usb = u;

    iad = {
        DL_INTERFACE_ASSOCIATION,
        DT_INTERFACE_ASSOCIATION,
        0,                          // bFirstInterface - filled out later
        2,                          // bInterfaceCount - contiguous interfaces associated with this function
        UC_COMM,                    // bFunctionClass
        USB_CDC_SUBCLASS_ETHERNET,  // bFunctionSubClass
        0,                          // bFunctionProtocol
        0,                          // iFunction
    };
    if0 = {
        DL_INTERFACE,               // bLength
        DT_INTERFACE,               // bDescType
        0,                          // bInterfaceNumber: filled out during addInterface()
        0,                          // bAlternateSetting
        1,                          // bNumEndpoints
        UC_COMM,                    // bInterfaceClass
        USB_CDC_SUBCLASS_ETHERNET,  // bInterfaceSubClass
        0,                          // bInterfaceProtocol
        0,                          // iInterface
        0, 0, 0,                    // dummy padding
        this,                       // callback
    };
    notifyEP = {
        DL_ENDPOINT,                // bLength
        DT_ENDPOINT,                // bDescType
        EP_DIR_IN,                  // bEndpointAddress: we provide direction, address is filled in by addEndpoint()
        EA_INTERRUPT,               // bmAttributes
        8,                          // wMaxPacketSize
        16,                         // bInterval
        0,                          // dummy padding
        this,                       // endpoint callback
    };
    cdcheader = {
        USB_CDC_LENGTH_HEADER,      // bLength
        DT_CDC_DESCRIPTOR,          // bDescType
        USB_CDC_SUBTYPE_HEADER,     // bDescSubType
        0x0110,                     // bcdCDC
    };
    cdcunion = {
        USB_CDC_LENGTH_UNION,       // bLength
        DT_CDC_DESCRIPTOR,          // bDescType
        USB_CDC_SUBTYPE_UNION,      // bDescSubType
        0,                          // bMasterInterface
        0,                          // bSlaveInterface0
    };
    cdcether = {
        USB_CDC_LENGTH_ETHER,       // bLength
        DT_CDC_DESCRIPTOR,          // bDescType
        USB_CDC_SUBTYPE_ETHERNET,   // bDescSubType
        0,                          // iMacAddress - filled in later
        0,                          // bmEthernetStatistics
        1526,                       // wMaxSegmentSize
        0,                          // wNumberMCFilters
        0,                          // bNumberPowerFilters
    };
    ifnop = {
        DL_INTERFACE,               // bLength
        DT_INTERFACE,               // bDescType
        0,                          // bInterfaceNumber: filled out during addInterface()
        0,                          // bAlternateSetting
        0,                          // bNumEndpoints
        UC_CDC_DATA,                // bInterfaceClass
        0,                          // bInterfaceSubClass
        0,                          // bInterfaceProtocol
        0,                          // iInterface
        0, 0, 0,                    // dummy padding
        this,                       // callback
    };
    ifdata = {
        DL_INTERFACE,               // bLength
        DT_INTERFACE,               // bDescType
        0,                          // bInterfaceNumber: filled out during addInterface()
        1,                          // bAlternateSetting
        2,                          // bNumEndpoints
        UC_CDC_DATA,                // bInterfaceClass
        0,                          // bInterfaceSubClass
        0,                          // bInterfaceProtocol
        0,                          // iInterface
        0, 0, 0,                    // dummy padding
        this,                       // callback
    };
    InEP = {
        DL_ENDPOINT,                // bLength
        DT_ENDPOINT,                // bDescType
        EP_DIR_IN,                  // bEndpointAddress: we provide direction, address is filled in by addEndpoint()
        EA_BULK,                    // bmAttributes
        MAX_PACKET_SIZE_EPBULK,     // wMaxPacketSize
        0,                          // bInterval
        0,                          // dummy padding
        this,                       // endpoint callback
    };
    OutEP = {
        DL_ENDPOINT,                // bLength
        DT_ENDPOINT,                // bDescType
        EP_DIR_OUT,                 // bEndpointAddress: we provide direction, address is filled in by addEndpoint()
        EA_BULK,                    // bmAttributes
        MAX_PACKET_SIZE_EPBULK,     // wMaxPacketSize
        0,                          // bInterval
        0,                          // dummy padding
        this,                       // endpoint callback
    };

    macstr_t s = usbstring("AEF0285D6621");
    memcpy(&macaddr, &s, sizeof(macaddr));

    usbdesc_string_l(17) description = usbstring("Smoothie Ethernet");
    memcpy(&iFunction, &description, sizeof(iFunction));

    usb->addDescriptor(&iad);
    uint8_t IfAddr =
        usb->addInterface(&if0);
    usb->addDescriptor(&cdcheader);
    usb->addDescriptor(&cdcunion);
    usb->addDescriptor(&cdcether);
    usb->addEndpoint(&notifyEP);
    usb->addInterface(&ifnop);
    uint8_t slaveIfAddr =
        usb->addInterface(&ifdata);
    usb->addEndpoint(&OutEP);
    usb->addEndpoint(&InEP);

    iad.bFirstInterface     = IfAddr;
    iad.iFunction           = if0.iInterface;

    cdcunion.bMasterInterface  = IfAddr;
    cdcunion.bSlaveInterface0  = slaveIfAddr;

    cdcether.iMacAddress    = usb->addString(&macaddr);

    pb_head = pb_tail = pb_count = pr_index = 0;
    for (pb_count = 0; pb_count < N_BUFFERS; pb_count++)
        packetbuffer[pb_count] = (int *) ahbmalloc(BUFSIZE, AHB_BANK_1);
    pb_count = 0;
    txpacket = NULL;

    interface_name = (uint8_t*) malloc(5);
    memcpy(interface_name, "usbX", 5);

    interface_name[3] = interface_count + '0';
    interface_count++;

    bAlternate = 0;

    mac_address = { 0xAE, 0xF0, 0x28, 0x5D, 0x66, 0x22 };
    // fixed address of 192.168.2.27;
    ip_address = (192 << 24) | (168 << 16) | (2 << 8) | (27 << 0);
    ip_mask = 0xFFFFFF00;
}

uint8_t* USBEthernet::buf_push(int length)
{
    __disable_irq();
    if (pb_count >= N_BUFFERS)
    {
        __enable_irq();
        return NULL;
    }
    int* r = packetbuffer[pb_head];
    pb_head = (pb_head + 1) % N_BUFFERS;
    pb_count++;
    ((int*) r)[BUFSIZE - sizeof(int)] = length;
    __enable_irq();
    return (uint8_t*) r;
}

uint8_t* USBEthernet::buf_pop(int* length)
{
    __disable_irq();
    if (pb_count == 0)
    {
        __enable_irq();
        return NULL;
    }
    int* r = packetbuffer[pb_tail];
    pb_tail = (pb_tail + 1) % N_BUFFERS;
    pb_count--;
    if (length)
        *length = ((int*) r)[BUFSIZE - sizeof(int)];
    __enable_irq();
    return (uint8_t*) r;
}

bool USBEthernet::USBEvent_Request(CONTROL_TRANSFER& transfer)
{
    return false;
}

bool USBEthernet::USBEvent_RequestComplete(CONTROL_TRANSFER& transfer, uint8_t* buf, uint32_t len)
{
    return false;
}

bool USBEthernet::USBEvent_EPIn(uint8_t bEP, uint8_t bEPStatus)
{
    if (txpacket)
    {
        int l = 0;
        if (txindex < txlength)
        {
            l = txlength - txindex;
            if (l > InEP.wMaxPacketSize)
                l = InEP.wMaxPacketSize;
            usb->write(InEP.bEndpointAddress, txpacket + txindex, l, InEP.wMaxPacketSize);
            txindex += l;
            if ((txlength >= txindex) && (l < InEP.wMaxPacketSize))
            {
                // completed sending packet
                txpacket = NULL;
                txlength = txindex = 0;
            }
            else
            {
                // we need to send more data
                return true;
            }
        }
        else
        {
            // zlp to end this packet
            usb->write(InEP.bEndpointAddress, NULL, 0, InEP.wMaxPacketSize);
            txpacket = NULL;
            txlength = txindex = 0;
        }
    }
    // we don't want to interrupt until we have a packet
    return false;
}

bool USBEthernet::USBEvent_EPOut(uint8_t bEP, uint8_t bEPStatus)
{
    if (pb_count >= N_BUFFERS)
        return false;

    uint32_t size = 0;

    if (!usb->readEP(OutEP.bEndpointAddress, (((uint8_t*) packetbuffer[pb_head]) + pr_index), &size, OutEP.wMaxPacketSize))
        return false;
    pr_index += size;

    if (size < OutEP.wMaxPacketSize)
    {
        buf_push(pr_index);
        pr_index = 0;
    }

    if (pb_count < N_BUFFERS)
        return true;

    return false;
}

bool USBEthernet::can_read_packet()
{
    return pb_count;
}

int USBEthernet::read_packet(uint8_t** buf)
{
    int length = 0;
    *buf = buf_pop(&length);
    usb->readStart(OutEP.bEndpointAddress, OutEP.wMaxPacketSize);
    return length;
}

bool USBEthernet::can_write_packet()
{
    return(txpacket == NULL);
}

int USBEthernet::write_packet(uint8_t *buf, int length)
{
    if (!can_write_packet())
        return -1;
    txpacket = buf;
    txindex  = 0;
    txlength = length;
    usb->endpointSetInterrupt(InEP.bEndpointAddress, true);
    return length;
}

void USBEthernet::on_main_loop(void* argument)
{
    if (pb_count)
    {
        int length = 0;
        int txlength = 0;
        uint8_t* buf;

        length = read_packet(&buf);

        // TODO: do something with this packet!
        txlength = net->receive_packet(this, buf, length);

        printf("Writing %d length packet\n", txlength);
        if (txlength)
        {
            write_packet(buf, txlength);
        }
    }
}

uint32_t USBEthernet::on_slow_ticker(uint32_t argument)
{
    int txlength = 0;
    uint8_t* buf = buf_push(BUFSIZE);

    txlength = net->periodical(100, this, buf, BUFSIZE);

    while (txlength && can_write_packet())
    {
        write_packet(buf, txlength);
        if (can_write_packet())
            txlength = net->periodical(0, this, buf, BUFSIZE);
    }

    buf_pop(NULL);

    return 0;
}

void USBEthernet::on_module_loaded(void)
{
    register_for_event(ON_MAIN_LOOP);

    this->kernel->slow_ticker->attach( 10, this, &USBEthernet::on_slow_ticker );
}
