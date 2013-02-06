#ifndef _USB_HPP
#define _USB_HPP

#include "descriptor.h"

#ifndef N_DESCRIPTORS
#define N_DESCRIPTORS 48
#endif

#include "USBDevice.h"

#include "Module.h"

class USB : public USBDevice, public Module {
    static usbdesc_base *descriptors[N_DESCRIPTORS];

    static usbdesc_device device;
    static usbdesc_configuration conf;
public:
    USB();
    USB(uint16_t idVendor, uint16_t idProduct, uint16_t bcdFirmwareRevision);

    void init(void);

    bool USBEvent_busReset(void);
    bool USBEvent_connectStateChanged(bool connected);
    bool USBEvent_suspendStateChanged(bool suspended);

    bool USBEvent_Request(CONTROL_TRANSFER&);
    bool USBEvent_RequestComplete(CONTROL_TRANSFER&, uint8_t *, uint32_t);

    bool USBEvent_EPIn(uint8_t, uint8_t);
    bool USBEvent_EPOut(uint8_t, uint8_t);

    bool USBCallback_setConfiguration(uint8_t configuration);
    bool USBCallback_setInterface(uint16_t interface, uint8_t alternate);

    void on_module_loaded(void);
    void on_idle(void*);

    int addDescriptor(usbdesc_base *descriptor);
    int addDescriptor(void *descriptor);
//     int findDescriptor(uint8_t start, uint8_t type, uint8_t index, uint8_t alternate);
    int addInterface(usbdesc_interface *);
    int addEndpoint(usbdesc_endpoint *);
    int addString(const void *);
    int getFreeEndpoint();
    int findStringIndex(uint8_t strid);

    void dumpDescriptors();
    void dumpDevice(usbdesc_device *);
    void dumpConfiguration(usbdesc_configuration *);
    void dumpInterface(usbdesc_interface *);
    void dumpEndpoint(usbdesc_endpoint *);
    void dumpString(int i);
    void dumpString(usbdesc_string *);
    void dumpCDC(uint8_t *);
};

#endif /* _USB_HPP */
