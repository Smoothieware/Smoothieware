#include "USB.h"

#include <cstdio>

#include "descriptor_cdc.h"
#include "descriptor_msc.h"

#define iprintf(...) do { } while (0)

usbdesc_base *USB::descriptors[N_DESCRIPTORS];

usbdesc_device USB::device = {
	DL_DEVICE,
	DT_DEVICE,
	USB_VERSION_2_0,	// .bcdUSB
	UC_MISC,        	// .bDeviceClass
	SUBCLASS_IAD,	    // .bDeviceSubClass
	PROTOCOL_IAD,		// .bDeviceProtocol
	64,					// .bMaxPacketSize0
	0x1d50,				// .idVendor
	0x6015,				// .idProduct
	0x0100,				// .bcdDevice
	0,					// .iManufacturer
	0,					// .iProduct
	0,					// .iSerialNumber
	1,					// .bNumConfigurations
};

static usbdesc_language lang = {
	DL_LANGUAGE,
	DT_LANGUAGE,
	{ SL_USENGLISH, },
};

static usbstring_const_init(manufacturer, "Uberclock");

static usbstring_const_init(product, "Smoothieboard");

static usbstring_init(serial, "01234567abcdefgh01234567abcdefgh");

static usbstring_const_init(str_default, "Default");


usbdesc_configuration USB::conf = {
	DL_CONFIGURATION,
	DT_CONFIGURATION,
	sizeof(usbdesc_configuration),
	0,					// .bNumInterfaces
	1,					// .bConfigurationValue
	0,					// .iConfiguration
	CA_BUSPOWERED,		// .bmAttributes
	500 mA,				// .bMaxPower
};

USB::USB() {
	int i;

	i = 0;
	descriptors[i++] = (usbdesc_base *) &device;
	descriptors[i++] = (usbdesc_base *) &conf;
	for (;i < N_DESCRIPTORS; i++)
		descriptors[i] = (usbdesc_base *) 0;

	addString(&lang);
	device.iManufacturer = addString(&manufacturer);
	device.iProduct = addString(&product);
	device.iSerialNumber = addString(&serial);
	conf.iConfiguration = addString(&str_default);
}

USB::USB(uint16_t idVendor, uint16_t idProduct, uint16_t bcdFirmwareRevision) {
    USB();
    device.idVendor = idVendor;
    device.idProduct = idProduct;
    device.bcdDevice = bcdFirmwareRevision;
}

void USB::init() {
	iprintf("init\n");

	uint32_t chip_serial[4];
	getSerialNumber(4, chip_serial);
	for (int j = 0; j < 4; j++) {
		for (int i = 0; i < 8; i++) {
			uint8_t c = (chip_serial[j] & 15);
			serial.str[j * 8 + 7 - i] = (c < 10)?(c + '0'):(c - 10 + 'A');
			chip_serial[j] >>= 4;
		}
	}

	setDescriptors(descriptors);

    USBHAL::init();

	iprintf("OK\n");
}

bool USB::USBCallback_setConfiguration(uint8_t configuration)
{
    iprintf("[CONFIGURE %d]", configuration);
    int i = findDescriptorIndex(0, DT_CONFIGURATION, configuration, 0);
    if (i > 0)
    {
        iprintf("[FOUND at %d]", i);
        uint8_t lastAlternate = 0;
        i++;
        for (; descriptors[i] != (usbdesc_base *) 0 && descriptors[i]->bDescType != DT_CONFIGURATION; i++) {
            switch(descriptors[i]->bDescType)
            {
                case DT_INTERFACE: {
                    usbdesc_interface *interface = (usbdesc_interface *) descriptors[i];
                    iprintf("[INTERFACE %d:%d]",interface->bInterfaceNumber, interface->bAlternateSetting);
                    interface->selectedAlternate = 0;
                    lastAlternate = interface->bAlternateSetting;
                    break;
                };
                case DT_ENDPOINT: {
                    usbdesc_endpoint *ep = (usbdesc_endpoint *) descriptors[i];
                    iprintf("[EP 0x%02X](%d)", ep->bEndpointAddress, lastAlternate);
                    if (lastAlternate == 0) {
                        iprintf("[Realised!]\n");
                        realiseEndpoint(ep->bEndpointAddress, ep->wMaxPacketSize, ((ep->bmAttributes & 3) == EA_ISOCHRONOUS)?ISOCHRONOUS:0);
                    }
                    break;
                };
            }
        }
        return true;
    }
    return false;
}

bool USB::USBCallback_setInterface(uint16_t interface, uint8_t alternate) {
    int i = findDescriptorIndex(0, DT_INTERFACE, interface, alternate);
    if (i > 0)
    {
        int j = findDescriptorIndex(0, DT_INTERFACE, interface, 0);
        if (j > 0) {
            ((usbdesc_interface *) descriptors[j])->selectedAlternate = alternate;
            for (; (descriptors[i]->bDescType != DT_INTERFACE) && (descriptors[i]->bDescType != DT_CONFIGURATION); i++) {
                if (descriptors[i]->bDescType == DT_ENDPOINT) {
                    usbdesc_endpoint *ep = (usbdesc_endpoint *) descriptors[i];
                    realiseEndpoint(ep->bEndpointAddress, ep->wMaxPacketSize, ((ep->bmAttributes & 3) == EA_ISOCHRONOUS)?ISOCHRONOUS:0);
                }
            }
        }
    }
    return false;
}

bool USB::USBEvent_busReset(void)
{
    iprintf("USB:Bus Reset\n");
    USBDevice::USBEvent_busReset();

    return false;
}

bool USB::USBEvent_connectStateChanged(bool connected)
{
    // TODO: something useful
    if (connected)
    {
        iprintf("USB:Connected\n");
        connect();
    }
    else
    {
        iprintf("USB:Disconnected\n");
        disconnect();
    }
    return true;
};

bool USB::USBEvent_suspendStateChanged(bool suspended)
{
    // TODO: something useful
    if (suspended)
    {
        iprintf("USB:Suspended\n");
    }
    else
    {
        iprintf("USB:Unsuspended\n");
    }
    return true;
};

bool USB::USBEvent_Request(CONTROL_TRANSFER& transfer)
{
/*    if (transfer.setup.bmRequestType.Type == CLASS_TYPE)
        iprintf("[CLASS]");
    if (transfer.setup.bmRequestType.Type == VENDOR_TYPE)
        iprintf("[VENDOR]")*/;
    if (
        (transfer.setup.bmRequestType.Type == CLASS_TYPE)
        ||
        (transfer.setup.bmRequestType.Type == VENDOR_TYPE)
       )
    {
        // decode request destination
        switch(transfer.setup.bmRequestType.Recipient)
        {
            case INTERFACE_RECIPIENT: {
//                 iprintf("[INTERFACE %d]", transfer.setup.wIndex);
                int i = findDescriptorIndex(0, DT_INTERFACE, transfer.setup.wIndex, 0);
                if (i >.0)
                {
                    usbdesc_interface *interface = (usbdesc_interface *) descriptors[i];
//                     iprintf("[FOUND at %d, handler is %p]", i, interface->classReceiver);
                    bool r = interface->classReceiver->USBEvent_Request(transfer);
//                     if (r)
//                         iprintf("[HANDLED]\n");
//                     else
//                         iprintf("[NOT handled]\n");
                    return r;
                }
                break;
            };
            case ENDPOINT_RECIPIENT: {
//                 iprintf("[ENDPOINT 0x%02X]", transfer.setup.wIndex);
                int i = findDescriptorIndex(0, DT_ENDPOINT, transfer.setup.wIndex, 0);
                if (i >.0)
                {
                    usbdesc_endpoint *ep = (usbdesc_endpoint *) descriptors[i];
//                     iprintf("[FOUND at %d, handler is %p]", i, ep->epReceiver);
                    bool r = ep->epReceiver->USBEvent_Request(transfer);
//                     if (r)
//                         iprintf("[HANDLED]\n");
//                     else
//                         iprintf("[NOT handled]\n");
                    return r;
                }
                break;
            }
        }
    }
//     iprintf("\n");
    return false;
};

bool USB::USBEvent_RequestComplete(CONTROL_TRANSFER& transfer, uint8_t *buf, uint32_t length)
{
    if ((transfer.setup.bmRequestType.Type == CLASS_TYPE) ||
        (transfer.setup.bmRequestType.Type == VENDOR_TYPE))
    {
        // decode request destination
        switch(transfer.setup.bmRequestType.Recipient)
        {
            case INTERFACE_RECIPIENT: {
                int i = findDescriptorIndex(0, DT_INTERFACE, transfer.setup.wIndex, 0);
                if (i >.0)
                {
                    usbdesc_interface *interface = (usbdesc_interface *) descriptors[i];
                    interface->classReceiver->USBEvent_RequestComplete(transfer, buf, length);
                }
                break;
            };
            case ENDPOINT_RECIPIENT: {
                int i = findDescriptorIndex(0, DT_ENDPOINT, transfer.setup.wIndex, 0);
                if (i >.0)
                {
                    usbdesc_endpoint *ep = (usbdesc_endpoint *) descriptors[i];
                    ep->epReceiver->USBEvent_RequestComplete(transfer, buf, length);
                }
                break;
            }
        }
    }
    return false;
    return false;
};

bool USB::USBEvent_EPIn(uint8_t bEP, uint8_t bEPStatus)
{
    int i = findDescriptorIndex(0, DT_ENDPOINT, bEP, 0);
    if (i > 0)
    {
        usbdesc_endpoint *ep = (usbdesc_endpoint *) descriptors[i];
//         iprintf("[EPIn 0x%02X ST 0x%02X Handler %p]", bEP, bEPStatus, ep->epReceiver);
        return ep->epReceiver->USBEvent_EPIn(bEP, bEPStatus);
    }
    return false;

};

bool USB::USBEvent_EPOut(uint8_t bEP, uint8_t bEPStatus)
{
//     iprintf("[EPOut 0x%02X ST 0x%02X]\n", bEP, bEPStatus);
    int i = findDescriptorIndex(0, DT_ENDPOINT, bEP, 0);
    if (i > 0)
    {
//         iprintf("[@%d ", i);
        usbdesc_endpoint *ep = (usbdesc_endpoint *) descriptors[i];
//         iprintf("Handler %p]\n", ep->epReceiver);
        return ep->epReceiver->USBEvent_EPOut(bEP, bEPStatus);
    }
    return false;
};


int USB::addDescriptor(usbdesc_base *descriptor) {
	for (int i = 0; i < N_DESCRIPTORS; i++) {
		if (descriptors[i] == (usbdesc_base *) 0) {
			descriptors[i] = descriptor;
			conf.wTotalLength += descriptor->bLength;
			return i;
		}
	}
	return -1;
}

int USB::addDescriptor(void *descriptor)
{
	return addDescriptor((usbdesc_base *) descriptor);
}

int USB::addInterface(usbdesc_interface *ifp) {
	usbdesc_interface *lastif = (usbdesc_interface *) 0;
	uint8_t i;
	for (i = 0; i < N_DESCRIPTORS; i++) {
		if (descriptors[i] == (usbdesc_base *) 0)
			break;
		if (descriptors[i]->bDescType == DT_INTERFACE)
			lastif = (usbdesc_interface *) descriptors[i];
	}
	if (i >= N_DESCRIPTORS) return -1;

	uint8_t n = conf.bNumInterfaces;

	if (lastif)
		if (ifp->bAlternateSetting != lastif->bAlternateSetting)
			n--;

	ifp->bInterfaceNumber = n;
// 	iprintf("[inserting Interface %p at %d]\n", ifp, i);
	descriptors[i] = (usbdesc_base *) ifp;
	conf.bNumInterfaces = n + 1;
	conf.wTotalLength += descriptors[i]->bLength;

	return n;
}

int USB::getFreeEndpoint() {
	uint8_t i;
	uint8_t lastii = 0;
	usbdesc_interface *lastif = (usbdesc_interface *) 0;

	for (i = 0; i < N_DESCRIPTORS; i++) {
		if (descriptors[i] == (usbdesc_base *) 0)
			break;
		if (descriptors[i]->bDescType == DT_INTERFACE) {
			lastif = (usbdesc_interface *) descriptors[i];
			lastii = i;
		}
	}
	if (i >= N_DESCRIPTORS) return -1;
	if (lastif == (usbdesc_interface *) 0) return -1;
	if (lastii == 0) return -1;

	uint8_t ep_count = 0;
	for (i = lastii; i < N_DESCRIPTORS; i++) {
		if (descriptors[i] == (usbdesc_base *) 0)
			return ep_count;
		if (descriptors[i]->bDescType == DT_ENDPOINT)
			ep_count++;
	}
	return ep_count;
}

int USB::addEndpoint(usbdesc_endpoint *epp) {
// 	iprintf("[EP ");
	uint8_t i;
	usbdesc_interface *lastif = (usbdesc_interface *) 0;
	for (i = 0; i < N_DESCRIPTORS; i++) {
		if (descriptors[i] == (usbdesc_base *) 0)
			break;
		if (descriptors[i]->bDescType == DT_INTERFACE)
			lastif = (usbdesc_interface *) descriptors[i];
	}
// 	iprintf("%d:%p ", i, lastif);
	if (i >= N_DESCRIPTORS) return -1;
	if (lastif == (usbdesc_interface *) 0) return -1;

	int n = getFreeEndpoint();

    // TODO: move this to the hardware-specific class
	// we need to scan through our descriptors, and find the first unused logical endpoint of the appropriate type
	// the LPC17xx has 16 logical endpoints mapped to 32 "physical" endpoints, looks like this means we have 16 endpoints to use and we get to pick direction
	// SO, let's first find all the endpoints in our list of the same type, pick the highest address then go find the next one

	// pick a starting logical endpoint- interrupt = 1, bulk = 2, iso = 3 then subtract 3 so we pick the first one later on
	int lepaddr = (4 - epp->bmAttributes) - 3;

	for (i = 0; i < N_DESCRIPTORS; i++) {
		if (descriptors[i] == (usbdesc_base *) 0)
			break;
		if (descriptors[i]->bDescType == DT_ENDPOINT) {
			usbdesc_endpoint *x = (usbdesc_endpoint *) descriptors[i];
			if (x->bmAttributes == epp->bmAttributes && ((x->bEndpointAddress & 0x80) == (epp->bEndpointAddress & 0x80)))
				if ((x->bEndpointAddress & 0x0F) > lepaddr)
					lepaddr = (x->bEndpointAddress & 0x0F);
		}
	}

	// now, lepaddr is the last logical endpoint of appropriate type
	// the endpoints go in groups of 3, except for the last one which is a bulk instead of isochronous
	// find the next free lep using this simple pattern
	if (epp->bmAttributes == EA_BULK && lepaddr == 14)
		lepaddr = 15;
	else
		lepaddr += 3;
	// now we have the next free logical endpoint of the appropriate type

	// if it's >15 we've run out, spit an error
	if (lepaddr > 15) return -1;

	// store logical address and direction bit
	epp->bEndpointAddress = lepaddr | (epp->bEndpointAddress & 0x80);

	descriptors[i] = (usbdesc_base *) epp;
	// 	lastif->bNumEndPoints = n + 1;

	conf.wTotalLength += descriptors[i]->bLength;

	return n;
}

int USB::addString(const void *ss) {
	const usbdesc_base *s = (const usbdesc_base *) ss;
	if (s->bDescType == DT_STRING) {
		uint8_t i;
		uint8_t stringcount = 0;
		for (i = 0; i < N_DESCRIPTORS; i++) {
			if (descriptors[i] == (usbdesc_base *) 0)
				break;
			if (descriptors[i]->bDescType == DT_STRING)
				stringcount++;
		}
		if (i >= N_DESCRIPTORS) return -1;

		descriptors[i] = const_cast<usbdesc_base*>(s);

		return stringcount;
	}
	return -1;
}

int USB::findStringIndex(uint8_t strid) {
	uint8_t i;
	uint8_t strcounter = 0;
	for (i = 0; i < N_DESCRIPTORS; i++) {
		if (descriptors[i] == (usbdesc_base *) 0)
			return -1;
		if (descriptors[i]->bDescType == DT_STRING) {
			if (strcounter == strid)
				return i;
			strcounter++;
		}
	}
	return -1;
}

void USB::dumpDevice(usbdesc_device *d) {
	iprintf("Device:\n");
	iprintf("\tUSB Version:  %d.%d\n", d->bcdUSB >> 8, (d->bcdUSB & 0xFF) >> 4);
	iprintf("\tClass:        0x%04X\n", d->bDeviceClass);
	iprintf("\tSubClass:     0x%04X\n", d->bDeviceSubClass);
	iprintf("\tProtocol:     0x%04X\n", d->bDeviceProtocol);
	iprintf("\tMax Packet:   %d\n", d->bMaxPacketSize);
	iprintf("\tVendor:       0x%04X\n", d->idVendor);
	iprintf("\tProduct:      0x%04X\n", d->idProduct);
	iprintf("\tManufacturer: "); dumpString(d->iManufacturer);
	iprintf("\tProduct:      "); dumpString(d->iProduct);
	iprintf("\tSerial:       "); dumpString(d->iSerialNumber);
	iprintf("\tNum Configs:  %d\n", d->bNumConfigurations);
}

void USB::dumpConfiguration(usbdesc_configuration *c) {
	iprintf("Configuration:\n");
	iprintf("\tTotal Length:        %db\n", c->wTotalLength);
	iprintf("\tNum Interfaces:      %d\n", c->bNumInterfaces);
	iprintf("\tConfiguration Value: %d\n", c->bConfigurationValue);
	iprintf("\tConfiguration:       "); dumpString(c->iConfiguration);
	iprintf("\tAttributes:          %s\n", ((c->bmAttributes & 0x80)?"Bus Powered":"Self Powered"));
	iprintf("\tMax Power:           %dmA\n", c->bMaxPower * 2);
}

void USB::dumpInterface(usbdesc_interface *i) {
	iprintf("\t*Interface\n");
	iprintf("\t\tNumber:        %d\n", i->bInterfaceNumber);
	iprintf("\t\tAlternate:     %d\n", i->bAlternateSetting);
	iprintf("\t\tNum Endpoints: %d\n", i->bNumEndPoints);
	iprintf("\t\tClass:         0x%02X ", i->bInterfaceClass);
	switch(i->bInterfaceClass) {
		case UC_COMM:
			iprintf("(COMM)");
			break;
		case UC_MASS_STORAGE:
			iprintf("(MSC)");
			break;
		case UC_CDC_DATA:
			iprintf("(CDC DATA)");
			break;
	}
	iprintf("\n");
	iprintf("\t\tSubClass:      0x%02X ", i->bInterfaceSubClass);
	switch(i->bInterfaceClass) {
		case UC_COMM: {
			switch(i->bInterfaceSubClass) {
				case USB_CDC_SUBCLASS_ACM:
					iprintf("(ACM)");
					break;
				case USB_CDC_SUBCLASS_ETHERNET:
					iprintf("(ETHERNET)");
					break;
			}
			break;
		}
		case UC_MASS_STORAGE: {
			switch(i->bInterfaceSubClass) {
				case MSC_SUBCLASS_SCSI:
					iprintf("(SCSI)");
					break;
			}
			break;
		}
	}
	iprintf("\n");
	iprintf("\t\tProtocol:      0x%02X ", i->bInterfaceProtocol);
	iprintf("\n");
	iprintf("\t\tInterface:     "); dumpString(i->iInterface);
}
void USB::dumpEndpoint(usbdesc_endpoint *e) {
	static const char* const attr[4] __attribute__ ((used)) = { "", "Isochronous", "Bulk", "Interrupt" };
	iprintf("\t\t*Endpoint\n");
	iprintf("\t\t\tAddress:    0x%02X (%s)\n", e->bEndpointAddress, ((e->bEndpointAddress & EP_DIR_IN)?"IN":"OUT"));
	iprintf("\t\t\tAttributes: 0x%02X (%s)\n", e->bmAttributes, attr[e->bmAttributes]);
	iprintf("\t\t\tMax Packet: %d\n", e->wMaxPacketSize);
	iprintf("\t\t\tInterval:   %d\n", e->bInterval);
}

void USB::dumpString(int i) {
	if (i > 0) {
		uint8_t j = findStringIndex(i);
		if (j > 0) {
			iprintf("[%d] ", i);
			dumpString((usbdesc_string *) descriptors[j]);
			return;
		}
	}
	iprintf("-none-\n");
}

void USB::dumpString(usbdesc_string *s) {
	uint8_t i;
	for (i = 0; i < (s->bLength - 2) / 2; i++) {
		if (s->str[i] >= 32 && s->str[i] < 128)
			putchar(s->str[i]);
		else
			iprintf("\\0x%02X", s->str[i]);
	}
	putchar('\n');
}

void USB::dumpCDC(uint8_t *d) {
	switch(d[2]) {
		case USB_CDC_SUBTYPE_HEADER: {
			usbcdc_header *h = (usbcdc_header *) d;
            if (h)
            {
                iprintf("\t\t*CDC header\n");
                iprintf("\t\t\tbcdCDC:  0x%04X\n", h->bcdCDC);
            }
			break;
		}
		case USB_CDC_SUBTYPE_UNION: {
			usbcdc_union *u = (usbcdc_union *) d;
            if (u)
            {
                iprintf("\t\t*CDC union\n");
                iprintf("\t\t\tMaster:  %d\n", u->bMasterInterface);
                iprintf("\t\t\tSlave:   %d\n", u->bSlaveInterface0);
            }
			break;
		}
		case USB_CDC_SUBTYPE_CALL_MANAGEMENT: {
			usbcdc_callmgmt *m = (usbcdc_callmgmt *) d;
			iprintf("\t\t*CDC Call Management\n");
			iprintf("\t\t\tCapabilities:  0x%02X ", m->bmCapabilities);
			if (m->bmCapabilities & USB_CDC_CALLMGMT_CAP_CALLMGMT)
				iprintf("(CALLMGMT)");
			if (m->bmCapabilities & USB_CDC_CALLMGMT_CAP_DATAINTF)
				iprintf("(DATAINTF)");
			iprintf("\n");
			iprintf("\t\t\tData Interface: %d\n", m->bDataInterface);
			break;
		}
		case USB_CDC_SUBTYPE_ACM: {
			usbcdc_acm *a = (usbcdc_acm *) d;
			iprintf("\t\t*CDC ACM\n");
			iprintf("\t\t\tCapabilities: 0x%02X ", a->bmCapabilities);
			if (a->bmCapabilities & USB_CDC_ACM_CAP_COMM)
				iprintf("(COMM)");
			if (a->bmCapabilities & USB_CDC_ACM_CAP_LINE)
				iprintf("(LINE)");
			if (a->bmCapabilities & USB_CDC_ACM_CAP_BRK)
				iprintf("(BRK)");
			if (a->bmCapabilities & USB_CDC_ACM_CAP_NOTIFY)
				iprintf("(NOTIFY)");
			iprintf("\n");
			break;
		}
		case USB_CDC_SUBTYPE_ETHERNET: {
			usbcdc_ether *e = (usbcdc_ether *) d;
			iprintf("\t\t*CDC Ethernet\n");
			iprintf("\t\t\tMAC address: "); dumpString(e->iMacAddress);
			iprintf("\t\t\tStatistics: 0x%02lX\n", e->bmEthernetStatistics);
			iprintf("\t\t\tMax Segment Size: %d\n", e->wMaxSegmentSize);
			iprintf("\t\t\tMC Filters: %d\n", e->wNumberMCFilters);
			iprintf("\t\t\tPower Filters: %d\n", e->bNumberPowerFilters);
			break;
		}
	}
}

void USB::dumpDescriptors() {
	uint8_t i;
	for (i = 0; i < N_DESCRIPTORS; i++) {
		if (descriptors[i] == (usbdesc_base *) 0) {
			iprintf("--- FIN at %d\n", i);
			return;
		}
		iprintf("[%d:+%d]", i, descriptors[i]->bLength);
		switch (descriptors[i]->bDescType) {
			case DT_DEVICE: {
				dumpDevice((usbdesc_device *) descriptors[i]);
				break;
			}
			case DT_CONFIGURATION: {
				dumpConfiguration((usbdesc_configuration *) descriptors[i]);
				break;
			}
			case DT_INTERFACE: {
				dumpInterface((usbdesc_interface *) descriptors[i]);
				break;
			}
			case DT_ENDPOINT: {
				dumpEndpoint((usbdesc_endpoint *) descriptors[i]);
				break;
			}
			case DT_STRING: {
				dumpString((usbdesc_string *) descriptors[i]);
				break;
			}
			case DT_CDC_DESCRIPTOR: {
				dumpCDC((uint8_t *) descriptors[i]);
				break;
			}
		}
	}
}

void USB::on_module_loaded()
{
    register_for_event(ON_IDLE);
    connect();
}

void USB::on_idle(void*)
{
    USBHAL::usbisr();
}
