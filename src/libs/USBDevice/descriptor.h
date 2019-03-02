#ifndef _DESCRIPTOR_H
#define _DESCRIPTOR_H

#include <stdint.h>

#define DL_DEVICE                   0x12
#define DL_CONFIGURATION            0x09
#define DL_INTERFACE                0x09
#define DL_ENDPOINT                 0x07
#define DL_LANGUAGE                 0x04
#define DL_INTERFACE_ASSOCIATION    0x08

#define mA                          /2

#define DT_DEVICE                   0x01
#define DT_CONFIGURATION            0x02
#define DT_STRING                   0x03
#define DT_LANGUAGE                 0x03
#define DT_INTERFACE                0x04
#define DT_ENDPOINT                 0x05
#define DT_DEVICE_QUALIFIER         0x06
#define DT_OTHER_SPEED              0x07
#define DT_INTERFACE_POWER          0x08
#define DT_OTG                      0x09
#define DT_DEBUG                    0x0A
#define DT_INTERFACE_ASSOCIATION    0x0B

#define USB_VERSION_1_0             0x0100
#define USB_VERSION_1_1             0x0110
#define USB_VERSION_2_0             0x0200
#define USB_VERSION_3_0             0x0300

#define UC_PER_INTERFACE            0x00
#define UC_AUDIO                    0x01
#define UC_COMM                     0x02
#define UC_HID                      0x03
#define UC_PHYSICAL                 0x05
#define UC_STILL_IMAGE              0x06
#define UC_PRINTER                  0x07
#define UC_MASS_STORAGE             0x08
#define UC_HUB                      0x09
#define UC_CDC_DATA                 0x0A
#define UC_CSCID                    0x0B
#define UC_CONTENT_SEC              0x0D
#define UC_VIDEO                    0x0E
#define UC_WIRELESS_CONTROLLER      0xE0
#define UC_MISC                     0xEF
#define UC_APP_SPEC                 0xFE
#define UC_VENDOR_SPEC              0xFF

#define SUBCLASS_IAD                0x02
#define PROTOCOL_IAD                0x01

#define CA_BUSPOWERED               0x80
#define CA_SELFPOWERED              0x40
#define CA_REMOTEWAKEUP             0x20

#define EP_DIR_MASK                 0x80
#define EP_DIR_OUT                  0x00
#define EP_DIR_IN                   0x80

#define EA_CONTROL                  0x00
#define EA_ISOCHRONOUS              0x01
#define EA_BULK                     0x02
#define EA_INTERRUPT                0x03

#define EA_ISO_NONE                 0x00
#define EA_ISO_ASYNC                0x04
#define EA_ISO_ADAPTIVE             0x08
#define EA_ISO_SYNC                 0x0C

#define EA_ISO_TYPE_DATA            0x00
#define EA_ISO_TYPE_FEEDBACK        0x10
#define EA_ISO_TYPE_EXPLICIT        0x20

#define SL_USENGLISH                0x0409
#define SL_AUENGLISH                0x0C09
#define SL_GERMAN                   0x0407

#include "USBEndpoints.h"

typedef struct __attribute__ ((packed))
{
    uint8_t         bLength;                // descriptor length
    uint8_t         bDescType;              // descriptor type: see DT_* defines
}
    usbdesc_base;

typedef struct __attribute__ ((packed))
{
    uint8_t         bLength;                // Device descriptor length (0x12)
    uint8_t         bDescType;              // DT_DEVICE (0x01)
    uint16_t        bcdUSB;                 // USB Specification Number which device complies to - see USB_VERSION_* defines
    uint8_t         bDeviceClass;           // USB Device Class - see UC_* defines
    uint8_t         bDeviceSubClass;        // Subclass Code
    uint8_t         bDeviceProtocol;        // Protocol Code
    uint8_t         bMaxPacketSize;         // Maximum Packet Size for Zero Endpoint. Valid Sizes are 8, 16, 32, 64
    uint16_t        idVendor;               // Vendor ID
    uint16_t        idProduct;              // Product ID
    uint16_t        bcdDevice;              // Device Release Number
    uint8_t         iManufacturer;          // Index of Manufacturer String Descriptor
    uint8_t         iProduct;               // Index of Product String Descriptor
    uint8_t         iSerialNumber;          // Index of Serial Number String Descriptor
    uint8_t         bNumConfigurations;     // Number of Possible Configurations
}
    usbdesc_device;

typedef struct __attribute__ ((packed))
{
    uint8_t         bLength;                // Configuration Descriptor Length (0x09)
    uint8_t         bDescType;              // DT_CONFIGURATION (0x02)
    uint16_t        wTotalLength;           // Total length in bytes of this descriptor plus all this configuration's interfaces plus their endpoints, see http://www.beyondlogic.org/usbnutshell/confsize.gif
    uint8_t         bNumInterfaces;         // Number of Interfaces
    uint8_t         bConfigurationValue;    // Value that host uses to select this configuration
    uint8_t         iConfiguration;         // Index of String Descriptor describing this configuration
    uint8_t         bmAttributes;           // bitmap. see CA_* defines
    uint8_t         bMaxPower;              // Max. Current = bMaxPower * 2mA
}
    usbdesc_configuration;

typedef struct __attribute__ ((packed))
{
    uint8_t         bLength;                // Interface Descriptor Length (0x09)
    uint8_t         bDescType;              // DT_INTERFACE (0x04)
    uint8_t         bInterfaceNumber;       // Number of Interface
    uint8_t         bAlternateSetting;      // Value used to select alternative setting

    uint8_t         bNumEndPoints;          // Number of Endpoints used for this interface
    uint8_t         bInterfaceClass;        // Class Code - see Device_Class_Enum
    uint8_t         bInterfaceSubClass;     // Subclass Code
    uint8_t         bInterfaceProtocol;     // Protocol Code

    uint8_t         iInterface;             // Index of String Descriptor Describing this interface
    uint8_t         selectedAlternate;      // the currently selected alternate for this group of interfaces. Set on alternate 0 for easy lookups
    uint8_t         dummy1;                 // pad to 32 bit boundary
    uint8_t         dummy2;                 // pad to 32 bit boundary

    USB_Class_Receiver *classReceiver;      // who do we call when we receive a setup packet for this interface?
}
    usbdesc_interface;

typedef struct __attribute__ ((packed))
{
    uint8_t         bLength;                // Endpoint Descriptor Length (0x07)
    uint8_t         bDescType;              // DT_ENDPOINT (0x05)
    uint8_t         bEndpointAddress;       // 0x00-0x0F = OUT endpoints, 0x80-0x8F = IN endpoints
    uint8_t         bmAttributes;           // bitmap, see Endpoint_Attributes_Enum

    uint16_t        wMaxPacketSize;         // Maximum Packet Size this endpoint is capable of sending or receiving
    uint8_t         bInterval;              // Interval for polling endpoint data transfers. Value in frame counts. Ignored for Bulk & Control Endpoints. Isochronous must equal 1 and field may range from 1 to 255 for interrupt endpoints.
    uint8_t         dummy;                  // pad to 32 bit boundary

    USB_Endpoint_Receiver *epReceiver;      // Who do we call when something happens on this endpoint?
}
    usbdesc_endpoint;

typedef struct __attribute__ ((packed))
{
    uint8_t         bLength;                // String Descriptor Length (2 + 2*nLang)
    uint8_t         bDescType;              // DT_STRING (0x03)
    uint16_t        wLangID[1];             // language code(s)
}
    usbdesc_language;

typedef struct __attribute__ ((packed))
{
    uint8_t         bLength;                // 2 + strlen
    uint8_t         bDescType;              // DT_STRING (0x03)
    uint16_t        str[];                  // UNICODE string
}
    usbdesc_string;

#define usbdesc_string_l(l) struct __attribute__ ((packed)) { uint8_t bLength; uint8_t bDescType; char16_t str[l]; }

#define usbstring(string) { sizeof(u ## string), DT_STRING, { u ## string } }

#define usbstring_init(name, string) struct __attribute__ ((packed)) { const uint8_t bLength; const uint8_t bDescType; char16_t str[sizeof(string)]; } name = { sizeof(u ## string), DT_STRING, { u ## string } }

#define usbstring_const_init(name, string) const struct __attribute__ ((packed)) { const uint8_t bLength; const uint8_t bDescType; const char16_t str[sizeof(string)]; } name = { sizeof(u ## string), DT_STRING, { u ## string } }

typedef struct __attribute__ ((packed))
{
    uint8_t         bLength;
    uint8_t         bDescType;
    uint8_t         bFirstInterface;        // interface index of the first interface for this function
    uint8_t         bInterfaceCount;        // number of contiguous interfaces used with this function
    uint8_t         bFunctionClass;         // Class code - see Device descriptor
    uint8_t         bFunctionSubClass;      // Subclass code
    uint8_t         bFunctionProtocol;      // Protocol code
    uint8_t         iFunction;              // function name string
}
    usbdesc_iad;

#endif /* _DESCRIPTOR_H */
