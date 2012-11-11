/* USBDescriptor.h */
/* Definitions and macros for constructing USB descriptors */
/* Copyright (c) 2011 ARM Limited. All rights reserved. */

/* Standard descriptor types */
#define DEVICE_DESCRIPTOR        (1)
#define CONFIGURATION_DESCRIPTOR (2)
#define STRING_DESCRIPTOR        (3)
#define INTERFACE_DESCRIPTOR     (4)
#define ENDPOINT_DESCRIPTOR      (5)
#define QUALIFIER_DESCRIPTOR     (6)

/* Standard descriptor lengths */
#define DEVICE_DESCRIPTOR_LENGTH        (0x12)
#define CONFIGURATION_DESCRIPTOR_LENGTH (0x09)
#define INTERFACE_DESCRIPTOR_LENGTH     (0x09)
#define ENDPOINT_DESCRIPTOR_LENGTH      (0x07)


/*string offset*/
#define STRING_OFFSET_LANGID            (0)
#define STRING_OFFSET_IMANUFACTURER     (1)
#define STRING_OFFSET_IPRODUCT          (2)
#define STRING_OFFSET_ISERIAL           (3)
#define STRING_OFFSET_ICONFIGURATION    (4)
#define STRING_OFFSET_IINTERFACE        (5)

/* USB Specification Release Number */
#define USB_VERSION_2_0 (0x0200)

/* Least/Most significant byte of short integer */
#define LSB(n)  ((uint8_t)((n)&0xff))
#define MSB(n)  ((uint8_t)(((n)&0xff00)>>8))

/* Convert physical endpoint number to descriptor endpoint number */
#define PHY_TO_DESC(endpoint) (((endpoint)>>1) | (((endpoint) & 1) ? 0x80:0))

/* bmAttributes in configuration descriptor */
/* C_RESERVED must always be set */
#define C_RESERVED      (1U<<7)
#define C_SELF_POWERED  (1U<<6)
#define C_REMOTE_WAKEUP (1U<<5)

/* bMaxPower in configuration descriptor */
#define C_POWER(mA)     ((mA)/2)

/* bmAttributes in endpoint descriptor */
#define E_CONTROL       (0x00)
#define E_ISOCHRONOUS   (0x01)
#define E_BULK          (0x02)
#define E_INTERRUPT     (0x03)

/* For isochronous endpoints only: */
#define E_NO_SYNCHRONIZATION    (0x00)
#define E_ASYNCHRONOUS          (0x04)
#define E_ADAPTIVE              (0x08)
#define E_SYNCHRONOUS           (0x0C)
#define E_DATA                  (0x00)
#define E_FEEDBACK              (0x10)
#define E_IMPLICIT_FEEDBACK     (0x20)
