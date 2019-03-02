#ifndef _DESCRIPTOR_CDC_H
#define _DESCRIPTOR_CDC_H

#define	DT_CDC_DESCRIPTOR                   0x24
#define	DT_CDC_ENDPOINT                     0x25

#define USB_CDC_SUBCLASS_DLC                0x01
#define USB_CDC_SUBCLASS_ACM                0x02
#define USB_CDC_SUBCLASS_TCM                0x03
#define USB_CDC_SUBCLASS_MCCM               0x04
#define USB_CDC_SUBCLASS_CAPI               0x05
#define USB_CDC_SUBCLASS_ETHERNET           0x06
#define USB_CDC_SUBCLASS_ATM                0x07
#define USB_CDC_SUBCLASS_WHCM               0x08
#define USB_CDC_SUBCLASS_DMM                0x09
#define USB_CDC_SUBCLASS_MDLM               0x0a
#define USB_CDC_SUBCLASS_OBEX               0x0b
#define USB_CDC_SUBCLASS_EEM                0x0c
#define USB_CDC_SUBCLASS_NCM                0x0d

#define USB_CDC_SUBTYPE_HEADER              0x00
#define USB_CDC_SUBTYPE_CALL_MANAGEMENT     0x01
#define USB_CDC_SUBTYPE_ACM                 0x02
#define USB_CDC_SUBTYPE_DLM                 0x03        /* direct line management */
#define USB_CDC_SUBTYPE_TRF                 0x04        /* telephone ringer */
#define USB_CDC_SUBTYPE_TCLSRCF             0x05        /* telephone call and line state reporting capabilities */
#define USB_CDC_SUBTYPE_UNION               0x06
#define USB_CDC_SUBTYPE_COUNTRY             0x07
#define USB_CDC_SUBTYPE_TOMF                0x08        /* telephone operational modes */
#define USB_CDC_SUBTYPE_USBTF               0x09        /* USB terminal */
#define USB_CDC_SUBTYPE_NETWORK_TERMINAL    0x0A
#define USB_CDC_SUBTYPE_PUF                 0x0B        /* protocol unit */
#define USB_CDC_SUBTYPE_EUF                 0x0C        /* extension unit */
#define USB_CDC_SUBTYPE_MCMF                0x0D        /* multi-channel management */
#define USB_CDC_SUBTYPE_CAPI_CM             0x0E        /* CAPI control management */
#define USB_CDC_SUBTYPE_ETHERNET            0x0F
#define USB_CDC_SUBTYPE_ATM_NETWORK         0x10
#define USB_CDC_SUBTYPE_WHCM                0x11        /* wireless handset control model */
#define USB_CDC_SUBTYPE_MDLM                0x12        /* mobile direct line model */
#define USB_CDC_SUBTYPE_MDLM_DETAIL         0x13        /* MDLM detail */
#define USB_CDC_SUBTYPE_DMM                 0x14        /* device management model */
#define USB_CDC_SUBTYPE_OBEX                0x15
#define USB_CDC_SUBTYPE_CSF                 0x16        /* command set */
#define USB_CDC_SUBTYPE_CSDF                0x17        /* command set detail */
#define USB_CDC_SUBTYPE_TCMF                0x18        /* telephone control model */
#define USB_CDC_SUBTYPE_OBEX_SI             0x19        /* OBEX service identifier */
#define USB_CDC_SUBTYPE_NCM                 0x1A

#define USB_CDC_PROTOCOL_NONE               0x00
#define USB_CDC_PROTOCOL_ITU_V250           0x01
#define USB_CDC_PROTOCOL_PCCA_101           0x02
#define USB_CDC_PROTOCOL_PCCA_101_O         0x03
#define USB_CDC_PROTOCOL_GSM_7_07           0x04
#define USB_CDC_PROTOCOL_3GPP_27_07         0x05
#define USB_CDC_PROTOCOL_C_S0017_0          0x06
#define USB_CDC_PROTOCOL_EEM                0x07
#define USB_CDC_PROTOCOL_SEE_DESCRIPTOR     0xFE
#define USB_CDC_PROTOCOL_VENDOR             0xFF

#define CDC_SET_LINE_CODING                 0x20
#define CDC_GET_LINE_CODING                 0x21
#define CDC_SET_CONTROL_LINE_STATE          0x22
#define CDC_SEND_BREAK                      0x23

// control line states
#define CDC_CLS_DTR                         0x01
#define CDC_CLS_RTS                         0x02

typedef struct __attribute__ ((packed)) {
	uint8_t	bLength;      // 5
	uint8_t	bDescType;    // DT_CDC_DESCRIPTOR      (0x24)
	uint8_t	bDescSubType; // USB_CDC_SUBTYPE_HEADER (0x00)
	uint16_t	bcdCDC;
} usbcdc_header;
#define	USB_CDC_LENGTH_HEADER sizeof(usbcdc_header)

typedef struct __attribute__ ((packed)) {
	uint8_t	bLength;      // 5
	uint8_t	bDescType;    // DT_CDC_DESCRIPTOR      (0x24)
	uint8_t	bDescSubType; // USB_CDC_SUBTYPE_CALL_MANAGEMENT (0x01)

	uint8_t	bmCapabilities;
#define	USB_CDC_CALLMGMT_CAP_CALLMGMT	0x01
#define USB_CDC_CALLMGMT_CAP_DATAINTF	0x02

	uint8_t	bDataInterface;
} usbcdc_callmgmt;
#define	USB_CDC_LENGTH_CALLMGMT sizeof(usbcdc_callmgmt)

typedef struct __attribute__ ((packed)) {
	uint8_t	bLength;      // 4
	uint8_t	bDescType;    // DT_CDC_DESCRIPTOR      (0x24)
	uint8_t	bDescSubType; // USB_CDC_SUBTYPE_ACM    (0x02)

	uint8_t	bmCapabilities;
#define	USB_CDC_ACM_CAP_COMM	0x01
#define USB_CDC_ACM_CAP_LINE	0x02
#define	USB_CDC_ACM_CAP_BRK		0x04
#define USB_CDC_ACM_CAP_NOTIFY	0x08
} usbcdc_acm;
#define	USB_CDC_LENGTH_ACM sizeof(usbcdc_acm)

typedef struct __attribute__ ((packed)) {
	uint8_t	bLength;      // 5+
	uint8_t	bDescType;    // DT_CDC_DESCRIPTOR      (0x24)
	uint8_t	bDescSubType; // USB_CDC_SUBTYPE_UNION  (0x06)

	uint8_t	bMasterInterface;
	uint8_t	bSlaveInterface0;
} usbcdc_union;
#define	USB_CDC_LENGTH_UNION sizeof(usbcdc_union)

typedef struct __attribute__ ((packed)) {
	uint8_t	bLength;							// 13
	uint8_t	bDescType;						// DT_CDC_DESCRIPTOR      (0x24)
	uint8_t	bDescSubType; 				// USB_CDC_SUBTYPE_ETHERNET (0x0F)

	uint8_t	iMacAddress;					// index of MAC address string
	uint32_t	bmEthernetStatistics;
	uint16_t	wMaxSegmentSize;			// 1514?
	uint16_t	wNumberMCFilters;			// 0
	uint8_t	bNumberPowerFilters;	// 0
} usbcdc_ether;
#define USB_CDC_LENGTH_ETHER sizeof(usbcdc_ether)

typedef struct __attribute__ ((packed)) {
    uint32_t    dwDTERate;              // data terminal rate, bits per second
    uint8_t     bCharFormat;            // 0: 1 stop bit, 1: 1.5 stop bits, 2: 2 stop bits
    uint8_t     bParityType;            // 0: none, 1: odd, 2: even, 3: mark, 4: space
    uint8_t     bDataBits;              // number of data bits (5, 6, 7, 8, 16)
} usbcdc_line_coding;

#endif /* _DESCRIPTOR_CDC_H */
