/* USBEndpoints_LPC11U.h */
/* Endpoint configuration for LPC11U */
/* Copyright (c) 2011 ARM Limited. All rights reserved. */

#define NUMBER_OF_LOGICAL_ENDPOINTS (5)
#define NUMBER_OF_PHYSICAL_ENDPOINTS (NUMBER_OF_LOGICAL_ENDPOINTS * 2)

/* Define physical endpoint numbers */

/*      Endpoint    No.     Type(s)       MaxPacket   DoubleBuffer  */
/*      ----------------    ------------  ----------  ---           */
#define EP0OUT      (0)  /* Control       64          No            */
#define EP0IN       (1)  /* Control       64          No            */
#define EP1OUT      (2)  /* Int/Bulk/Iso  64/64/1023  Yes           */
#define EP1IN       (3)  /* Int/Bulk/Iso  64/64/1023  Yes           */
#define EP2OUT      (4)  /* Int/Bulk/Iso  64/64/1023  Yes           */
#define EP2IN       (5)  /* Int/Bulk/Iso  64/64/1023  Yes           */
#define EP3OUT      (6)  /* Int/Bulk/Iso  64/64/1023  Yes           */
#define EP3IN       (7)  /* Int/Bulk/Iso  64/64/1023  Yes           */
#define EP4OUT      (8)  /* Int/Bulk/Iso  64/64/1023  Yes           */
#define EP4IN       (9)  /* Int/Bulk/Iso  64/64/1023  Yes           */

/* Maximum Packet sizes */

#define MAX_PACKET_SIZE_EP0 (64)
#define MAX_PACKET_SIZE_EP1 (64) /* Int/Bulk */
#define MAX_PACKET_SIZE_EP2 (64) /* Int/Bulk */
#define MAX_PACKET_SIZE_EP3 (64) /* Int/Bulk */
#define MAX_PACKET_SIZE_EP4 (64) /* Int/Bulk */

#define MAX_PACKET_SIZE_EP1_ISO (1023) /* Isochronous */
#define MAX_PACKET_SIZE_EP2_ISO (1023) /* Isochronous */
#define MAX_PACKET_SIZE_EP3_ISO (1023) /* Isochronous */
#define MAX_PACKET_SIZE_EP4_ISO (1023) /* Isochronous */

/* Generic endpoints - intended to be portable accross devices */
/* and be suitable for simple USB devices. */

/* Bulk endpoint */
#define EPBULK_OUT  (EP2OUT)
#define EPBULK_IN   (EP2IN)
/* Interrupt endpoint */
#define EPINT_OUT   (EP1OUT)
#define EPINT_IN    (EP1IN)
/* Isochronous endpoint */
#define EPISO_OUT   (EP3OUT)
#define EPISO_IN    (EP3IN)

#define MAX_PACKET_SIZE_EPBULK  (MAX_PACKET_SIZE_EP2)
#define MAX_PACKET_SIZE_EPINT   (MAX_PACKET_SIZE_EP1)
#define MAX_PACKET_SIZE_EPISO   (MAX_PACKET_SIZE_EP3_ISO)
