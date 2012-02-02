/* USBEndpoints_LPC17_LPC23.h */
/* Endpoint configuration for LPC1768 and LPC2368 */
/* Copyright (c) 2011 ARM Limited. All rights reserved. */

#define NUMBER_OF_LOGICAL_ENDPOINTS (16)
#define NUMBER_OF_PHYSICAL_ENDPOINTS (NUMBER_OF_LOGICAL_ENDPOINTS * 2)

/* Define physical endpoint numbers */

/*      Endpoint    No.     Type(s)       MaxPacket   DoubleBuffer  */
/*      ----------------    ------------  ----------  ---           */
#define EP0OUT      (0)  /* Control       64          No            */
#define EP0IN       (1)  /* Control       64          No            */
#define EP1OUT      (2)  /* Interrupt     64          No            */
#define EP1IN       (3)  /* Interrupt     64          No            */
#define EP2OUT      (4)  /* Bulk          64          Yes           */
#define EP2IN       (5)  /* Bulk          64          Yes           */
#define EP3OUT      (6)  /* Isochronous   1023        Yes           */
#define EP3IN       (7)  /* Isochronous   1023        Yes           */
#define EP4OUT      (8)  /* Interrupt     64          No            */
#define EP4IN       (9)  /* Interrupt     64          No            */
#define EP5OUT      (10) /* Bulk          64          Yes           */
#define EP5IN       (11) /* Bulk          64          Yes           */
#define EP6OUT      (12) /* Isochronous   1023        Yes           */
#define EP6IN       (13) /* Isochronous   1023        Yes           */
#define EP7OUT      (14) /* Interrupt     64          No            */
#define EP7IN       (15) /* Interrupt     64          No            */
#define EP8OUT      (16) /* Bulk          64          Yes           */
#define EP8IN       (17) /* Bulk          64          Yes           */
#define EP9OUT      (18) /* Isochronous   1023        Yes           */
#define EP9IN       (19) /* Isochronous   1023        Yes           */
#define EP10OUT     (20) /* Interrupt     64          No            */
#define EP10IN      (21) /* Interrupt     64          No            */
#define EP11OUT     (22) /* Bulk          64          Yes           */
#define EP11IN      (23) /* Bulk          64          Yes           */
#define EP12OUT     (24) /* Isochronous   1023        Yes           */
#define EP12IN      (25) /* Isochronous   1023        Yes           */
#define EP13OUT     (26) /* Interrupt     64          No            */
#define EP13IN      (27) /* Interrupt     64          No            */
#define EP14OUT     (28) /* Bulk          64          Yes           */
#define EP14IN      (29) /* Bulk          64          Yes           */
#define EP15OUT     (30) /* Bulk          64          Yes           */
#define EP15IN      (31) /* Bulk          64          Yes           */

/* Maximum Packet sizes */

#define MAX_PACKET_SIZE_EP0  (64)
#define MAX_PACKET_SIZE_EP1  (64)
#define MAX_PACKET_SIZE_EP2  (64)
#define MAX_PACKET_SIZE_EP3  (1023)
#define MAX_PACKET_SIZE_EP4  (64)
#define MAX_PACKET_SIZE_EP5  (64)
#define MAX_PACKET_SIZE_EP6  (1023)
#define MAX_PACKET_SIZE_EP7  (64)
#define MAX_PACKET_SIZE_EP8  (64)
#define MAX_PACKET_SIZE_EP9  (1023)
#define MAX_PACKET_SIZE_EP10 (64)
#define MAX_PACKET_SIZE_EP11 (64)
#define MAX_PACKET_SIZE_EP12 (1023)
#define MAX_PACKET_SIZE_EP13 (64)
#define MAX_PACKET_SIZE_EP14 (64)
#define MAX_PACKET_SIZE_EP15 (64)

/* Generic endpoints - intended to be portable accross devices */
/* and be suitable for simple USB devices. */

/* Bulk endpoints */
#define EPBULK_OUT  (EP2OUT)
#define EPBULK_IN   (EP2IN)
/* Interrupt endpoints */
#define EPINT_OUT   (EP1OUT)
#define EPINT_IN    (EP1IN)
/* Isochronous endpoints */
#define EPISO_OUT   (EP3OUT)
#define EPISO_IN    (EP3IN)

#define MAX_PACKET_SIZE_EPBULK  (MAX_PACKET_SIZE_EP2)
#define MAX_PACKET_SIZE_EPINT   (MAX_PACKET_SIZE_EP1)
#define MAX_PACKET_SIZE_EPISO   (MAX_PACKET_SIZE_EP3)

/* Bulk endpoints */
#define MSDBULK_OUT  (EP5OUT)
#define MSDBULK_IN   (EP5IN)
#define MAX_PACKET_SIZE_MSDBULK  (MAX_PACKET_SIZE_EP5)
