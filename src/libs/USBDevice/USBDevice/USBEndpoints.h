/* USBEndpoints.h */
/* USB endpoint configuration */
/* Copyright (c) 2011 ARM Limited. All rights reserved. */

#ifndef USBENDPOINTS_H
#define USBENDPOINTS_H

/* SETUP packet size */
#define SETUP_PACKET_SIZE (8)

/* Options flags for configuring endpoints */
#define DEFAULT_OPTIONS     (0)
#define SINGLE_BUFFERED     (1U << 0)
#define ISOCHRONOUS         (1U << 1)
#define RATE_FEEDBACK_MODE  (1U << 2) /* Interrupt endpoints only */

/* Endpoint transfer status, for endpoints > 0 */
typedef enum {
    EP_COMPLETED,   /* Transfer completed */
    EP_PENDING,     /* Transfer in progress */
    EP_INVALID,     /* Invalid parameter */
    EP_STALLED,     /* Endpoint stalled */
} EP_STATUS;

/* Include configuration for specific target */
#if defined(TARGET_LPC1768) || defined(TARGET_LPC2368)
#include "USBEndpoints_LPC17_LPC23.h"
#elif defined(TARGET_LPC11U24)
#include "USBEndpoints_LPC11U.h"
#else
#error "Unknown target type"
#endif

#endif
