/* Title: mbed_interface
 * Functions to control the mbed interface
 *
 * mbed Microcontrollers have a built-in interface to provide functionality such as 
 * drag-n-drop download, reset, serial-over-usb, and access to the mbed local file 
 * system. These functions provide means to control the interface suing semihost
 * calls it supports.
 */
 
/* mbed Microcontroller Library - mbed_interface
 * Copyright (c) 2009-2011 ARM Limited. All rights reserved.
 */
 
#ifndef MBED_INTERFACE_H
#define MBED_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Function: mbed_interface_connected
 * Determine whether the mbed interface is connected, based on whether debug is enabled
 * 
 * Variables:
 *  returns - 1 if interface is connected, else 0
 */
int mbed_interface_connected(void);

/* Function: mbed_interface_reset
 * Instruct the mbed interface to reset, as if the reset button had been pressed
 *
 * Variables:
 *  returns - 1 if successful, else 0 (e.g. interface not present)
 */
int mbed_interface_reset(void);

/* Function: mbed_interface_disconnect
 * This will disconnect the debug aspect of the interface, so semihosting will be disabled.
 * The interface will still support the USB serial aspect
 *
 * Variables:
 *  returns - 0 if successful, else -1 (e.g. interface not present)
 */
int mbed_interface_disconnect(void);

/* Function: mbed_interface_powerdown
 * This will disconnect the debug aspect of the interface, and if the USB cable is not 
 * connected, also power down the interface. If the USB cable is connected, the interface
 * will remain powered up and visible to the host 
 *
 * Variables:
 *  returns - 0 if successful, else -1 (e.g. interface not present)
 */
int mbed_interface_powerdown(void);

/* Function: mbed_interface_uid
 * This returns a string containing the 32-character UID of the mbed interface
 *
 * This is a weak function that can be overwritten if required
 *
 * Variables:
 *  uid - A 33-byte array to write the null terminated 32-byte string
 *  returns - 0 if successful, else -1 (e.g. interface not present)
 */
int mbed_interface_uid(char *uid);

/* Function: mbed_mac_address
 * This returns a unique 6-byte MAC address, based on the interface UID
 *
 * If the interface is not present, it returns a default fixed MAC address (00:02:F7:F0:00:00)
 *
 * This is a weak function that can be overwritten if you want to provide your own mechanism to
 * provide a MAC address.

 * Variables:
 *  mac - A 6-byte array to write the MAC address
 */
void mbed_mac_address(char *mac);

/* Function: mbed_die
 * Cause the mbed to flash the BLOD LED sequence
 */
void mbed_die(void);

#ifdef __cplusplus
}
#endif

#endif
