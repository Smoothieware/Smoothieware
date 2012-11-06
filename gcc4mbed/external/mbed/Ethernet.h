/* mbed Microcontroller Library - Ethernet
 * Copyright (c) 2009-2011 ARM Limited. All rights reserved.
 */ 
 
#ifndef MBED_ETHERNET_H
#define MBED_ETHERNET_H

#include "device.h"

#if DEVICE_ETHERNET

#include "Base.h"

namespace mbed {

/* Class: Ethernet
 *  An ethernet interface, to use with the ethernet pins.
 *
 * Example:
 * > // Read destination and source from every ethernet packet
 * >
 * > #include "mbed.h"
 * >
 * > Ethernet eth;
 * > 
 * > int main() {
 * >     char buf[0x600];
 * >     
 * >     while(1) {
 * >         int size = eth.receive();
 * >         if(size > 0) {
 * >             eth.read(buf, size);
 * >             printf("Destination:  %02X:%02X:%02X:%02X:%02X:%02X\n",
 * >                     buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
 * >             printf("Source: %02X:%02X:%02X:%02X:%02X:%02X\n",
 * >                     buf[6], buf[7], buf[8], buf[9], buf[10], buf[11]);
 * >         }
 * >         
 * >         wait(1);
 * >     }
 * > }
 *
 */
class Ethernet : public Base {

public:
    
    /* Constructor: Ethernet
     *  Initialise the ethernet interface.
     */
    Ethernet();

    /* Destructor: Ethernet
     *  Powers the hardware down.
     */
    virtual ~Ethernet();

    enum Mode {
        AutoNegotiate
        , HalfDuplex10
        , FullDuplex10
        , HalfDuplex100
        , FullDuplex100
    };

    /* Function: write
     *  Writes into an outgoing ethernet packet.
     *
     *  It will append size bytes of data to the previously written bytes.
     *  
     *  Variables:
     *   data - An array to write.
     *   size - The size of data.
     *
     *  Returns:
     *   The number of written bytes.
     */
    int write(const char *data, int size);

    /* Function: send
     *  Send an outgoing ethernet packet.
     *
     *  After filling in the data in an ethernet packet it must be send.
     *  Send will provide a new packet to write to.
     *
     * Returns:
     *  0 - If the sending was failed.
     *  1 - If the package is successfully sent.
     */
    int send();

    /* Function: receive
     *  Recevies an arrived ethernet packet.
     *
     *  Receiving an ethernet packet will drop the last received ethernet packet 
     *  and make a new ethernet packet ready to read.
     *  If no ethernet packet is arrived it will return 0.
     *
     * Returns:
     *  0 - If no ethernet packet is arrived.
     *  The size of the arrived packet.
     */
    int receive();

    /* Function: read
     *  Read from an recevied ethernet packet.
     *
     *  After receive returnd a number bigger than 0it is
     *  possible to read bytes from this packet.
     *  Read will write up to size bytes into data.
     *
     *  It is possible to use read multible times. 
     *  Each time read will start reading after the last read byte before.
     *
     * Returns:
     *  The number of byte read.
     */
    int read(char *data, int size);
    
    /* Function: address
     *  Gives the ethernet address of the mbed.
     *
     * Variables:
     *  mac - Must be a pointer to a 6 byte char array to copy the ethernet address in.
     */
    void address(char *mac);

    /* Function: link
     *  Returns if an ethernet link is pressent or not. It takes a wile after Ethernet initializion to show up.
     * 
     * Returns:
     *  0 - If no ethernet link is pressent.
     *  1 - If an ethernet link is pressent.
     *
     * Example:
     * > // Using the Ethernet link function
     * > #include "mbed.h"
     * >
     * > Ethernet eth;
     * >
     * > int main() {
     * >     wait(1); // Needed after startup.
     * >     if(eth.link()) {
     * >         printf("online\n");
     * >     } else {
     * >          printf("offline\n");
     * >     }
     * > }
     *
     */
    int link();

    /* Function: set_link
     *  Sets the speed and duplex parameters of an ethernet link
     *
     *  Variables:
     *   mode - the speed and duplex mode to set the link to:
     *
     * > AutoNegotiate      Auto negotiate speed and duplex
     * > HalfDuplex10       10 Mbit, half duplex
     * > FullDuplex10       10 Mbit, full duplex
     * > HalfDuplex100      100 Mbit, half duplex
     * > FullDuplex100      100 Mbit, full duplex
     */
    void set_link(Mode mode);

};

} // namespace mbed

#endif

#endif
