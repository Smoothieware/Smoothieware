/* mbed Microcontroller Library - can
 * Copyright (c) 2009-2011 ARM Limited. All rights reserved.
 */ 

#ifndef MBED_CAN_H
#define MBED_CAN_H

#include "device.h"

#if DEVICE_CAN

#include "Base.h"
#include "platform.h" 
#include "PinNames.h"
#include "PeripheralNames.h"

#include "can_helper.h" 
#include "FunctionPointer.h"

#include <string.h>

namespace mbed {

/* Class: CANMessage
 * 
 */
class CANMessage : public CAN_Message {

public:

    /* Constructor: CANMessage
     *  Creates empty CAN message.
     */
    CANMessage() {
      len    = 8;
      type   = CANData;
      format = CANStandard;
      id     = 0;
      memset(data, 0, 8);
    }
    
    /* Constructor: CANMessage
     *  Creates CAN message with specific content.
     */
    CANMessage(int _id, const char *_data, char _len = 8, CANType _type = CANData, CANFormat _format = CANStandard) {
      len    = _len & 0xF;
      type   = _type;
      format = _format;
      id     = _id;
      memcpy(data, _data, _len);
    }

    /* Constructor: CANMessage
     *  Creates CAN remote message.
     */
    CANMessage(int _id, CANFormat _format = CANStandard) {
      len    = 0;
      type   = CANRemote;
      format = _format;
      id     = _id;
      memset(data, 0, 8);
    }
#if 0 // Inhereted from CAN_Message, for documentation only

    /* Variable: id
     *  The message id.
     *
     * If format is CANStandard it must be an 11 bit long id
     * If format is CANExtended it must be an 29 bit long id
     */
    unsigned int   id;
    
    /* Variable: data
     *  Space for 8 byte payload.
     *
     * If type is CANData data can store up to 8 byte data.
     */
    unsigned char  data[8];
    
    /* Variable: len
     *  Length of data in bytes.
     *
     * If type is CANData data can store up to 8 byte data.
     */
    unsigned char  len;
    
    /* Variable: format
     *  Defines if the message has standard or extended format.
     *
     * Defines the type of message id:
     * Default is CANStandard which implies 11 bit id.
     * CANExtended means 29 bit message id.
     */
    CANFormat      format;
    
    /* Variable: type
     *  Defines the type of a message.
     *
     * The message type can rather be CANData for a message with data (default).
     * Or CANRemote for a request of a specific CAN message.
     */
    CANType        type;               // 0 - DATA FRAME, 1 - REMOTE FRAME
#endif
};

/* Class: CAN
 *  A can bus client, used for communicating with can devices
 */
class CAN : public Base {

public:

    /* Constructor: CAN
     *  Creates an CAN interface connected to specific pins.
     *
     * Example:
     * > #include "mbed.h"
     * > 
     * > Ticker ticker;
     * > DigitalOut led1(LED1);
     * > DigitalOut led2(LED2);
     * > CAN can1(p9, p10);
     * > CAN can2(p30, p29);
     * > 
     * > char counter = 0;
     * > 
     * > void send() {
     * >     if(can1.write(CANMessage(1337, &counter, 1))) {
     * >         printf("Message sent: %d\n", counter);
     * >         counter++;
     * >     } 
     * >     led1 = !led1;
     * > }
     * > 
     * > int main() {
     * >     ticker.attach(&send, 1);
     * >    CANMessage msg;
     * >     while(1) {
     * >         if(can2.read(msg)) {
     * >             printf("Message received: %d\n\n", msg.data[0]);
     * >             led2 = !led2;
     * >         } 
     * >         wait(0.2);
     * >     }
     * > } 
     *
     * Variables:
     *  rd - read from transmitter
     *  td - transmit to transmitter
     */
    CAN(PinName rd, PinName td);
    virtual ~CAN();
    
    /* Function: frequency
     *  Set the frequency of the CAN interface
     *
     * Variables:
     *  hz - The bus frequency in hertz
     *  returns - 1 if successful, 0 otherwise
     */
    int frequency(int hz);
    
    /* Function: write
     *  Write a CANMessage to the bus.
     *
     * Variables:
     *  msg - The CANMessage to write.
     *
     * Returns:
     *  0 - If write failed.
     *  1 - If write was successful.
     */
    int write(CANMessage msg);
    
    /* Function: read
     *  Read a CANMessage from the bus.
     * 
     * Variables:
     *  msg - A CANMessage to read to.
     *
     * Returns:
     *  0 - If no message arrived.
     *  1 - If message arrived.
     */
    int read(CANMessage &msg);
    
    /* Function: reset
     *  Reset CAN interface.
     *
     * To use after error overflow.
     */
    void reset();

    /* Function: monitor
     *  Puts or removes the CAN interface into silent monitoring mode
     *
     * Variables:
     *  silent - boolean indicating whether to go into silent mode or not
     */
    void monitor(bool silent);
    
    /* Function: rderror
     *  Returns number of read errors to detect read overflow errors.
     */
    unsigned char rderror();

    /* Function: tderror
     *  Returns number of write errors to detect write overflow errors.
     */
    unsigned char tderror();

    /* Function: attach
     *  Attach a function to call whenever a CAN frame received interrupt is
     *  generated.
     *
     * Variables:
     *  fptr - A pointer to a void function, or 0 to set as none
     */
    void attach(void (*fptr)(void));
   
   /* Function attach
    *  Attach a member function to call whenever a CAN frame received interrupt
    *  is generated.
    *
    * Variables:
    *  tptr - pointer to the object to call the member function on
    *  mptr - pointer to the member function to be called
    */
   template<typename T>
   void attach(T* tptr, void (T::*mptr)(void)) {
        if((mptr != NULL) && (tptr != NULL)) {
            _rxirq.attach(tptr, mptr);
            setup_interrupt();
        } else {
            remove_interrupt();
        }
    }
    
private:

    CANName _id;
    FunctionPointer _rxirq;

    void setup_interrupt(void);
    void remove_interrupt(void);
};

} // namespace mbed

#endif    // MBED_CAN_H

#endif
