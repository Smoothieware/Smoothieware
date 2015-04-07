/* Copyright (c) 2010-2011 mbed.org, MIT License
*
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
* and associated documentation files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all copies or
* substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
* BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef USBBUSINTERFACE_H
#define USBBUSINTERFACE_H

#include <stdint.h>

// #include "mbed.h"
#include "USBEndpoints.h"

class USBHAL : public USB_State_Receiver, public USB_Frame_Receiver, public USB_Endpoint_Receiver {
public:
    /* Configuration */
    USBHAL();
    ~USBHAL();

    void init(void);

    void connect(void);
    void disconnect(void);
    void configureDevice(void);
    void unconfigureDevice(void);
    void setAddress(uint8_t address);
    void remoteWakeup(void);

    /* Endpoint 0 */
    void EP0setup(uint8_t *buffer);
    void EP0read(void);
    uint32_t EP0getReadResult(uint8_t *buffer);
    void EP0write(uint8_t *buffer, uint32_t size);
    void EP0getWriteResult(void);
    void EP0stall(void);

    /* Other endpoints */
    EP_STATUS endpointRead(uint8_t bEP, uint32_t maximumSize);
    EP_STATUS endpointReadResult(uint8_t bEP, uint8_t *data, uint32_t *bytesRead);
    EP_STATUS endpointWrite(uint8_t bEP, uint8_t *data, uint32_t size);
    EP_STATUS endpointWriteResult(uint8_t bEP);
    uint8_t endpointStatus(uint8_t bEP);
    void stallEndpoint(uint8_t bEP);
    void unstallEndpoint(uint8_t bEP);
    bool realiseEndpoint(uint8_t bEP, uint32_t maxPacket, uint32_t options);
    bool getEndpointStallState(uint8_t bEP);
    uint32_t endpointReadcore(uint8_t bEP, uint8_t *buffer);

    uint16_t lastFrame(void);

    bool endpointSetInterrupt(uint8_t bEP, bool enabled);
    bool endpointGetInterrupt(uint8_t bEP);
    void endpointTriggerInterrupt(uint8_t bEP);

    /* misc hardware stuff */
    uint32_t getSerialNumber(int length, uint32_t *buf);

    static void _usbisr(void);

    void usbisr(void);
    static USBHAL * instance;
    uint8_t can_transfer[32];

protected:
    virtual bool USBEvent_busReset(void){return false;};
    virtual bool USBEvent_connectStateChanged(bool connected){return false;};
    virtual bool USBEvent_suspendStateChanged(bool suspended){return false;};

    virtual bool USBEvent_Frame(uint16_t){return false;};

    virtual bool USBEvent_Request(CONTROL_TRANSFER&){return false;};
    virtual bool USBEvent_RequestComplete(CONTROL_TRANSFER&, uint8_t *, uint32_t){return false;};

    virtual bool USBEvent_EPIn(uint8_t, uint8_t){return false;};
    virtual bool USBEvent_EPOut(uint8_t, uint8_t){return false;};

    virtual void busReset(void){};
    virtual void EP0setupCallback(void){};
    virtual void EP0out(void){};
    virtual void EP0in(void){};
    virtual void connectStateChanged(unsigned int connected){};
    virtual void suspendStateChanged(unsigned int suspended){};
    virtual void SOF(int frameNumber){};
};

#endif
