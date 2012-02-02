/* USBBusInterface.h */
/* USB Bus Interface */
/* Copyright (c) 2011 ARM Limited. All rights reserved. */

#ifndef USBBUSINTERFACE_H
#define USBBUSINTERFACE_H

#include "mbed.h"
#include "USBEndpoints.h"

class USBHAL {
public:
    /* Configuration */
    USBHAL();
    ~USBHAL();
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
    EP_STATUS endpointRead(uint8_t endpoint, uint32_t maximumSize);
    EP_STATUS endpointReadResult(uint8_t endpoint, uint8_t *data, uint32_t *bytesRead);
    EP_STATUS endpointWrite(uint8_t endpoint, uint8_t *data, uint32_t size);
    EP_STATUS endpointWriteResult(uint8_t endpoint);
    void stallEndpoint(uint8_t endpoint);
    void unstallEndpoint(uint8_t endpoint);
    bool realiseEndpoint(uint8_t endpoint, uint32_t maxPacket, uint32_t options);
    bool getEndpointStallState(unsigned char endpoint);
    uint32_t endpointReadcore(uint8_t endpoint, uint8_t *buffer);
    
protected:
    virtual void busReset(void){};
    virtual void EP0setupCallback(void){};
    virtual void EP0out(void){};
    virtual void EP0in(void){};
    virtual void connectStateChanged(unsigned int connected){};
    virtual void suspendStateChanged(unsigned int suspended){};
    virtual void SOF(int frameNumber){};
    virtual bool EP1_OUT_callback(){return false;};
    virtual bool EP1_IN_callback(){return false;};
    virtual bool EP2_OUT_callback(){return false;};
    virtual bool EP2_IN_callback(){return false;};
    virtual bool EP3_OUT_callback(){return false;};
    virtual bool EP3_IN_callback(){return false;};
    
private:
    void usbisr(void);
    static void _usbisr(void);
    static USBHAL * instance;
};
#endif


