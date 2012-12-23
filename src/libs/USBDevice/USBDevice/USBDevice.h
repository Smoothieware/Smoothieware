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

#ifndef USBDEVICE_H
#define USBDEVICE_H

// #include "mbed.h"
#include "USBDevice_Types.h"
#include "USBHAL.h"

#include "../descriptor.h"

class USBDevice: public USBHAL
{
public:
    USBDevice();

    bool configured(void);

    bool setDescriptors(usbdesc_base ** descriptors);

    void connect(void);

    void disconnect(void);

    bool addEndpoint(uint8_t endpoint, uint32_t maxPacket);

    bool readStart(uint8_t endpoint, uint32_t maxSize);
    bool readEP(uint8_t endpoint, uint8_t * buffer, uint32_t * size, uint32_t maxSize);
    bool readEP_NB(uint8_t endpoint, uint8_t * buffer, uint32_t * size, uint32_t maxSize);

    bool write(uint8_t endpoint, uint8_t * buffer, uint32_t size, uint32_t maxSize);
    bool writeNB(uint8_t endpoint, uint8_t * buffer, uint32_t size, uint32_t maxSize);

    virtual bool USBCallback_setConfiguration(uint8_t configuration) { return false; };

    virtual bool USBCallback_setInterface(uint16_t interface, uint8_t alternate) { return false; };

protected:
    virtual bool USBEvent_busReset(void);

    virtual void EP0setupCallback(void);
    virtual void EP0out(void);
    virtual void EP0in(void);

    virtual bool USBEvent_Frame(uint16_t);

    int  findDescriptorIndex(uint8_t start, uint8_t descriptorType, uint8_t descriptorIndex, uint8_t alternate);
    int  findDescriptorIndex(uint8_t descriptorType, uint8_t descriptorIndex);
    uint8_t * findDescriptor(uint8_t descriptorType, uint8_t descriptorIndex);
    uint8_t * findDescriptor(uint8_t descriptorType);

    CONTROL_TRANSFER * getTransferPtr(void);

    uint16_t VENDOR_ID;
    uint16_t PRODUCT_ID;
    uint16_t PRODUCT_RELEASE;

private:
    bool addRateFeedbackEndpoint(uint8_t endpoint, uint32_t maxPacket);
    bool requestGetDescriptor(void);
    bool controlOut(void);
    bool controlIn(void);
    bool requestSetAddress(void);
    bool requestSetConfiguration(void);
    bool requestSetFeature(void);
    bool requestClearFeature(void);
    bool requestGetStatus(void);
    bool requestSetup(void);
    bool controlSetup(void);
    void decodeSetupPacket(uint8_t *data, SETUP_PACKET *packet);
    bool requestGetConfiguration(void);
    bool requestGetInterface(void);
    bool requestSetInterface(void);
    bool assembleConfigDescriptor(void);

    usbdesc_base **descriptors;

    volatile uint16_t lastFrameIndex;

    CONTROL_TRANSFER transfer;
    USB_DEVICE device;

    uint8_t control_buffer[64];

    uint16_t currentInterface;
    uint8_t currentAlternate;
};


#endif
