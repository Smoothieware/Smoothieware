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

#include <stdint.h>

// for memcpy
#include <cstring>
// for iprintf
#include <cstdio>

#include "USBEndpoints.h"
#include "USBDevice.h"
#include "USBDescriptor.h"

#define DEBUG 1
#define printf iprintf

/* Device status */
#define DEVICE_STATUS_SELF_POWERED  (1U<<0)
#define DEVICE_STATUS_REMOTE_WAKEUP (1U<<1)

/* Endpoint status */
#define ENDPOINT_STATUS_HALT        (1U<<0)

/* Standard feature selectors */
#define DEVICE_REMOTE_WAKEUP        (1)
#define ENDPOINT_HALT               (0)

/* Macro to convert wIndex endpoint number to physical endpoint number */
#define WINDEX_TO_PHYSICAL(endpoint) (((endpoint & 0x0f) << 1) + \
    ((endpoint & 0x80) ? 1 : 0))

#define iprintf(...)

#define setled(a, b) do {} while (0)
// extern void setled(int, bool);

bool USBDevice::setDescriptors(usbdesc_base ** newDescriptors)
{
    if (configured() == false)
    {
        descriptors = newDescriptors;
        return true;
    }

    if (descriptors == newDescriptors)
        return true;

    return false;
}

bool USBDevice::assembleConfigDescriptor()
{
    static uint8_t confBuffer[MAX_PACKET_SIZE_EP0];
    static uint8_t confIndex = 0;
    static uint8_t confSubIndex = 0;

    uint8_t confBufferPtr = 0;
    transfer.ptr = confBuffer;

    iprintf("ASSEMBLE CONFIG\n");

    // magic number to reset descriptor assembler
    if (transfer.remaining == -1)
    {
        iprintf("CONFIG:RESET\n");
        // loop through descriptors, find matching configuration descriptor
        for (int i = 0; descriptors[i] != (usbdesc_base *) 0; i++)
        {
            if (descriptors[i]->bDescType == DT_CONFIGURATION)
            {
                usbdesc_configuration *conf = (usbdesc_configuration *) descriptors[i];
                iprintf("CONFIG:FOUND %d(%d) @%d\n", conf->bConfigurationValue, DESCRIPTOR_INDEX(transfer.setup.wValue), i);

                if (
                    (conf->bConfigurationValue == DESCRIPTOR_INDEX(transfer.setup.wValue))
                    ||
                    (
                        (conf->bConfigurationValue == 1)
                        &&
                        (DESCRIPTOR_INDEX(transfer.setup.wValue) == 0)
                    )
                   )
                {
                    // we're going to transmit wTotalLength to the host
                    transfer.remaining = conf->wTotalLength;
                    // limit length
                    if (transfer.remaining > transfer.setup.wLength)
                        transfer.remaining = transfer.setup.wLength;
                    // start at index of this config descriptor
                    confIndex = i;
                    // reset counters
                    confSubIndex = 0;
                    iprintf("CONFIG:Sending %ld bytes\n", transfer.remaining);
                    break;
                }
            }
        }
    }

    if (transfer.remaining == -1)
        return false;

    int remaining = transfer.remaining;

    // assemble a packet
    while ((confBufferPtr < MAX_PACKET_SIZE_EP0) && (remaining > 0))
    {
        iprintf("CONFIG:%d bytes to go, at %d:%d\n", remaining, confIndex, confSubIndex);
        // if we haven't finished the current descriptor, copy the rest into the buffer
        if (descriptors[confIndex]->bLength > confSubIndex)
        {
            uint8_t *descriptorBuf = (uint8_t *) descriptors[confIndex];
            // destination - appropriate position in confBuffer
            uint8_t *dst = &confBuffer[confBufferPtr];
            // source - appropriate position in current descriptor
            uint8_t *src = &descriptorBuf[confSubIndex];
            // length - rest of descriptor
            uint8_t  len = descriptors[confIndex]->bLength - confSubIndex;
            // max length - amount of space left in confBuffer
            uint8_t maxlen = MAX_PACKET_SIZE_EP0 - confBufferPtr;
            // max length - remaining size of configuration descriptor
            if (maxlen > remaining)
                maxlen = remaining;
            // limit length to max length
            if (len > maxlen)
                len = maxlen;
            // copy to buffer
            iprintf("CONFIG:cpy %d from %p to %p\n", len, src, dst);
            memcpy(dst, src, len);
            // advance position pointer
            confSubIndex += len;
            confBufferPtr += len;
            remaining -= len;
        }
        // this descriptor complete, move to next descriptor
        if (descriptors[confIndex]->bLength <= confSubIndex)
        {
            iprintf("CONFIG:next\n");
            do {
                confIndex += 1;
            } while ((descriptors[confIndex] != NULL) && (descriptors[confIndex]->bDescType == DT_STRING));
            confSubIndex = 0;
            if (descriptors[confIndex] == 0)
                return false;
        }
    }
    if (remaining < transfer.remaining)
        return true;
    return false;
}

bool USBDevice::requestGetDescriptor(void)
{
    uint8_t bType, bIndex;
//     int iCurIndex;

    bType = DESCRIPTOR_TYPE(transfer.setup.wValue);
    bIndex = DESCRIPTOR_INDEX(transfer.setup.wValue);

    iprintf("GET DESCRIPTOR %02Xh %02Xh\n", bType, bIndex);
    int i = findDescriptorIndex(bType, bIndex);
    if (i >= 0) {
        iprintf("FOUND at %d\n", i);
        if (bType == DT_CONFIGURATION)
        {
            transfer.remaining = -1;
            assembleConfigDescriptor();
            iprintf("Assembling CONFIG descriptor: %ldb\n", transfer.remaining);
        }
        else
        {
            transfer.remaining = descriptors[i]->bLength;
            transfer.ptr = (uint8_t *) descriptors[i];
            iprintf("Assembling descriptor %d(%d): %ldb\n", bType, bIndex, transfer.remaining);
        }
        transfer.direction = DEVICE_TO_HOST;
        return true;
    }

    return false;
}

void USBDevice::decodeSetupPacket(uint8_t *data, SETUP_PACKET *packet)
{
    /* Fill in the elements of a SETUP_PACKET structure from raw data */
    packet->bmRequestType.dataTransferDirection = (data[0] & 0x80) >> 7;
    packet->bmRequestType.Type = (data[0] & 0x60) >> 5;
    packet->bmRequestType.Recipient = data[0] & 0x1f;
    packet->bRequest = data[1];
    packet->wValue = (data[2] | (uint16_t)data[3] << 8);
    packet->wIndex = (data[4] | (uint16_t)data[5] << 8);
    packet->wLength = (data[6] | (uint16_t)data[7] << 8);
}

bool USBDevice::controlOut(void)
{
    iprintf("{ControlOUT}");
    /* Control transfer data OUT stage */
//     uint8_t buffer[MAX_PACKET_SIZE_EP0];
    int32_t packetSize;

    /* Check we should be transferring data OUT */
    if (transfer.direction != HOST_TO_DEVICE)
    {
        return false;
    }

    /* Read from endpoint */
    packetSize = EP0getReadResult(transfer.ptr);

    /* Check if transfer size is valid */
    if (packetSize > transfer.remaining)
    {
        /* Too big */
        return false;
    }

    /* Update transfer */
    transfer.ptr += packetSize;
    transfer.remaining -= packetSize;

    /* Check if transfer has completed */
    if (transfer.remaining == 0)
    {
        /* Transfer completed */
        if (transfer.notify)
        {
            /* Notify class layer. */
//             USBCallback_requestCompleted(buffer, packetSize);
            USBEvent_RequestComplete(transfer, transfer.ptr, packetSize);
            transfer.notify = false;
        }
        /* Status stage */
        EP0write(NULL, 0);
    }
    else
    {
        EP0read();
    }

    return true;
}

bool USBDevice::controlIn(void)
{
    iprintf("{ControlIN}");
    /* Control transfer data IN stage */
    uint32_t packetSize;

    /* Check if transfer has completed (status stage transactions */
    /* also have transfer.remaining == 0) */
    if (transfer.remaining == 0)
    {
        if (transfer.zlp)
        {
            /* Send zero length packet */
            EP0write(NULL, 0);
            transfer.zlp = false;
        }

        /* Transfer completed */
        if (transfer.notify)
        {
            /* Notify class layer. */
//             USBCallback_requestCompleted(NULL, 0);
            USBEvent_RequestComplete(transfer, NULL, 0);
            transfer.notify = false;
        }

        EP0read();

        iprintf("{z}");
        /* Completed */
        return true;
    }

    /* Check we should be transferring data IN */
    if (transfer.direction != DEVICE_TO_HOST)
    {
        iprintf("WRONG DIR");
        return false;
    }

    packetSize = transfer.remaining;

    if (packetSize > MAX_PACKET_SIZE_EP0)
    {
        packetSize = MAX_PACKET_SIZE_EP0;
    }

    iprintf("[W:%ld]", packetSize);

    /* Write to endpoint */
    EP0write(transfer.ptr, packetSize);

    iprintf(",");

    transfer.remaining -= packetSize;

    /* Update transfer */
    if ((transfer.setup.bRequest == GET_DESCRIPTOR)                  &&
        (transfer.setup.bmRequestType.Type == STANDARD_TYPE)         &&
        (DESCRIPTOR_TYPE(transfer.setup.wValue) == DT_CONFIGURATION)    )
    {
        assembleConfigDescriptor();
    }
    else
    {
        transfer.ptr += packetSize;
    }
    iprintf("[RM:%ld]\n", transfer.remaining);

    return true;
}

bool USBDevice::requestSetAddress(void)
{
    /* Set the device address */
    setAddress(transfer.setup.wValue);

    if (transfer.setup.wValue == 0)
    {
        device.state = DEFAULT;
    }
    else
    {
        device.state = ADDRESS;
    }

    iprintf("ADDRESS is 0x%02X!\n", transfer.setup.wValue);

    return true;
}

bool USBDevice::requestSetConfiguration(void)
{
    device.configuration = transfer.setup.wValue;
    iprintf("SET CONFIGURATION: %d\n", transfer.setup.wValue);
    /* Set the device configuration */
    if (device.configuration == 0)
    {
        iprintf("DECONFIGURED\n");
        /* Not configured */
        unconfigureDevice();
        device.state = ADDRESS;
    }
    else
    {
        if (USBCallback_setConfiguration(device.configuration))
        {
            iprintf("SET CONFIGURATION:SUCCESS!\n");
            /* Valid configuration */
            configureDevice();
            device.state = CONFIGURED;
        }
        else
        {
            iprintf("SET CONFIGURATION:failure!\n");
            return false;
        }
    }

    return true;
}

bool USBDevice::requestGetConfiguration(void)
{
    /* Send the device configuration */
    transfer.ptr = &device.configuration;
    transfer.remaining = sizeof(device.configuration);
    transfer.direction = DEVICE_TO_HOST;
    return true;
}

bool USBDevice::requestGetInterface(void)
{
    /* Return the selected alternate setting for an interface */

    if (device.state != CONFIGURED)
    {
        return false;
    }

    /* Send the alternate setting */
    transfer.setup.wIndex = currentInterface;
    transfer.ptr = &currentAlternate;
    transfer.remaining = sizeof(currentAlternate);
    transfer.direction = DEVICE_TO_HOST;
    return true;
}

bool USBDevice::requestSetInterface(void)
{
    bool success = false;
    if(USBCallback_setInterface(transfer.setup.wIndex, transfer.setup.wValue))
    {
        success = true;
        currentInterface = transfer.setup.wIndex;
        currentAlternate = transfer.setup.wValue;
    }
    return success;
}

bool USBDevice::requestSetFeature()
{
    bool success = false;

    if (device.state != CONFIGURED)
    {
        /* Endpoint or interface must be zero */
        if (transfer.setup.wIndex != 0)
        {
            return false;
        }
    }

    switch (transfer.setup.bmRequestType.Recipient)
    {
        case DEVICE_RECIPIENT:
            /* TODO: Remote wakeup feature not supported */
            break;
        case ENDPOINT_RECIPIENT:
            if (transfer.setup.wValue == ENDPOINT_HALT)
            {
                /* TODO: We should check that the endpoint number is valid */
                stallEndpoint(
                    WINDEX_TO_PHYSICAL(transfer.setup.wIndex));
                success = true;
            }
            break;
        default:
            break;
    }

    return success;
}

bool USBDevice::requestClearFeature()
{
    bool success = false;

    if (device.state != CONFIGURED)
    {
        /* Endpoint or interface must be zero */
        if (transfer.setup.wIndex != 0)
        {
            return false;
        }
    }

    switch (transfer.setup.bmRequestType.Recipient)
    {
        case DEVICE_RECIPIENT:
            /* TODO: Remote wakeup feature not supported */
            break;
        case ENDPOINT_RECIPIENT:
            /* TODO: We should check that the endpoint number is valid */
            if (transfer.setup.wValue == ENDPOINT_HALT)
            {
                unstallEndpoint(transfer.setup.wIndex);
                success = true;
            }
            break;
        default:
            break;
    }

    return success;
}

bool USBDevice::requestGetStatus(void)
{
    static uint16_t status;
    bool success = false;

    if (device.state != CONFIGURED)
    {
        /* Endpoint or interface must be zero */
        if (transfer.setup.wIndex != 0)
        {
            return false;
        }
    }

    switch (transfer.setup.bmRequestType.Recipient)
    {
        case DEVICE_RECIPIENT:
            /* TODO: Currently only supports self powered devices */
            status = DEVICE_STATUS_SELF_POWERED;
            success = true;
            break;
        case INTERFACE_RECIPIENT:
            status = 0;
            success = true;
            break;
        case ENDPOINT_RECIPIENT:
            /* TODO: We should check that the endpoint number is valid */
            if (getEndpointStallState(transfer.setup.wIndex))
            {
                status = ENDPOINT_STATUS_HALT;
            }
            else
            {
                status = 0;
            }
            success = true;
            break;
        default:
            break;
    }

    if (success)
    {
        /* Send the status */
        transfer.ptr = (uint8_t *)&status; /* Assumes little endian */
        transfer.remaining = sizeof(status);
        transfer.direction = DEVICE_TO_HOST;
    }

    return success;
}

bool USBDevice::requestSetup(void)
{
//     bool success = false;

    /* Process standard requests */
    if ((transfer.setup.bmRequestType.Type == STANDARD_TYPE))
    {
        switch (transfer.setup.bRequest)
        {
             case GET_STATUS:
                return requestGetStatus();
             case CLEAR_FEATURE:
                return requestClearFeature();
             case SET_FEATURE:
                return requestSetFeature();
             case SET_ADDRESS:
                return requestSetAddress();
            case GET_DESCRIPTOR:
                return requestGetDescriptor();
            case SET_DESCRIPTOR:
                /* TODO: Support is optional, not implemented here */
                return false;
            case GET_CONFIGURATION:
                return requestGetConfiguration();
            case SET_CONFIGURATION:
                return requestSetConfiguration();
            case GET_INTERFACE:
                return requestGetInterface();
            case SET_INTERFACE:
                return requestSetInterface();

            default:
                return false;
        }
    }

    return false;
}

bool USBDevice::controlSetup(void)
{
    iprintf("{ControlSETUP}");
    /* Control transfer setup stage */

    EP0setup(control_buffer);

    /* Initialise control transfer state */
    decodeSetupPacket(control_buffer, &transfer.setup);
    transfer.ptr = control_buffer;
    transfer.remaining = 0;
    transfer.direction = 0;
    transfer.zlp = false;
    transfer.notify = false;

#ifdef DEBUG
    printf("dataTransferDirection: %d\r\nType: %d\r\nRecipient: %d\r\nbRequest: %d\r\nwValue: %d\r\nwIndex: %d\r\nwLength: %d\r\n",transfer.setup.bmRequestType.dataTransferDirection,
                                                                                                                                   transfer.setup.bmRequestType.Type,
                                                                                                                                   transfer.setup.bmRequestType.Recipient,
                                                                                                                                   transfer.setup.bRequest,
                                                                                                                                   transfer.setup.wValue,
                                                                                                                                   transfer.setup.wIndex,
                                                                                                                                   transfer.setup.wLength);
#endif

    /* Class / vendor specific */
    if (!USBEvent_Request(transfer))
    {
        /* Standard requests */
        if (!requestSetup())
        {
#ifdef DEBUG
            printf("fail!!!!\n");
#endif
            return false;
        }
        iprintf("OK\n");
    }

    /* Check transfer size and direction */
    if (transfer.setup.wLength > 0)
    {
        if (transfer.setup.bmRequestType.dataTransferDirection == DEVICE_TO_HOST)
        {
            /* IN data stage is required */
            if (transfer.direction != DEVICE_TO_HOST)
            {
                iprintf("<a\n");
                return false;
            }

            /* Transfer must be less than or equal to the size */
            /* requested by the host */
            if (transfer.remaining > transfer.setup.wLength)
            {
                transfer.remaining = transfer.setup.wLength;
            }
        }
        else
        {
            /* OUT data stage is required */
            if (transfer.direction != HOST_TO_DEVICE)
            {
                iprintf("<b\n");
                return false;
            }

            /* Transfer must be equal to the size requested by the host */
            if (transfer.remaining != transfer.setup.wLength)
            {
                iprintf("<c\n");
                return false;
            }
        }
    }
    else
    {
        /* No data stage; transfer size must be zero */
        if (transfer.remaining != 0)
        {
            iprintf("<d\n");
            return false;
        }
    }

    /* Data or status stage if applicable */
    if (transfer.setup.wLength > 0)
    {
        if (transfer.setup.bmRequestType.dataTransferDirection == DEVICE_TO_HOST)
        {
            /* Check if we'll need to send a zero length packet at */
            /* the end of this transfer */
            if (transfer.setup.wLength > transfer.remaining)
            {
                /* Device wishes to transfer less than host requested */
                if ((transfer.remaining % MAX_PACKET_SIZE_EP0) == 0)
                {
                    /* Transfer is a multiple of EP0 max packet size */
                    transfer.zlp = true;
                }
            }

            /* IN stage */
            controlIn();
        }
        else
        {
            iprintf(">f");
            /* OUT stage */
            EP0read();
        }
    }
    else
    {
        iprintf(">g");
        /* Status stage */
        EP0write(NULL, 0);
    }

    iprintf("<e\n");
    return true;
}

bool USBDevice::USBEvent_busReset(void)
{
    device.state = DEFAULT;
    device.configuration = 0;
    device.suspended = false;

    return true;
}

bool USBDevice::USBEvent_Frame(uint16_t Frame)
{
    lastFrameIndex = Frame;
    return true;
}

void USBDevice::EP0setupCallback(void)
{
#ifdef DEBUG
    iprintf("EP0Setup\n");
#endif
    /* Endpoint 0 setup event */
    if (!controlSetup())
    {
        /* Protocol stall */
        EP0stall();
    }
}

void USBDevice::EP0out(void)
{
#ifdef DEBUG
    iprintf("EP0OUT\n");
#endif
    /* Endpoint 0 OUT data event */
    if (!controlOut())
    {
        /* Protocol stall; this will stall both endpoints */
        EP0stall();
    }
}

void USBDevice::EP0in(void)
{
#ifdef DEBUG
    printf("EP0IN\r\n");
#endif
    /* Endpoint 0 IN data event */
    if (!controlIn())
    {
        /* Protocol stall; this will stall both endpoints */
        EP0stall();
    }
}

bool USBDevice::configured(void)
{
    /* Returns true if device is in the CONFIGURED state */
    return (device.state == CONFIGURED);
}

void USBDevice::connect(void)
{
//     iprintf("USBDevice::connect\n");
    /* Connect device */
    USBHAL::connect();
//     iprintf("USBDevice::connect OK\n");

    /* Block if not configured */
//     while (!configured());
}

void USBDevice::disconnect(void)
{
    /* Disconnect device */
    USBHAL::disconnect();
}

CONTROL_TRANSFER * USBDevice::getTransferPtr(void)
{
    return &transfer;
}

bool USBDevice::addEndpoint(uint8_t endpoint, uint32_t maxPacket)
{
    return realiseEndpoint(endpoint, maxPacket, 0);
}

bool USBDevice::addRateFeedbackEndpoint(uint8_t endpoint, uint32_t maxPacket)
{
    /* For interrupt endpoints only */
    return realiseEndpoint(endpoint, maxPacket, RATE_FEEDBACK_MODE);
}

int USBDevice::findDescriptorIndex(uint8_t start, uint8_t descriptorType, uint8_t descriptorIndex, uint8_t alternate)
{
//     uint8_t currentConfiguration = 0;
    uint8_t index = 0;

    if (descriptorType == DT_DEVICE)
        return 0;

    if (descriptorType == DT_CONFIGURATION)
        return 1;

    int i;
    for (i = start; descriptors[i] != NULL; i++) {
        if (descriptors[i]->bDescType == DT_CONFIGURATION) {
//             usbdesc_configuration *conf = (usbdesc_configuration *) descriptors[i];
//             currentConfiguration = conf->bConfigurationValue;
//             iprintf("CONF{%d/%d}\n", currentConfiguration, device.configuration);
            index = 0;
        }
//         if ((currentConfiguration == device.configuration) || (descriptorType == DT_CONFIGURATION)) {
            if (descriptors[i]->bDescType == descriptorType) {
                switch(descriptorType) {
                    case DT_CONFIGURATION: {
                        index = ((usbdesc_configuration *) descriptors[i])->bConfigurationValue;
                    };
                    case DT_INTERFACE: {
                        index = ((usbdesc_interface *) descriptors[i])->bInterfaceNumber;
                    };
                    case DT_ENDPOINT: {
                        index = ((usbdesc_endpoint *) descriptors[i])->bEndpointAddress;
                    };
                }
                iprintf("FOUND %d:%d at %d, looking for %d\n", descriptorType, index, i, descriptorIndex);
                if (index == descriptorIndex)
                {
                    if (
                        (descriptorType != DT_INTERFACE)
                        ||
                        (((usbdesc_interface *) descriptors[i])->bAlternateSetting == alternate)
                       )
                    {
                        iprintf("Descriptor Found at %d!\n", i);
                        return i;
                    }
                }
                index++;
            }
//         }
    }
    iprintf("Descriptor not found\n");
    return -1;
}

int USBDevice::findDescriptorIndex(uint8_t descriptorType, uint8_t descriptorIndex)
{
    return findDescriptorIndex(0, descriptorType, descriptorIndex, 0);
}

uint8_t * USBDevice::findDescriptor(uint8_t descriptorType, uint8_t descriptorIndex)
{
    int i = findDescriptorIndex(descriptorType, descriptorIndex);
    if (i >= 0)
        return (uint8_t *) descriptors[i];
    return NULL;
}

uint8_t * USBDevice::findDescriptor(uint8_t descriptorType)
{
    if (descriptorType == DT_CONFIGURATION)
        return findDescriptor(descriptorType, 1);
    else
        return findDescriptor(descriptorType, 0);
    /* Reached end of the descriptors - not found */
    return NULL;
}

USBDevice::USBDevice()
{
    device.state = POWERED;
    device.configuration = 0;
    device.suspended = false;
};


bool USBDevice::readStart(uint8_t endpoint, uint32_t maxSize)
{
    endpointSetInterrupt(endpoint, true);
    return endpointRead(endpoint, maxSize) == EP_PENDING;
}

bool USBDevice::write(uint8_t endpoint, uint8_t * buffer, uint32_t size, uint32_t maxSize)
{
    EP_STATUS result;

    if (size > maxSize)
    {
        return false;
    }


    if(!configured()) {
        return false;
    }

    /* Send report */
    result = endpointWrite(endpoint, buffer, size);

    if (result != EP_PENDING)
    {
        return false;
    }

    /* Wait for completion */
    setled(4, 1);
    do {
        result = endpointWriteResult(endpoint);
    } while ((result == EP_PENDING) && configured());
    setled(4, 0);

    return (result == EP_COMPLETED);
}

bool USBDevice::writeNB(uint8_t endpoint, uint8_t * buffer, uint32_t size, uint32_t maxSize)
{
    EP_STATUS result;

    if (size > maxSize)
    {
        return false;
    }

    if(!configured()) {
        return false;
    }

    /* Send report */
    result = endpointWrite(endpoint, buffer, size);

    if (result != EP_PENDING)
    {
        return false;
    }

    result = endpointWriteResult(endpoint);

    return (result == EP_COMPLETED);
}

bool USBDevice::readEP(uint8_t bEP, uint8_t * buffer, uint32_t * size, uint32_t maxSize)
{
    EP_STATUS result;

    if(!configured()) {
        return false;
    }

    setled(4, 1);
    /* Wait for completion */
    do {
        result = endpointReadResult(bEP, buffer, size);
    } while ((result == EP_PENDING) && configured());
    setled(4, 0);

    return (result == EP_COMPLETED);
}

bool USBDevice::readEP_NB(uint8_t endpoint, uint8_t * buffer, uint32_t * size, uint32_t maxSize)
{
    EP_STATUS result;

    if(!configured()) {
        return false;
    }

    result = endpointReadResult(endpoint, buffer, size);

    return (result == EP_COMPLETED);
}
