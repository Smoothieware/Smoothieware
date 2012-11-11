/* USBDevice.c */
/* Generic USB device */
/* Copyright (c) 2011 ARM Limited. All rights reserved. */

/* Reference: */
/* Universal Serial Bus Specification Revision 2.0, Chapter 9 "USB Device Framework" */

#include "stdint.h"

#include "USBEndpoints.h"
#include "USBDevice.h"
#include "USBDescriptor.h"
#include "USBHID_Types.h"


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


bool USBDevice::requestGetDescriptor(void)
{
    bool success = false;

    switch (DESCRIPTOR_TYPE(transfer.setup.wValue))
    {
        case DEVICE_DESCRIPTOR:
            if (deviceDesc() != NULL)
            {
                if ((deviceDesc()[0] == DEVICE_DESCRIPTOR_LENGTH) \
                    && (deviceDesc()[1] == DEVICE_DESCRIPTOR))
                {
                    transfer.remaining = DEVICE_DESCRIPTOR_LENGTH;
                    transfer.ptr = deviceDesc();
                    transfer.direction = DEVICE_TO_HOST;
                    success = true;
                }
            }
            break;
        case CONFIGURATION_DESCRIPTOR:
            if (configurationDesc() != NULL)
            {
                if ((configurationDesc()[0] == CONFIGURATION_DESCRIPTOR_LENGTH) \
                    && (configurationDesc()[1] == CONFIGURATION_DESCRIPTOR))
                {
                    /* Get wTotalLength */
                    transfer.remaining = configurationDesc()[2] \
                        | (configurationDesc()[3] << 8);

                    transfer.ptr = configurationDesc();
                    transfer.direction = DEVICE_TO_HOST;
                    success = true;
                }
            }
            break;
        case STRING_DESCRIPTOR:
            switch (DESCRIPTOR_INDEX(transfer.setup.wValue))
            {
                            case STRING_OFFSET_LANGID:
                                transfer.remaining = stringLangidDesc()[0];
                                transfer.ptr = stringLangidDesc();
                                transfer.direction = DEVICE_TO_HOST;
                                success = true;
                                break;
                            case STRING_OFFSET_IMANUFACTURER:
                                transfer.remaining =  stringImanufacturerDesc()[0];
                                transfer.ptr = stringImanufacturerDesc();
                                transfer.direction = DEVICE_TO_HOST;
                                success = true;
                                break;
                            case STRING_OFFSET_IPRODUCT:
                                transfer.remaining = stringIproductDesc()[0];
                                transfer.ptr = stringIproductDesc();
                                transfer.direction = DEVICE_TO_HOST;
                                success = true;
                                break;
                            case STRING_OFFSET_ISERIAL:
                                transfer.remaining = stringIserialDesc()[0];
                                transfer.ptr = stringIserialDesc();
                                transfer.direction = DEVICE_TO_HOST;
                                success = true;
                                break;
                            case STRING_OFFSET_ICONFIGURATION:
                                transfer.remaining = stringIConfigurationDesc()[0];
                                transfer.ptr = stringIConfigurationDesc();
                                transfer.direction = DEVICE_TO_HOST;
                                success = true;
                                break;
                            case STRING_OFFSET_IINTERFACE:
                                transfer.remaining = stringIinterfaceDesc()[0];
                                transfer.ptr = stringIinterfaceDesc();
                                transfer.direction = DEVICE_TO_HOST;
                                success = true;
                                break;
            }
            break;
        case INTERFACE_DESCRIPTOR:
        case ENDPOINT_DESCRIPTOR:
            /* TODO: Support is optional, not implemented here */
            break;
        default:
            break;
    }

    return success;
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
    /* Control transfer data OUT stage */
    uint8_t buffer[MAX_PACKET_SIZE_EP0];
    uint32_t packetSize;

    /* Check we should be transferring data OUT */
    if (transfer.direction != HOST_TO_DEVICE)
    {
        return false;
    }

    /* Read from endpoint */
    packetSize = EP0getReadResult(buffer);

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
            USBCallback_requestCompleted(buffer, packetSize);
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
            USBCallback_requestCompleted(NULL, 0);
            transfer.notify = false;
        }

        EP0read();

        /* Completed */
        return true;
    }

    /* Check we should be transferring data IN */
    if (transfer.direction != DEVICE_TO_HOST)
    {
        return false;
    }

    packetSize = transfer.remaining;

    if (packetSize > MAX_PACKET_SIZE_EP0)
    {
        packetSize = MAX_PACKET_SIZE_EP0;
    }

    /* Write to endpoint */
    EP0write(transfer.ptr, packetSize);

    /* Update transfer */
    transfer.ptr += packetSize;
    transfer.remaining -= packetSize;

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

    return true;
}

bool USBDevice::requestSetConfiguration(void)
{

    device.configuration = transfer.setup.wValue;
    /* Set the device configuration */
    if (device.configuration == 0)
    {
        /* Not configured */
        unconfigureDevice();
        device.state = ADDRESS;
    }
    else
    {
        if (USBCallback_setConfiguration(device.configuration))
        {
            /* Valid configuration */
            configureDevice();
            device.state = CONFIGURED;
        }
        else
        {
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
                unstallEndpoint( WINDEX_TO_PHYSICAL(transfer.setup.wIndex));
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
            if (getEndpointStallState(
                WINDEX_TO_PHYSICAL(transfer.setup.wIndex)))
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
    bool success = false;

    /* Process standard requests */
    if ((transfer.setup.bmRequestType.Type == STANDARD_TYPE))
    {
        switch (transfer.setup.bRequest)
        {
             case GET_STATUS:
                 success = requestGetStatus();
                 break;
             case CLEAR_FEATURE:
                 success = requestClearFeature();
                 break;
             case SET_FEATURE:
                 success = requestSetFeature();
                 break;
             case SET_ADDRESS:
                success = requestSetAddress();
                 break;
             case GET_DESCRIPTOR:
                 success = requestGetDescriptor();
                 break;
             case SET_DESCRIPTOR:
                 /* TODO: Support is optional, not implemented here */
                 success = false;
                 break;
             case GET_CONFIGURATION:
                 success = requestGetConfiguration();
                 break;
             case SET_CONFIGURATION:
                 success = requestSetConfiguration();
                 break;
             case GET_INTERFACE:
                 success = requestGetInterface();
                 break;
             case SET_INTERFACE:
                 success = requestSetInterface();
                 break;
             default:
                 break;
        }
    }

    return success;
}

bool USBDevice::controlSetup(void)
{
    bool success = false;

    /* Control transfer setup stage */
    uint8_t buffer[MAX_PACKET_SIZE_EP0];

    EP0setup(buffer);

    /* Initialise control transfer state */
    decodeSetupPacket(buffer, &transfer.setup);
    transfer.ptr = NULL;
    transfer.remaining = 0;
    transfer.direction = 0;
    transfer.zlp = false;
    transfer.notify = false;

    /* Class / vendor specific */
    success = USBCallback_request();

    if (!success)
    {
        /* Standard requests */
        if (!requestSetup())
        {
            return false;
        }
    }

    /* Check transfer size and direction */
    if (transfer.setup.wLength>0)
    {
        if (transfer.setup.bmRequestType.dataTransferDirection \
            == DEVICE_TO_HOST)
        {
            /* IN data stage is required */
            if (transfer.direction != DEVICE_TO_HOST)
            {
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
                return false;
            }

            /* Transfer must be equal to the size requested by the host */
            if (transfer.remaining != transfer.setup.wLength)
            {
                return false;
            }
        }
    }
    else
    {
        /* No data stage; transfer size must be zero */
        if (transfer.remaining != 0)
        {
            return false;
        }
    }

    /* Data or status stage if applicable */
    if (transfer.setup.wLength>0)
    {
        if (transfer.setup.bmRequestType.dataTransferDirection \
            == DEVICE_TO_HOST)
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
            /* OUT stage */
            EP0read();
        }
    }
    else
    {
        /* Status stage */
        EP0write(NULL, 0);
    }

    return true;
}

void USBDevice::busReset(void)
{
    device.state = DEFAULT;
    device.configuration = 0;
    device.suspended = false;

    /* Call class / vendor specific busReset function */
    USBCallback_busReset();
}

void USBDevice::EP0setupCallback(void)
{
    /* Endpoint 0 setup event */
    if (!controlSetup())
    {
        /* Protocol stall */
        EP0stall();
    }

    /* Return true if an OUT data stage is expected */
}

void USBDevice::EP0out(void)
{
    /* Endpoint 0 OUT data event */
    if (!controlOut())
    {
        /* Protocol stall; this will stall both endpoints */
        EP0stall();
    }
}

void USBDevice::EP0in(void)
{
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
    /* Connect device */
    USBHAL::connect();
    /* Block if not configured */
    while (!configured());
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

uint8_t * USBDevice::findDescriptor(uint8_t descriptorType)
{
    /* Find a descriptor within the list of descriptors */
    /* following a configuration descriptor. */
    uint16_t wTotalLength;
    uint8_t *ptr;

    if (configurationDesc() == NULL)
    {
        return NULL;
    }

    /* Check this is a configuration descriptor */
    if ((configurationDesc()[0] != CONFIGURATION_DESCRIPTOR_LENGTH) \
            || (configurationDesc()[1] != CONFIGURATION_DESCRIPTOR))
    {
        return NULL;
    }

    wTotalLength = configurationDesc()[2] | (configurationDesc()[3] << 8);

    /* Check there are some more descriptors to follow */
    if (wTotalLength <= (CONFIGURATION_DESCRIPTOR_LENGTH+2))
    /* +2 is for bLength and bDescriptorType of next descriptor */
    {
        return false;
    }

    /* Start at first descriptor after the configuration descriptor */
    ptr = &(configurationDesc()[CONFIGURATION_DESCRIPTOR_LENGTH]);

    do {
        if (ptr[1] /* bDescriptorType */ == descriptorType)
        {
            /* Found */
            return ptr;
        }

        /* Skip to next descriptor */
        ptr += ptr[0]; /* bLength */
    } while (ptr < (configurationDesc() + wTotalLength));

    /* Reached end of the descriptors - not found */
    return NULL;
}


void USBDevice::connectStateChanged(unsigned int connected)
{
}

void USBDevice::suspendStateChanged(unsigned int suspended)
{
}


USBDevice::USBDevice(uint16_t vendor_id, uint16_t product_id, uint16_t product_release){
    VENDOR_ID = vendor_id;
    PRODUCT_ID = product_id;
    PRODUCT_RELEASE = product_release;

    /* Set initial device state */
    device.state = POWERED;
    device.configuration = 0;
    device.suspended = false;
};


bool USBDevice::readStart(uint8_t endpoint, uint16_t maxSize)
{
    return endpointRead(endpoint, maxSize) == EP_PENDING;
}


bool USBDevice::write(uint8_t endpoint, uint8_t * buffer, uint16_t size, uint16_t maxSize)
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
    do {
        result = endpointWriteResult(endpoint);
    } while ((result == EP_PENDING) && configured());

    return (result == EP_COMPLETED);
}


bool USBDevice::writeNB(uint8_t endpoint, uint8_t * buffer, uint16_t size, uint16_t maxSize)
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



bool USBDevice::readEP(uint8_t endpoint, uint8_t * buffer, uint16_t * size, uint16_t maxSize)
{
    EP_STATUS result;
    
    if(!configured()) {
        return false;
    }

    /* Wait for completion */
    do {
        result = endpointReadResult(endpoint, buffer, (uint32_t *)size);
    } while ((result == EP_PENDING) && configured());

    return (result == EP_COMPLETED);
}


bool USBDevice::readEP_NB(uint8_t endpoint, uint8_t * buffer, uint16_t * size, uint16_t maxSize)
{
    EP_STATUS result;
    
    if(!configured()) {
        return false;
    }

    result = endpointReadResult(endpoint, buffer, (uint32_t *)size);
    
    return (result == EP_COMPLETED);
}



uint8_t * USBDevice::deviceDesc() {
    static uint8_t deviceDescriptor[] = {
        DEVICE_DESCRIPTOR_LENGTH,       /* bLength */
        DEVICE_DESCRIPTOR,              /* bDescriptorType */
        LSB(USB_VERSION_2_0),           /* bcdUSB (LSB) */
        MSB(USB_VERSION_2_0),           /* bcdUSB (MSB) */
        0x00,                           /* bDeviceClass */
        0x00,                           /* bDeviceSubClass */
        0x00,                           /* bDeviceprotocol */
        MAX_PACKET_SIZE_EP0,            /* bMaxPacketSize0 */
        LSB(VENDOR_ID),                 /* idVendor (LSB) */
        MSB(VENDOR_ID),                 /* idVendor (MSB) */
        LSB(PRODUCT_ID),                /* idProduct (LSB) */
        MSB(PRODUCT_ID),                /* idProduct (MSB) */
        LSB(PRODUCT_RELEASE),           /* bcdDevice (LSB) */
        MSB(PRODUCT_RELEASE),           /* bcdDevice (MSB) */
        STRING_OFFSET_IMANUFACTURER,    /* iManufacturer */
        STRING_OFFSET_IPRODUCT,         /* iProduct */
        STRING_OFFSET_ISERIAL,          /* iSerialNumber */
        0x01                            /* bNumConfigurations */
    };
    return deviceDescriptor;
}

uint8_t * USBDevice::stringLangidDesc() {
    static uint8_t stringLangidDescriptor[] = {
        0x04,               /*bLength*/
        STRING_DESCRIPTOR,  /*bDescriptorType 0x03*/
        0x09,0x00,          /*bString Lang ID - 0x009 - English*/
    };
    return stringLangidDescriptor;
}

uint8_t * USBDevice::stringImanufacturerDesc() {
    static uint8_t stringImanufacturerDescriptor[] = {
        0x12,                                            /*bLength*/
        STRING_DESCRIPTOR,                               /*bDescriptorType 0x03*/
        'm',0,'b',0,'e',0,'d',0,'.',0,'o',0,'r',0,'g',0, /*bString iManufacturer - mbed.org*/
    };
    return stringImanufacturerDescriptor;
}

uint8_t * USBDevice::stringIserialDesc() {
    static uint8_t stringIserialDescriptor[] = {
        0x16,                                                           /*bLength*/
        STRING_DESCRIPTOR,                                              /*bDescriptorType 0x03*/
        '0',0,'1',0,'2',0,'3',0,'4',0,'5',0,'6',0,'7',0,'8',0,'9',0,    /*bString iSerial - 0123456789*/
    };
    return stringIserialDescriptor;
}

uint8_t * USBDevice::stringIConfigurationDesc() {
    static uint8_t stringIconfigurationDescriptor[] = {
        0x06,               /*bLength*/
        STRING_DESCRIPTOR,  /*bDescriptorType 0x03*/
        '0',0,'1',0,        /*bString iConfiguration - 01*/
    };
    return stringIconfigurationDescriptor;
}

uint8_t * USBDevice::stringIinterfaceDesc() {
    static uint8_t stringIinterfaceDescriptor[] = {
        0x08,               /*bLength*/
        STRING_DESCRIPTOR,  /*bDescriptorType 0x03*/
        'U',0,'S',0,'B',0,  /*bString iInterface - USB*/
    };
    return stringIinterfaceDescriptor;
}

uint8_t * USBDevice::stringIproductDesc() {
    static uint8_t stringIproductDescriptor[] = {
        0x16,                                                       /*bLength*/
        STRING_DESCRIPTOR,                                          /*bDescriptorType 0x03*/
        'U',0,'S',0,'B',0,' ',0,'D',0,'E',0,'V',0,'I',0,'C',0,'E',0 /*bString iProduct - USB DEVICE*/
    };
    return stringIproductDescriptor;
}
