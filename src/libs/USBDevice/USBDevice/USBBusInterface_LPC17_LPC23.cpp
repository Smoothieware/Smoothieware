// USBBusInterface_LPC17_LPC23.c
// USB Bus Interface for NXP LPC1768 and LPC2368
// Copyright (c) 2011 ARM Limited. All rights reserved.

#ifdef TARGET_LPC1768

#include "USBBusInterface.h"


// Get endpoint direction
#define IN_EP(endpoint)     ((endpoint) & 1U ? true : false)
#define OUT_EP(endpoint)    ((endpoint) & 1U ? false : true)

// Convert physical endpoint number to register bit
#define EP(endpoint) (1UL<<endpoint)

// Power Control for Peripherals register
#define PCUSB      (1UL<<31)

// USB Clock Control register
#define DEV_CLK_EN (1UL<<1)
#define AHB_CLK_EN (1UL<<4)

// USB Clock Status register
#define DEV_CLK_ON (1UL<<1)
#define AHB_CLK_ON (1UL<<4)

// USB Device Interupt registers
#define FRAME      (1UL<<0)
#define EP_FAST    (1UL<<1)
#define EP_SLOW    (1UL<<2)
#define DEV_STAT   (1UL<<3)
#define CCEMPTY    (1UL<<4)
#define CDFULL     (1UL<<5)
#define RxENDPKT   (1UL<<6)
#define TxENDPKT   (1UL<<7)
#define EP_RLZED   (1UL<<8)
#define ERR_INT    (1UL<<9)

// USB Control register
#define RD_EN (1<<0)
#define WR_EN (1<<1)
#define LOG_ENDPOINT(endpoint) ((endpoint>>1)<<2)

// USB Receive Packet Length register
#define DV      (1UL<<10)
#define PKT_RDY (1UL<<11)
#define PKT_LNGTH_MASK (0x3ff)

// Serial Interface Engine (SIE)
#define SIE_WRITE   (0x01)
#define SIE_READ    (0x02)
#define SIE_COMMAND (0x05)
#define SIE_CMD_CODE(phase, data) ((phase<<8)|(data<<16))

// SIE Command codes
#define SIE_CMD_SET_ADDRESS        (0xD0)
#define SIE_CMD_CONFIGURE_DEVICE   (0xD8)
#define SIE_CMD_SET_MODE           (0xF3)
#define SIE_CMD_READ_FRAME_NUMBER  (0xF5)
#define SIE_CMD_READ_TEST_REGISTER (0xFD)
#define SIE_CMD_SET_DEVICE_STATUS  (0xFE)
#define SIE_CMD_GET_DEVICE_STATUS  (0xFE)
#define SIE_CMD_GET_ERROR_CODE     (0xFF)
#define SIE_CMD_READ_ERROR_STATUS  (0xFB)

#define SIE_CMD_SELECT_ENDPOINT(endpoint)                 (0x00+endpoint)
#define SIE_CMD_SELECT_ENDPOINT_CLEAR_INTERRUPT(endpoint) (0x40+endpoint)
#define SIE_CMD_SET_ENDPOINT_STATUS(endpoint)             (0x40+endpoint)

#define SIE_CMD_CLEAR_BUFFER    (0xF2)
#define SIE_CMD_VALIDATE_BUFFER (0xFA)

// SIE Device Status register
#define SIE_DS_CON    (1<<0)
#define SIE_DS_CON_CH (1<<1)
#define SIE_DS_SUS    (1<<2)
#define SIE_DS_SUS_CH (1<<3)
#define SIE_DS_RST    (1<<4)

// SIE Device Set Address register
#define SIE_DSA_DEV_EN  (1<<7)

// SIE Configue Device register
#define SIE_CONF_DEVICE (1<<0)

// Select Endpoint register
#define SIE_SE_FE       (1<<0)
#define SIE_SE_ST       (1<<1)
#define SIE_SE_STP      (1<<2)
#define SIE_SE_PO       (1<<3)
#define SIE_SE_EPN      (1<<4)
#define SIE_SE_B_1_FULL (1<<5)
#define SIE_SE_B_2_FULL (1<<6)

// Set Endpoint Status command
#define SIE_SES_ST      (1<<0)
#define SIE_SES_DA      (1<<5)
#define SIE_SES_RF_MO   (1<<6)
#define SIE_SES_CND_ST  (1<<7)


USBHAL * USBHAL::instance;

volatile int epComplete;
uint32_t endpointStallState;

static void SIECommand(uint32_t command) {
    // The command phase of a SIE transaction
    LPC_USB->USBDevIntClr = CCEMPTY;
    LPC_USB->USBCmdCode = SIE_CMD_CODE(SIE_COMMAND, command);
    while (!(LPC_USB->USBDevIntSt & CCEMPTY));
}

static void SIEWriteData(uint8_t data) {
    // The data write phase of a SIE transaction
    LPC_USB->USBDevIntClr = CCEMPTY;
    LPC_USB->USBCmdCode = SIE_CMD_CODE(SIE_WRITE, data);
    while (!(LPC_USB->USBDevIntSt & CCEMPTY));
}

static uint8_t SIEReadData(uint32_t command) {
    // The data read phase of a SIE transaction
    LPC_USB->USBDevIntClr = CDFULL;
    LPC_USB->USBCmdCode = SIE_CMD_CODE(SIE_READ, command);
    while (!(LPC_USB->USBDevIntSt & CDFULL));
    return (uint8_t)LPC_USB->USBCmdData;
}

static void SIEsetDeviceStatus(uint8_t status) {
    // Write SIE device status register
    SIECommand(SIE_CMD_SET_DEVICE_STATUS);
    SIEWriteData(status);
}

static uint8_t SIEgetDeviceStatus(void) {
    // Read SIE device status register
    SIECommand(SIE_CMD_GET_DEVICE_STATUS);
    return SIEReadData(SIE_CMD_GET_DEVICE_STATUS);
}

void SIEsetAddress(uint8_t address) {
    // Write SIE device address register
    SIECommand(SIE_CMD_SET_ADDRESS);
    SIEWriteData((address & 0x7f) | SIE_DSA_DEV_EN);
}

static uint8_t SIEselectEndpoint(uint8_t endpoint) {
    // SIE select endpoint command
    SIECommand(SIE_CMD_SELECT_ENDPOINT(endpoint));
    return SIEReadData(SIE_CMD_SELECT_ENDPOINT(endpoint));
}

static uint8_t SIEclearBuffer(void) {
    // SIE clear buffer command
    SIECommand(SIE_CMD_CLEAR_BUFFER);
    return SIEReadData(SIE_CMD_CLEAR_BUFFER);
}

static void SIEvalidateBuffer(void) {
    // SIE validate buffer command
    SIECommand(SIE_CMD_VALIDATE_BUFFER);
}

static void SIEsetEndpointStatus(uint8_t endpoint, uint8_t status) {
    // SIE set endpoint status command
    SIECommand(SIE_CMD_SET_ENDPOINT_STATUS(endpoint));
    SIEWriteData(status);
}

static uint16_t SIEgetFrameNumber(void) __attribute__ ((unused));
static uint16_t SIEgetFrameNumber(void) {
    // Read current frame number
    uint16_t lowByte;
    uint16_t highByte;

    SIECommand(SIE_CMD_READ_FRAME_NUMBER);
    lowByte = SIEReadData(SIE_CMD_READ_FRAME_NUMBER);
    highByte = SIEReadData(SIE_CMD_READ_FRAME_NUMBER);

    return (highByte << 8) | lowByte;
}

static void SIEconfigureDevice(void) {
    // SIE Configure device command
    SIECommand(SIE_CMD_CONFIGURE_DEVICE);
    SIEWriteData(SIE_CONF_DEVICE);
}

static void SIEunconfigureDevice(void) {
    // SIE Configure device command
    SIECommand(SIE_CMD_CONFIGURE_DEVICE);
    SIEWriteData(0);
}

static void SIEconnect(void) {
    // Connect USB device
    uint8_t status;

    status = SIEgetDeviceStatus();
    SIEsetDeviceStatus(status | SIE_DS_CON);
}


static void SIEdisconnect(void) {
    // Disconnect USB device
    uint8_t status;

    status = SIEgetDeviceStatus();
    SIEsetDeviceStatus(status & ~SIE_DS_CON);
}


static uint8_t selectEndpointClearInterrupt(uint8_t endpoint) {
    // Implemented using using EP_INT_CLR.
    LPC_USB->USBEpIntClr = EP(endpoint);
    while (!(LPC_USB->USBDevIntSt & CDFULL));
    return (uint8_t)LPC_USB->USBCmdData;
}





static void enableEndpointEvent(uint8_t endpoint) {
    // Enable an endpoint interrupt
    LPC_USB->USBEpIntEn |= EP(endpoint);
}

static void disableEndpointEvent(uint8_t endpoint) __attribute__ ((unused));
static void disableEndpointEvent(uint8_t endpoint) {
    // Disable an endpoint interrupt
    LPC_USB->USBEpIntEn &= ~EP(endpoint);
}

static volatile uint32_t __attribute__((used)) dummyRead;


uint32_t USBHAL::endpointReadcore(uint8_t endpoint, uint8_t *buffer) {
    // Read from an OUT endpoint
    uint32_t size;
    uint32_t i;
    uint32_t data = 0;
    uint8_t offset;

    LPC_USB->USBCtrl = LOG_ENDPOINT(endpoint) | RD_EN;
    while (!(LPC_USB->USBRxPLen & PKT_RDY));

    size = LPC_USB->USBRxPLen & PKT_LNGTH_MASK;

    offset = 0;

    if (size > 0) {
        for (i=0; i<size; i++) {
            if (offset==0) {
                // Fetch up to four bytes of data as a word
                data = LPC_USB->USBRxData;
            }

            // extract a byte
            *buffer = (data>>offset) & 0xff;
            buffer++;

            // move on to the next byte
            offset = (offset + 8) % 32;
        }
    } else {
        dummyRead = LPC_USB->USBRxData;
    }

    LPC_USB->USBCtrl = 0;

    if ((endpoint >> 1) % 3 || (endpoint >> 1) == 0) {
        SIEselectEndpoint(endpoint);
        SIEclearBuffer();
    }
    
    return size;
}

static void endpointWritecore(uint8_t endpoint, uint8_t *buffer, uint32_t size) {
    // Write to an IN endpoint
    uint32_t temp, data;
    uint8_t offset;

    LPC_USB->USBCtrl = LOG_ENDPOINT(endpoint) | WR_EN;

    LPC_USB->USBTxPLen = size;
    offset = 0;
    data = 0;

    if (size>0) {
        do {
            // Fetch next data byte into a word-sized temporary variable
            temp = *buffer++;

            // Add to current data word
            temp = temp << offset;
            data = data | temp;

            // move on to the next byte
            offset = (offset + 8) % 32;
            size--;

            if ((offset==0) || (size==0)) {
                // Write the word to the endpoint
                LPC_USB->USBTxData = data;
                data = 0;
            }
        } while (size>0);
    } else {
        LPC_USB->USBTxData = 0;
    }

    // Clear WR_EN to cover zero length packet case
    LPC_USB->USBCtrl=0;

    SIEselectEndpoint(endpoint);
    SIEvalidateBuffer();
}







USBHAL::USBHAL(void) {
    // Disable IRQ
    NVIC_DisableIRQ(USB_IRQn);

    // Enable power to USB device controller
    LPC_SC->PCONP |= PCUSB;

    // Enable USB clocks
    LPC_USB->USBClkCtrl |= DEV_CLK_EN | AHB_CLK_EN;
    while (LPC_USB->USBClkSt != (DEV_CLK_ON | AHB_CLK_ON));

    // Configure pins P0.29 and P0.30 to be USB D+ and USB D-
    LPC_PINCON->PINSEL1 &= 0xc3ffffff;
    LPC_PINCON->PINSEL1 |= 0x14000000;

    // Disconnect USB device
    SIEdisconnect();

    // Configure pin P2.9 to be Connect
    LPC_PINCON->PINSEL4 &= 0xfffcffff;
    LPC_PINCON->PINSEL4 |= 0x00040000;

    // Connect must be low for at least 2.5uS
    wait(0.3);

    // Set the maximum packet size for the control endpoints
    realiseEndpoint(EP0IN, MAX_PACKET_SIZE_EP0, 0);
    realiseEndpoint(EP0OUT, MAX_PACKET_SIZE_EP0, 0);

    // Attach IRQ
    instance = this;
    NVIC_SetVector(USB_IRQn, (uint32_t)&_usbisr);
    NVIC_EnableIRQ(USB_IRQn);

    // Enable interrupts for device events and EP0
    LPC_USB->USBDevIntEn = EP_SLOW | DEV_STAT | FRAME;
    enableEndpointEvent(EP0IN);
    enableEndpointEvent(EP0OUT);
}

USBHAL::~USBHAL(void) {
    // Ensure device disconnected
    SIEdisconnect();

    // Disable USB interrupts
    NVIC_DisableIRQ(USB_IRQn);
}

void USBHAL::connect(void) {
    // Connect USB device
    SIEconnect();
}

void USBHAL::disconnect(void) {
    // Disconnect USB device
    SIEdisconnect();
}

void USBHAL::configureDevice(void) {
    SIEconfigureDevice();
}

void USBHAL::unconfigureDevice(void) {
    SIEunconfigureDevice();
}

void USBHAL::setAddress(uint8_t address) {
    SIEsetAddress(address);
}

void USBHAL::EP0setup(uint8_t *buffer) {
    endpointReadcore(EP0OUT, buffer);
}

void USBHAL::EP0read(void) {
    // Not required
}

uint32_t USBHAL::EP0getReadResult(uint8_t *buffer) {
    return endpointReadcore(EP0OUT, buffer);
}

void USBHAL::EP0write(uint8_t *buffer, uint32_t size) {
    endpointWritecore(EP0IN, buffer, size);
}

void USBHAL::EP0getWriteResult(void) {
    // Not required
}

void USBHAL::EP0stall(void) {
    // This will stall both control endpoints
    stallEndpoint(EP0OUT);
}

EP_STATUS USBHAL::endpointRead(uint8_t endpoint, uint32_t maximumSize) {
    return EP_PENDING;
}

EP_STATUS USBHAL::endpointReadResult(uint8_t endpoint, uint8_t * buffer, uint32_t *bytesRead) {

    //for isochronous endpoint, we don't wait an interrupt
    if ((endpoint >> 1) % 3 || (endpoint >> 1) == 0) {
        if (!(epComplete & EP(endpoint)))
            return EP_PENDING;
    }
    
    *bytesRead = endpointReadcore(endpoint, buffer);
    epComplete &= ~EP(endpoint);
    return EP_COMPLETED;
}

EP_STATUS USBHAL::endpointWrite(uint8_t endpoint, uint8_t *data, uint32_t size) {
    if (getEndpointStallState(endpoint)) {
        return EP_STALLED;
    }

    epComplete &= ~EP(endpoint);

    endpointWritecore(endpoint, data, size);
    return EP_PENDING;
}

EP_STATUS USBHAL::endpointWriteResult(uint8_t endpoint) {
    if (epComplete & EP(endpoint)) {
        epComplete &= ~EP(endpoint);
        return EP_COMPLETED;
    }

    return EP_PENDING;
}

bool USBHAL::realiseEndpoint(uint8_t endpoint, uint32_t maxPacket, uint32_t flags) {
    // Realise an endpoint
    LPC_USB->USBDevIntClr = EP_RLZED;
    LPC_USB->USBReEp |= EP(endpoint);
    LPC_USB->USBEpInd = endpoint;
    LPC_USB->USBMaxPSize = maxPacket;

    while (!(LPC_USB->USBDevIntSt & EP_RLZED));
    LPC_USB->USBDevIntClr = EP_RLZED;

    // Clear stall state
    endpointStallState &= ~EP(endpoint);

    enableEndpointEvent(endpoint);
    return true;
}

void USBHAL::stallEndpoint(uint8_t endpoint) {
    // Stall an endpoint
    if ( (endpoint==EP0IN) || (endpoint==EP0OUT) ) {
        // Conditionally stall both control endpoints
        SIEsetEndpointStatus(EP0OUT, SIE_SES_CND_ST);
    } else {
        SIEsetEndpointStatus(endpoint, SIE_SES_ST);

        // Update stall state
        endpointStallState |= EP(endpoint);
    }
}

void USBHAL::unstallEndpoint(uint8_t endpoint) {
    // Unstall an endpoint. The endpoint will also be reinitialised
    SIEsetEndpointStatus(endpoint, 0);

    // Update stall state
    endpointStallState &= ~EP(endpoint);
}

bool USBHAL::getEndpointStallState(uint8_t endpoint) {
    // Returns true if endpoint stalled
    return endpointStallState & EP(endpoint);
}

void USBHAL::remoteWakeup(void) {
    // Remote wakeup
    uint8_t status;

    // Enable USB clocks
    LPC_USB->USBClkCtrl |= DEV_CLK_EN | AHB_CLK_EN;
    while (LPC_USB->USBClkSt != (DEV_CLK_ON | AHB_CLK_ON));

    status = SIEgetDeviceStatus();
    SIEsetDeviceStatus(status & ~SIE_DS_SUS);
}





void USBHAL::_usbisr(void) {
    instance->usbisr();
}


void USBHAL::usbisr(void) {
    uint8_t devStat;

    if (LPC_USB->USBDevIntSt & FRAME) {
        // Start of frame event
        SOF(SIEgetFrameNumber());
        // Clear interrupt status flag
        LPC_USB->USBDevIntClr = FRAME;
    }

    if (LPC_USB->USBDevIntSt & DEV_STAT) {
        // Device Status interrupt
        // Must clear the interrupt status flag before reading the device status from the SIE
        LPC_USB->USBDevIntClr = DEV_STAT;

        // Read device status from SIE
        devStat = SIEgetDeviceStatus();

        if (devStat & SIE_DS_RST) {
            // Bus reset
            busReset();
        }
    }

    if (LPC_USB->USBDevIntSt & EP_SLOW) {
        // (Slow) Endpoint Interrupt

        // Process each endpoint interrupt
        if (LPC_USB->USBEpIntSt & EP(EP0OUT)) {
            if (selectEndpointClearInterrupt(EP0OUT) & SIE_SE_STP) {
                // this is a setup packet
                EP0setupCallback();
            } else {
                EP0out();
            }
            LPC_USB->USBDevIntClr = EP_SLOW;
        }

        if (LPC_USB->USBEpIntSt & EP(EP0IN)) {
            selectEndpointClearInterrupt(EP0IN);
            LPC_USB->USBDevIntClr = EP_SLOW;
            EP0in();
        }

        // TODO: This should cover all endpoints, not just EP1,2,3:
        if (LPC_USB->USBEpIntSt & EP(EP1IN)) {
            selectEndpointClearInterrupt(EP1IN);
            epComplete |= EP(EP1IN);
            LPC_USB->USBDevIntClr = EP_SLOW;
            if (EP1_IN_callback())
                epComplete &= ~EP(EP1IN);
        }

        if (LPC_USB->USBEpIntSt & EP(EP1OUT)) {
            selectEndpointClearInterrupt(EP1OUT);
            epComplete |= EP(EP1OUT);
            LPC_USB->USBDevIntClr = EP_SLOW;
            if (EP1_OUT_callback())
                epComplete &= ~EP(EP1OUT);
        }

        if (LPC_USB->USBEpIntSt & EP(EP2IN)) {
            selectEndpointClearInterrupt(EP2IN);
            epComplete |= EP(EP2IN);
            LPC_USB->USBDevIntClr = EP_SLOW;
            if (EP2_IN_callback())
                epComplete &= ~EP(EP2IN);
        }

        if (LPC_USB->USBEpIntSt & EP(EP2OUT)) {
            selectEndpointClearInterrupt(EP2OUT);
            epComplete |= EP(EP2OUT);
            LPC_USB->USBDevIntClr = EP_SLOW;
            if (EP2_OUT_callback())
                epComplete &= ~EP(EP2OUT);
        }

        if (LPC_USB->USBEpIntSt & EP(EP3IN)) {
            selectEndpointClearInterrupt(EP3IN);
            epComplete |= EP(EP3IN);
            LPC_USB->USBDevIntClr = EP_SLOW;
            if (EP3_IN_callback())
                epComplete &= ~EP(EP3IN);
        }

        if (LPC_USB->USBEpIntSt & EP(EP3OUT)) {
            selectEndpointClearInterrupt(EP3OUT);
            epComplete |= EP(EP3OUT);
            LPC_USB->USBDevIntClr = EP_SLOW;
            if (EP3_OUT_callback())
                epComplete &= ~EP(EP3OUT);
        }

        if (LPC_USB->USBEpIntSt & EP(EP5IN)) {
            selectEndpointClearInterrupt(EP5IN);
            epComplete |= EP(EP5IN);
            LPC_USB->USBDevIntClr = EP_SLOW;
            if (EP5_IN_callback())
                epComplete &= ~EP(EP5IN);
        }

        if (LPC_USB->USBEpIntSt & EP(EP5OUT)) {
            selectEndpointClearInterrupt(EP5OUT);
            epComplete |= EP(EP5OUT);
            LPC_USB->USBDevIntClr = EP_SLOW;
            if (EP5_OUT_callback())
                epComplete &= ~EP(EP5OUT);
        }
    }
}

#endif
