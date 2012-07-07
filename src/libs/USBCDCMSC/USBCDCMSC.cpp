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

#include "stdint.h"
#include "stdarg.h"
#include "USBCDCMSC.h"
#include "USBBusInterface.h"
#include "libs/SerialMessage.h"

#include "libs/StreamOutput.h"
#include "libs/RingBuffer.h"

static uint8_t cdc_line_coding[7]= {0x80, 0x25, 0x00, 0x00, 0x00, 0x00, 0x08};

#define DEFAULT_CONFIGURATION (1)

#define CDC_SET_LINE_CODING        0x20
#define CDC_GET_LINE_CODING        0x21
#define CDC_SET_CONTROL_LINE_STATE 0x22
#define CDC_SEND_BREAK             0x23

#define MAX_CDC_REPORT_SIZE MAX_PACKET_SIZE_EPBULK



#define DISK_OK         0x00
#define NO_INIT         0x01
#define NO_DISK         0x02
#define WRITE_PROTECT   0x04

#define CBW_Signature   0x43425355
#define CSW_Signature   0x53425355

// SCSI Commands
#define TEST_UNIT_READY            0x00
#define REQUEST_SENSE              0x03
#define FORMAT_UNIT                0x04
#define INQUIRY                    0x12
#define MODE_SELECT6               0x15
#define MODE_SENSE6                0x1A
#define START_STOP_UNIT            0x1B
#define MEDIA_REMOVAL              0x1E
#define READ_FORMAT_CAPACITIES     0x23
#define READ_CAPACITY              0x25
#define READ10                     0x28
#define WRITE10                    0x2A
#define VERIFY10                   0x2F
#define READ12                     0xA8
#define WRITE12                    0xAA
#define MODE_SELECT10              0x55
#define MODE_SENSE10               0x5A

// MSC class specific requests
#define MSC_REQUEST_RESET          0xFF
#define MSC_REQUEST_GET_MAX_LUN    0xFE

#define DEFAULT_CONFIGURATION (1)

// Sense codes
#define SENSE_NO_SENSE          0x00
#define SENSE_RECOVERED_ERROR   0x01
#define SENSE_NOT_READY         0x02
#define SENSE_MEDIUM_ERROR      0x03
#define SENSE_HARDWARE_ERROR    0x04
#define SENSE_ILLEGAL_REQUEST   0x05
#define SENSE_UNIT_ATTENTION    0x06
#define SENSE_DATA_PROTECT      0x07
#define SENSE_BLANK_CHECK       0x08
#define SENSE_ABORTED_COMMAND   0x0B
#define SENSE_VOLUME_OVERFLOW   0x0D
#define SENSE_MISCOMPARE        0x0E

// max packet size
#define MAX_PACKET  MAX_PACKET_SIZE_EPBULK

// CSW Status
enum Status {
    CSW_PASSED,
    CSW_FAILED,
    CSW_ERROR,
};


USBCDCMSC::USBCDCMSC(SDFileSystem *sd, uint16_t vendor_id, uint16_t product_id, uint16_t product_release): USBDevice(vendor_id, product_id, product_release), cdcbuf(128), _sd(sd) {
    cdcbreak = 0;
    _status = NO_INIT;
    connect();
//    USBDevice::connect();
	setSense(SENSE_NO_SENSE, 0x00, 0x00);
    USBHAL::connect();
}

bool USBCDCMSC::USBCallback_request(void) {
    /* Called in ISR context */

    bool success = false;
    CONTROL_TRANSFER * transfer = getTransferPtr();
    static uint8_t maxLUN[1] = {0};

    /* Process class-specific requests */

    if (transfer->setup.bmRequestType.Type == CLASS_TYPE) {
        switch (transfer->setup.bRequest) {
            case CDC_GET_LINE_CODING:
                transfer->remaining = 7;
                transfer->ptr = cdc_line_coding;
                transfer->direction = DEVICE_TO_HOST;
                success = true;
                break;
            case CDC_SET_LINE_CODING:
                transfer->remaining = 7;
                success = true;
                break;
            case CDC_SET_CONTROL_LINE_STATE:
                success = true;
                break;
            case CDC_SEND_BREAK:
                cdcbreak = 1;
                success = true;
                break;
            case MSC_REQUEST_RESET:
                reset();
                success = true;
                break;
            case MSC_REQUEST_GET_MAX_LUN:
                transfer->remaining = 1;
                transfer->ptr = maxLUN;
                transfer->direction = DEVICE_TO_HOST;
                success = true;
                break;
            default:
                break;
        }
    }

    return success;
}


// Called in ISR context
// Set configuration. Return false if the
// configuration is not supported.
bool USBCDCMSC::USBCallback_setConfiguration(uint8_t configuration) {
    if (configuration != DEFAULT_CONFIGURATION) {
        return false;
    }

    // Configure endpoints > 0
    addEndpoint(EPINT_IN, MAX_PACKET_SIZE_EPINT);
    addEndpoint(EPBULK_IN, MAX_PACKET_SIZE_EPBULK);
    addEndpoint(EPBULK_OUT, MAX_PACKET_SIZE_EPBULK);

    // Configure endpoints > 0
    addEndpoint(MSDBULK_IN, MAX_PACKET_SIZE_MSDBULK);
    addEndpoint(MSDBULK_OUT, MAX_PACKET_SIZE_MSDBULK);

    // We activate the endpoint to be able to recceive data
    readStart(EPBULK_OUT, MAX_PACKET_SIZE_EPBULK);

    //activate readings
    readStart(MSDBULK_OUT, MAX_PACKET_SIZE_MSDBULK);
    return true;
}

bool USBCDCMSC::send(uint8_t * buffer, uint16_t size) {
    return USBDevice::write(EPBULK_IN, buffer, size, MAX_CDC_REPORT_SIZE);
}

bool USBCDCMSC::readEP(uint8_t * buffer, uint16_t * size) {
    if (!USBDevice::readEP(EPBULK_OUT, buffer, size, MAX_CDC_REPORT_SIZE))
        return false;
    if (!readStart(EPBULK_OUT, MAX_CDC_REPORT_SIZE))
        return false;
    return true;
}

bool USBCDCMSC::readEP_NB(uint8_t * buffer, uint16_t * size) {
    if (!USBDevice::readEP_NB(EPBULK_OUT, buffer, size, MAX_CDC_REPORT_SIZE))
        return false;
    if (!readStart(EPBULK_OUT, MAX_CDC_REPORT_SIZE))
        return false;
    return true;
}


uint8_t * USBCDCMSC::deviceDesc() {
    static uint8_t deviceDescriptor[] = {
        18,                   // bLength
        1,                    // bDescriptorType
        0x10, 0x01,           // bcdUSB
        0xef,                    // bDeviceClass
        0x02,                    // bDeviceSubClass
        0x01,                    // bDeviceProtocol
        MAX_PACKET_SIZE_EP0,  // bMaxPacketSize0
        LSB(VENDOR_ID), MSB(VENDOR_ID),  // idVendor
        LSB(PRODUCT_ID), MSB(PRODUCT_ID),// idProduct
        0x00, 0x01,           // bcdDevice
        1,                    // iManufacturer
        2,                    // iProduct
        3,                    // iSerialNumber
        1                     // bNumConfigurations
    };
    return deviceDescriptor;
}

uint8_t * USBCDCMSC::stringIinterfaceDesc() {
    static uint8_t stringIinterfaceDescriptor[] = {
        0x0e,
        STRING_DESCRIPTOR,
        'C',0,'D',0,'C',0,'M',0,'S',0,'C',0,
    };
    return stringIinterfaceDescriptor;
}

uint8_t * USBCDCMSC::stringIproductDesc() {
    static uint8_t stringIproductDescriptor[] = {
        0x1c,
        STRING_DESCRIPTOR,
        'C',0,'D',0,'C',0,'M',0,'S',0,'C',0,' ',0,'D',0,'E',0,'V',0,'I',0,'C',0,'E',0
    };
    return stringIproductDescriptor;
}


uint8_t * USBCDCMSC::configurationDesc() {
    static uint8_t configDescriptor[] = {
        9,                      // bLength;
        2,                      // bDescriptorType;
        LSB(0x62),              // wTotalLength
        MSB(0x62),
        3,                      // bNumInterfaces
        1,                      // bConfigurationValue
        0,                      // iConfiguration
        0xc0,                   // bmAttributes
        50,                     // bMaxPower

        // IAD
//        0x08, 0x0B, 0x00, 0x02, 0x02, 0x00, 0x00, 0x00,
        0x08, 0x0B, 0x00, 0x02, 0x02, 0x02, 0x01, 0x00,

        // interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12
        9,                      // bLength
        4,                      // bDescriptorType
        0,                      // bInterfaceNumber
        0,                      // bAlternateSetting
        1,                      // bNumEndpoints
        0x02,                   // bInterfaceClass
        0x02,                   // bInterfaceSubClass
        0x01,                   // bInterfaceProtocol
        0,                      // iInterface

        // CDC Header Functional Descriptor, CDC Spec 5.2.3.1, Table 26
        5,                      // bFunctionLength
        0x24,                   // bDescriptorType
        0x00,                   // bDescriptorSubtype
        0x10, 0x01,             // bcdCDC

        // Call Management Functional Descriptor, CDC Spec 5.2.3.2, Table 27
        5,                      // bFunctionLength
        0x24,                   // bDescriptorType
        0x01,                   // bDescriptorSubtype
        0x03,                   // bmCapabilities
        1,                      // bDataInterface

        // Abstract Control Management Functional Descriptor, CDC Spec 5.2.3.3, Table 28
        4,                      // bFunctionLength
        0x24,                   // bDescriptorType
        0x02,                   // bDescriptorSubtype
        0x06,                   // bmCapabilities

        // Union Functional Descriptor, CDC Spec 5.2.3.8, Table 33
        5,                      // bFunctionLength
        0x24,                   // bDescriptorType
        0x06,                   // bDescriptorSubtype
        0,                      // bMasterInterface
        1,                      // bSlaveInterface0

        // endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
        ENDPOINT_DESCRIPTOR_LENGTH,     // bLength
        ENDPOINT_DESCRIPTOR,            // bDescriptorType
        PHY_TO_DESC(EPINT_IN),          // bEndpointAddress
        E_INTERRUPT,                    // bmAttributes (0x03=intr)
        LSB(MAX_PACKET_SIZE_EPINT),     // wMaxPacketSize (LSB)
        MSB(MAX_PACKET_SIZE_EPINT),     // wMaxPacketSize (MSB)
        16,                             // bInterval




        // interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12
        9,          // bLength
        4,          // bDescriptorType
        1,          // bInterfaceNumber
        0,          // bAlternateSetting
        2,          // bNumEndpoints
        0x0A,       // bInterfaceClass
        0x00,       // bInterfaceSubClass
        0x00,       // bInterfaceProtocol
        0,          // iInterface

        // endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
        7,                      // bLength
        5,                      // bDescriptorType
        PHY_TO_DESC(EPBULK_IN), // bEndpointAddress
        0x02,                   // bmAttributes (0x02=bulk)
        LSB(MAX_PACKET_SIZE_EPBULK),    // wMaxPacketSize (LSB)
        MSB(MAX_PACKET_SIZE_EPBULK),    // wMaxPacketSize (MSB)
        0,                      // bInterval

        // endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
        7,                      // bLength
        5,                      // bDescriptorType
        PHY_TO_DESC(EPBULK_OUT),// bEndpointAddress
        0x02,                   // bmAttributes (0x02=bulk)
        LSB(MAX_PACKET_SIZE_EPBULK),    // wMaxPacketSize (LSB)
        MSB(MAX_PACKET_SIZE_EPBULK),     // wMaxPacketSize (MSB)
        0,                       // bInterval

        // Interface 2, Alternate Setting 0, MSC Class
        9,      // bLength
        4,      // bDescriptorType
        0x02,   // bInterfaceNumber
        0x00,   // bAlternateSetting
        0x02,   // bNumEndpoints
        0x08,   // bInterfaceClass
        0x06,   // bInterfaceSubClass
        0x50,   // bInterfaceProtocol
        0x04,   // iInterface

        // endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
        7,                          // bLength
        5,                          // bDescriptorType
        PHY_TO_DESC(MSDBULK_IN),     // bEndpointAddress
        0x02,                       // bmAttributes (0x02=bulk)
        LSB(MAX_PACKET_SIZE_EPBULK),// wMaxPacketSize (LSB)
        MSB(MAX_PACKET_SIZE_EPBULK),// wMaxPacketSize (MSB)
        0,                          // bInterval

        // endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
        7,                          // bLength
        5,                          // bDescriptorType
        PHY_TO_DESC(MSDBULK_OUT),    // bEndpointAddress
        0x02,                       // bmAttributes (0x02=bulk)
        LSB(MAX_PACKET_SIZE_EPBULK),// wMaxPacketSize (LSB)
        MSB(MAX_PACKET_SIZE_EPBULK),// wMaxPacketSize (MSB)
        0                           // bInterval
    };
    return configDescriptor;
}

int USBCDCMSC::_putc(int c) {
    send((uint8_t *)&c, 1);
    return 1;
}

int USBCDCMSC::_getc() {
    uint8_t c;
    while (cdcbuf.isEmpty());
    cdcbuf.dequeue(&c);
    return c;
}

int USBCDCMSC::printf(const char* format, ...) {
	va_list args;
	int slen;
	uint8_t *result, *current;

	va_start (args, format);
	slen = vasprintf ((char **)&result, format, args);
	va_end (args);

	current = result;

	// Send full-size packets as many times as needed
	while(slen > MAX_CDC_REPORT_SIZE) {
		send(current, MAX_CDC_REPORT_SIZE);
		current += MAX_CDC_REPORT_SIZE;
		slen -= MAX_CDC_REPORT_SIZE;
	}

	// send a full packet followed by Zero Length Packet
	if(slen == MAX_CDC_REPORT_SIZE) {
		send(current, MAX_CDC_REPORT_SIZE);
		send(current, 0);
	}

	// send a partial packet if needed to finish
	if(slen < MAX_CDC_REPORT_SIZE) {
		send(current, slen);
	}
	free(result);
	return 0;
}

bool USBCDCMSC::writeBlock(uint8_t * buf, uint16_t size) {
    if(size > MAX_PACKET_SIZE_EPBULK) {
        return false;
    }
    if(!send(buf, size)) {
        return false;
    }
    return true;
}



bool USBCDCMSC::EP2_OUT_callback() {
    uint8_t c[65];
    uint16_t size = 0;

    //we read the packet received and put it on the circular buffer
    readEP(c, &size);
    for (int i = 0; i < size; i++) {
        cdcbuf.queue(c[i]);
    }

    //call a potential handler
    rx.call();

    // We reactivate the endpoint to receive next characters
    readStart(EPBULK_OUT, MAX_PACKET_SIZE_EPBULK);
    return true;
}

uint8_t USBCDCMSC::available() {
    return cdcbuf.available();
}


bool USBCDCMSC::connect() {

    //disk initialization
    if (disk_status() & NO_INIT) {
        if (disk_initialize()) {
            return false;
        }
    }

    // get number of blocks
    BlockCount = disk_sectors();

    // get memory size
    MemorySize = disk_size();

    if (BlockCount >= 0) {
        BlockSize = MemorySize / BlockCount;
        if (BlockSize != 0) {
            page = (uint8_t *)malloc(BlockSize * sizeof(uint8_t));
            if (page == NULL)
                return false;
        }
    } else {
        return false;
    }

    //connect the device
//    USBDevice::connect();
    return true;
}


void USBCDCMSC::reset() {
    stage = READ_CBW;
}


// Called in ISR context called when a data is received
bool USBCDCMSC::EP5_OUT_callback() {
    uint16_t size = 0;
    uint8_t buf[MAX_PACKET_SIZE_EPBULK];
    USBDevice::readEP(MSDBULK_OUT, buf, &size, MAX_PACKET_SIZE_EPBULK);
    switch (stage) {
            // the device has to decode the CBW received
        case READ_CBW:
            CBWDecode(buf, size);
            break;

            // the device has to receive data from the host
        case PROCESS_CBW:
            switch (cbw.CB[0]) {
                case WRITE10:
                case WRITE12:
                    memoryWrite(buf, size);
                    break;
                case VERIFY10:
                    memoryVerify(buf, size);
                    break;
            }
            break;

            // an error has occured: stall endpoint and send CSW
        default:
            stallEndpoint(MSDBULK_OUT);
            csw.Status = CSW_ERROR;
            sendCSW();
            break;
    }

    //reactivate readings on the OUT bulk endpoint
    readStart(MSDBULK_OUT, MAX_PACKET_SIZE_EPBULK);
    return true;
}

// Called in ISR context when a data has been transferred
bool USBCDCMSC::EP5_IN_callback() {
    switch (stage) {

            // the device has to send data to the host
        case PROCESS_CBW:
            switch (cbw.CB[0]) {
                case READ10:
                case READ12:
                    memoryRead();
                    break;
            }
            break;

            //the device has to send a CSW
        case SEND_CSW:
            sendCSW();
            break;

            // an error has occured
        case ERROR:
            stallEndpoint(MSDBULK_IN);
            sendCSW();
            break;

            // the host has received the CSW -> we wait a CBW
        case WAIT_CSW:
            stage = READ_CBW;
            break;
    }
    return true;
}


void USBCDCMSC::memoryWrite (uint8_t * buf, uint16_t size) {

    if ((addr + size) > MemorySize) {
        size = MemorySize - addr;
        stage = ERROR;
        stallEndpoint(MSDBULK_OUT);
    }

    // we fill an array in RAM of 1 block before writing it in memory
    for (int i = 0; i < size; i++)
        page[addr%BlockSize + i] = buf[i];

    // if the array is filled, write it in memory
    if (!((addr + size)%BlockSize)) {
        if (!(disk_status() & WRITE_PROTECT)) {
            disk_write((const char *)page, addr/BlockSize);
        }
    }

    addr += size;
    length -= size;
    csw.DataResidue -= size;

    if ((!length) || (stage != PROCESS_CBW)) {
        csw.Status = (stage == ERROR) ? CSW_FAILED : CSW_PASSED;
        sendCSW();
    }
}

void USBCDCMSC::memoryVerify (uint8_t * buf, uint16_t size) {
    uint32_t n;

    if ((addr + size) > MemorySize) {
        size = MemorySize - addr;
        stage = ERROR;
        stallEndpoint(MSDBULK_OUT);
    }

    // beginning of a new block -> load a whole block in RAM
    if (!(addr%BlockSize))
        disk_read((char *)page, addr/BlockSize);

    // info are in RAM -> no need to re-read memory
    for (n = 0; n < size; n++) {
        if (page[addr%BlockSize + n] != buf[n]) {
            memOK = false;
            break;
        }
    }

    addr += size;
    length -= size;
    csw.DataResidue -= size;

    if ( !length || (stage != PROCESS_CBW)) {
        csw.Status = (memOK && (stage == PROCESS_CBW)) ? CSW_PASSED : CSW_FAILED;
        sendCSW();
    }
}


bool USBCDCMSC::inquiryRequest (void) {
    uint8_t inquiry[] = { 0x00, 0x80, 0x00, 0x01,
                          36 - 4, 0x80, 0x00, 0x00,
                          'M', 'B', 'E', 'D', '.', 'O', 'R', 'G',
                          'M', 'B', 'E', 'D', ' ', 'U', 'S', 'B', ' ', 'D', 'I', 'S', 'K', ' ', ' ', ' ',
                          '1', '.', '0', ' ',
                        };
    if (!msd_write(inquiry, sizeof(inquiry))) {
        return false;
    }
    return true;
}


bool USBCDCMSC::readFormatCapacity() {
    uint8_t capacity[] = { 0x00, 0x00, 0x00, 0x08,
                           (BlockCount >> 24) & 0xff,
                           (BlockCount >> 16) & 0xff,
                           (BlockCount >> 8) & 0xff,
                           (BlockCount >> 0) & 0xff,

                           0x02,
                           (BlockSize >> 16) & 0xff,
                           (BlockSize >> 8) & 0xff,
                           (BlockSize >> 0) & 0xff,
                         };
    if (!msd_write(capacity, sizeof(capacity))) {
        return false;
    }
    return true;
}


bool USBCDCMSC::readCapacity (void) {
    uint8_t capacity[] = {
        ((BlockCount - 1) >> 24) & 0xff,
        ((BlockCount - 1) >> 16) & 0xff,
        ((BlockCount - 1) >> 8) & 0xff,
        ((BlockCount - 1) >> 0) & 0xff,

        (BlockSize >> 24) & 0xff,
        (BlockSize >> 16) & 0xff,
        (BlockSize >> 8) & 0xff,
        (BlockSize >> 0) & 0xff,
    };
    if (!msd_write(capacity, sizeof(capacity))) {
        return false;
    }
    return true;
}

bool USBCDCMSC::msd_write (uint8_t * buf, uint16_t size) {

    if (size >= cbw.DataLength) {
        size = cbw.DataLength;
    }
    stage = SEND_CSW;

    if (!writeNB(MSDBULK_IN, buf, size, MAX_PACKET_SIZE_EPBULK)) {
        return false;
    }

    csw.DataResidue -= size;
    csw.Status = CSW_PASSED;
    return true;
}


bool USBCDCMSC::modeSense6 (void) {
    uint8_t sense6[] = { 0x03, 0x00, 0x00, 0x00 };
    if (!msd_write(sense6, sizeof(sense6))) {
        return false;
    }
    return true;
}

void USBCDCMSC::sendCSW() {
    csw.Signature = CSW_Signature;
    writeNB(MSDBULK_IN, (uint8_t *)&csw, sizeof(CSW), MAX_PACKET_SIZE_EPBULK);
    stage = WAIT_CSW;
}

void USBCDCMSC::setSense (uint8_t sense_key, uint8_t asc, uint8_t ascq) {
    sense.error = 0x70;
    sense.sense_key = sense_key;
    sense.additional_sense_length = 0x0a;
    sense.asc = asc;
    sense.ascq = ascq;    
}

bool USBCDCMSC::requestSense (void) {
    if (!msd_write((uint8_t *)&sense, sizeof(sense))) {
        return false;
    }
	
    return true;
}

void USBCDCMSC::mediaRemoval() {
	 csw.Status = CSW_PASSED;
	 sendCSW();
}

void USBCDCMSC::fail() {
    csw.Status = CSW_FAILED;
    sendCSW();
}


void USBCDCMSC::CBWDecode(uint8_t * buf, uint16_t size) {
    if (size == sizeof(cbw)) {
        memcpy((uint8_t *)&cbw, buf, size);
        if (cbw.Signature == CBW_Signature) {
            csw.Tag = cbw.Tag;
            csw.DataResidue = cbw.DataLength;
            if ((cbw.CBLength <  1) || (cbw.CBLength > 16) ) {
                fail();
            } else {
                switch (cbw.CB[0]) {
                    case TEST_UNIT_READY:
                        testUnitReady();
                        break;
                    case REQUEST_SENSE:
                        requestSense();
                        break;
                    case INQUIRY:
                        inquiryRequest();
                        break;
                    case MODE_SENSE6:
                        modeSense6();
                        break;
                    case READ_FORMAT_CAPACITIES:
                        readFormatCapacity();
                        break;
	                case MEDIA_REMOVAL:
	                    mediaRemoval();
						break;
                    case READ_CAPACITY:
                        readCapacity();
                        break;
                    case READ10:
                    case READ12:
                        if (infoTransfer()) {
                            if ((cbw.Flags & 0x80)) {
                                stage = PROCESS_CBW;
                                memoryRead();
                            } else {
                                stallEndpoint(MSDBULK_OUT);
                                csw.Status = CSW_ERROR;
                                sendCSW();
                            }
                        }
                        break;
                    case WRITE10:
                    case WRITE12:
                        if (infoTransfer()) {
                            if (!(cbw.Flags & 0x80)) {
                                stage = PROCESS_CBW;
                            } else {
                                stallEndpoint(MSDBULK_IN);
                                csw.Status = CSW_ERROR;
                                sendCSW();
                            }
                        }
                        break;
                    case VERIFY10:
                        if (!(cbw.CB[1] & 0x02)) {
                            csw.Status = CSW_PASSED;
                            sendCSW();
                            break;
                        }
                        if (infoTransfer()) {
                            if (!(cbw.Flags & 0x80)) {
                                stage = PROCESS_CBW;
                                memOK = true;
                            } else {
                                stallEndpoint(MSDBULK_IN);
                                csw.Status = CSW_ERROR;
                                sendCSW();
                            }
                        }
                        break;
                    default:
                        fail();
                        break;
                }
            }
        }
    }
}

void USBCDCMSC::testUnitReady (void) {

    if (cbw.DataLength != 0) {
        if ((cbw.Flags & 0x80) != 0) {
            stallEndpoint(MSDBULK_IN);
        } else {
            stallEndpoint(MSDBULK_OUT);
        }
    }

    csw.Status = CSW_PASSED;
    sendCSW();
}


void USBCDCMSC::memoryRead (void) {
    uint32_t n;

    n = (length > MAX_PACKET) ? MAX_PACKET : length;

    if ((addr + n) > MemorySize) {
        n = MemorySize - addr;
        stage = ERROR;
    }

    // we read an entire block
    if (!(addr%BlockSize))
        disk_read((char *)page, addr/BlockSize);

    // write data which are in RAM
    writeNB(MSDBULK_IN, &page[addr%BlockSize], n, MAX_PACKET_SIZE_EPBULK);

    addr += n;
    length -= n;

    csw.DataResidue -= n;

    if ( !length || (stage != PROCESS_CBW)) {
        csw.Status = (stage == PROCESS_CBW) ? CSW_PASSED : CSW_FAILED;
        stage = (stage == PROCESS_CBW) ? SEND_CSW : stage;
    }
}


bool USBCDCMSC::infoTransfer (void) {
    uint32_t n;

    // Logical Block Address of First Block
    n = (cbw.CB[2] << 24) | (cbw.CB[3] << 16) | (cbw.CB[4] <<  8) | (cbw.CB[5] <<  0);

    addr = n * BlockSize;

    // Number of Blocks to transfer
    switch (cbw.CB[0]) {
        case READ10:
        case WRITE10:
        case VERIFY10:
            n = (cbw.CB[7] <<  8) | (cbw.CB[8] <<  0);
            break;

        case READ12:
        case WRITE12:
            n = (cbw.CB[6] << 24) | (cbw.CB[7] << 16) | (cbw.CB[8] <<  8) | (cbw.CB[9] <<  0);
            break;
    }

    length = n * BlockSize;

    if (!cbw.DataLength) {              // host requests no data
        csw.Status = CSW_FAILED;
        sendCSW();
        return false;
    }

    if (cbw.DataLength != length) {
        if ((cbw.Flags & 0x80) != 0) {
            stallEndpoint(MSDBULK_IN);
        } else {
            stallEndpoint(MSDBULK_OUT);
        }

        csw.Status = CSW_FAILED;
        sendCSW();
        return false;
    }

    return true;
}

int USBCDCMSC::isBreak () {
    int ret = cdcbreak;
    cdcbreak = 0;
    return ret;
}

int USBCDCMSC::disk_initialize() {
    if (_sd->disk_initialize()) {
        _status |= NO_DISK;
        return 1;
    } else {
        _status = DISK_OK;
        return 0;
    }
}

int USBCDCMSC::disk_write(const char *buffer, int block_number) {
    return _sd->disk_write(buffer, block_number);    
}

int USBCDCMSC::disk_read(char *buffer, int block_number) {        
    return _sd->disk_read(buffer, block_number);    
}

int USBCDCMSC::disk_status() {
    return _status;
}

int USBCDCMSC::disk_sectors() {
     return _sd->disk_sectors(); 
}
int USBCDCMSC::disk_size() {
    return _sd->disk_sectors() * 512;
}


void USBCDCMSC::on_module_loaded(){
    // We want to be called every time a new char is received
    this->attach(this, &USBCDCMSC::on_serial_char_received);

    // We only call the command dispatcher in the main loop, nowhere else
    this->register_for_event(ON_MAIN_LOOP);
}






void USBCDCMSC::on_main_loop(void* argument){
    //if( this->configured() ){
    //    this->kernel->serial->printf("a:%d\r\n", this->buffer.size());
    //}
    if( this->has_char('\n') ){
        int index = 0;
        string received;
        while(1){
           char c;
           this->buffer.pop_front(c);
           if( c == '\n' ){
                struct SerialMessage message; 
                message.message = received;
                message.stream = this;
                this->kernel->call_event(ON_CONSOLE_LINE_RECEIVED, &message ); 
               //this->kernel->serial->printf("received: %s \r\n", received.c_str() ); 
               //this->printf("received: %s\r\n", received.c_str() ); 
               return;
            }else{
                received += c;
            }
        }
    }


}

void USBCDCMSC::on_serial_char_received(){
    while(this->available()){
        char received = this->_getc();
        // convert CR to NL (for host OSs that don't send NL)
        if( received == '\r' ){ received = '\n'; }
        //if( this->kernel != NULL ){ 
        //    this->kernel->serial->printf("received:%c\r\n", received); 
        //} 
        this->buffer.push_back(received); 
    }

}


bool USBCDCMSC::has_char(char letter){
    int index = this->buffer.head;
    while( index != this->buffer.tail ){
        if( this->buffer.buffer[index] == letter ){
            return true;
        }
        index = this->buffer.next_block_index(index);
    }
    return false;
}
