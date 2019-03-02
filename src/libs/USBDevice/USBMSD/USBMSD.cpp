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
#include <stdlib.h>
#include <cstring>
#include <cstdio>

#include "USBMSD.h"

#include "descriptor_msc.h"

#include "Kernel.h"

#include "platform_memory.h"

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

#define STARTSTOP_STOPMOTOR        0x0
#define STARTSTOP_STARTMOTOR       0x1
#define STARTSTOP_EJECT            0x2
#define STARTSTOP_LOAD             0x3

#define DEFAULT_CONFIGURATION (1)

// max packet size
#define MAX_PACKET  MAX_PACKET_SIZE_EPBULK

// #define iprintf(...) THEKERNEL->streams->printf(__VA_ARGS__)
#define iprintf(...) do { } while (0)

// CSW Status
enum Status {
    CSW_PASSED,
    CSW_FAILED,
    CSW_ERROR,
};

USBMSD::USBMSD(USB *u, MSD_Disk *d) {
    this->usb = u;
    this->disk = d;

    usbdesc_interface i = {
        DL_INTERFACE,           // bLength
        DT_INTERFACE,           // bDescType
        0,                      // bInterfaceNumber - filled out by USB during attach()
        0,                      // bAlternateSetting
        2,                      // bNumEndpoints
        UC_MASS_STORAGE,        // bInterfaceClass
        MSC_SUBCLASS_SCSI,      // bInterfaceSubClass
        MSC_PROTOCOL_BULK_ONLY, // bInterfaceProtocol
        0,                      // iInterface
        0, 0, 0,                // dummy padding
        this,                   // callback
    };
    memcpy(&MSC_Interface, &i, sizeof(MSC_Interface));

    usbdesc_endpoint j = {
        DL_ENDPOINT,            // bLength
        DT_ENDPOINT,            // bDescType
        EP_DIR_IN,              // bEndpointAddress - we provide direction, index is filled out by USB during attach()
        EA_BULK,                // bmAttributes
        MAX_PACKET_SIZE_EPBULK, // wMaxPacketSize
        0,                      // bInterval
        0,                      // dummy padding
        this,                   // endpoint callback
    };
    memcpy(&MSC_BulkIn, &j, sizeof(MSC_BulkIn));

    usbdesc_endpoint k = {
        DL_ENDPOINT,            // bLength
        DT_ENDPOINT,            // bDescType
        EP_DIR_OUT,             // bEndpointAddress - we provide direction, index is filled out by USB during attach()
        EA_BULK,                // bmAttributes
        MAX_PACKET_SIZE_EPBULK, // wMaxPacketSize
        0,                      // bInterval
        0,                      // dummy padding
        this,                   // endpoint callback
    };
    memcpy(&MSC_BulkOut, &k, sizeof(MSC_BulkOut));

    // because gcc-4.6 won't let us simply do MSC_Description = usbstring("Smoothie MSD")
    usbdesc_string_l(13) us = usbstring("Smoothie MSD");
    memcpy(&MSC_Description, &us, sizeof(MSC_Description));

    usb->addInterface(&MSC_Interface);

    usb->addEndpoint(&MSC_BulkIn);
    usb->addEndpoint(&MSC_BulkOut);

    MSC_Interface.iInterface =
    	usb->addString(&MSC_Description);
}

// Called in ISR context to process a class specific request
bool USBMSD::USBEvent_Request(CONTROL_TRANSFER &transfer)
{
    iprintf("MSD:Control: ");
    bool success = false;
//     CONTROL_TRANSFER * transfer = getTransferPtr();
    static uint8_t maxLUN[1] = {0};

    if (transfer.setup.bmRequestType.Type == CLASS_TYPE) {
        switch (transfer.setup.bRequest) {
            case MSC_REQUEST_RESET:
//                 iprintf("MSC:Req Reset\n");
                reset();
                success = true;
                break;
            case MSC_REQUEST_GET_MAX_LUN:
//                 iprintf("MSC:Req Get_Max_Lun\n");
                transfer.remaining = 1;
                transfer.ptr = maxLUN;
                transfer.direction = DEVICE_TO_HOST;
                success = true;
                break;
            default:
                break;
        }
    }
    iprintf("%d\n", success?1:0);

    return success;
}

bool USBMSD::USBEvent_RequestComplete(CONTROL_TRANSFER &transfer, uint8_t *buf, uint32_t length)
{
    return true;
}

bool USBMSD::connect()
{
    BlockCount = 0;

    //disk initialization
    if (disk->disk_status() & NO_INIT) {
        if (disk->disk_initialize()) {
            return false;
        }
    }

    // get number of blocks
    BlockCount = disk->disk_sectors();

    // get memory size
//     MemorySize = disk->disk_size();
    BlockSize = disk->disk_blocksize();

    if ((BlockCount > 0) && (BlockSize != 0)) {
        page = (uint8_t*) AHB0.alloc(BlockSize);
        if (page == NULL)
            return false;
    } else {
        return false;
    }

    return true;
}


void USBMSD::reset() {
    stage = READ_CBW;
    usb->endpointSetInterrupt(MSC_BulkOut.bEndpointAddress, true);
    usb->endpointSetInterrupt(MSC_BulkIn.bEndpointAddress, false);
}


// Called in ISR context called when a data is received
// bool USBMSD::EP2_OUT_callback() {
bool USBMSD::USBEvent_EPOut(uint8_t bEP, uint8_t bEPStatus) {
    uint32_t size = 0;
//     uint8_t buf[MAX_PACKET_SIZE_EPBULK];
    usb->readEP(MSC_BulkOut.bEndpointAddress, buffer, &size, MAX_PACKET_SIZE_EPBULK);
    iprintf("MSD:EPOut:Read %lu\n", size);
    switch (stage) {
            // the device has to decode the CBW received
        case READ_CBW:
            CBWDecode(buffer, size);
            break;

            // the device has to receive data from the host
        case PROCESS_CBW:
            switch (cbw.CB[0]) {
                case WRITE10:
                case WRITE12:
                    memoryWrite(buffer, size);
                    break;
                case VERIFY10:
                    memoryVerify(buffer, size);
                    break;
            }
            break;

            // an error has occured: stall endpoint and send CSW
        default:
            usb->stallEndpoint(MSC_BulkOut.bEndpointAddress);
            csw.Status = CSW_ERROR;
            sendCSW();
            break;
    }

    //reactivate readings on the OUT bulk endpoint
    usb->readStart(MSC_BulkOut.bEndpointAddress, MAX_PACKET_SIZE_EPBULK);
    return true;
}

// Called in ISR context when a data has been transferred
// bool USBMSD::EP2_IN_callback() {
bool USBMSD::USBEvent_EPIn(uint8_t bEP, uint8_t bEPStatus) {
    uint8_t _stage = stage;
    if (stage != 2)
        iprintf("MSD:In:S%d,G:", stage);

    bool gotMoreData = false;

    switch (stage) {
        // not sure
        case READ_CBW:  // stage 0
            gotMoreData = false;
            break;

        // the device has to send data to the host
        case PROCESS_CBW: // stage 2
            gotMoreData = false;
            switch (cbw.CB[0]) {
                case READ10:
                case READ12:
                    memoryRead();
                    gotMoreData = true;
                    break;
            }
            break;

        //the device has to send a CSW
        case SEND_CSW:      // stage 3
            sendCSW();
            gotMoreData = true;
            break;

        // an error has occured
        case ERROR:         // stage 1
            usb->stallEndpoint(MSC_BulkIn.bEndpointAddress);
            sendCSW();
            gotMoreData = false;
            break;

        // the host has received the CSW -> we wait a CBW
        case WAIT_CSW:      // stage 4
            stage = READ_CBW;
            gotMoreData = false;
            break;
    }

    if (_stage != 2)
        iprintf("%d\n", gotMoreData?1:0);

    return gotMoreData;
}

void USBMSD::memoryWrite (uint8_t * buf, uint16_t size) {

    if (lba > BlockCount) {
        size = (BlockCount - lba) * BlockSize + addr_in_block;
        stage = ERROR;
        usb->stallEndpoint(MSC_BulkOut.bEndpointAddress);
    }

    // we fill an array in RAM of 1 block before writing it in memory
    for (int i = 0; i < size; i++)
        page[addr_in_block + i] = buf[i];

    // if the array is filled, write it in memory
    if ((addr_in_block + size) >= BlockSize) {
        if (!(disk->disk_status() & WRITE_PROTECT)) {
            disk->disk_write((const char *)page, lba);
        }
    }

    addr_in_block += size;
    length -= size;
    csw.DataResidue -= size;
    if (addr_in_block >= BlockSize)
    {
        addr_in_block = 0;
        lba++;
    }

    if ((!length) || (stage != PROCESS_CBW)) {
        csw.Status = (stage == ERROR) ? CSW_FAILED : CSW_PASSED;
        sendCSW();
    }
}

void USBMSD::memoryVerify (uint8_t * buf, uint16_t size) {
    uint32_t n;

    if (lba > BlockCount) {
        size = (BlockCount - lba) * BlockSize + addr_in_block;
        stage = ERROR;
        usb->stallEndpoint(MSC_BulkOut.bEndpointAddress);
    }

    // beginning of a new block -> load a whole block in RAM
    if (addr_in_block == 0)
        disk->disk_read((char *)page, lba);

    // info are in RAM -> no need to re-read memory
    for (n = 0; n < size; n++) {
        if (page[addr_in_block + n] != buf[n]) {
            memOK = false;
            break;
        }
    }

    addr_in_block += size;
    length -= size;
    csw.DataResidue -= size;

    if (addr_in_block >= BlockSize)
    {
        addr_in_block = 0;
        lba++;
    }

    if ( !length || (stage != PROCESS_CBW)) {
        csw.Status = (memOK && (stage == PROCESS_CBW)) ? CSW_PASSED : CSW_FAILED;
        sendCSW();
    }
}


bool USBMSD::inquiryRequest (void) {
    uint8_t inquiry[] = { 0x00, 0x80, 0x00, 0x01,
                          36 - 4, 0x00, 0x00, 0x01,
                          'M', 'B', 'E', 'D', '.', 'O', 'R', 'G',
                          'M', 'B', 'E', 'D', ' ', 'U', 'S', 'B', ' ', 'D', 'I', 'S', 'K', ' ', ' ', ' ',
                          '1', '.', '0', ' ',
                        };

    if (BlockCount == 0)
        inquiry[0] = 0x20; // PERIPHERAL_QUALIFIER = 1 : "A peripheral device is not connected, however usually we do support this type of peripheral"

    if (!write(inquiry, sizeof(inquiry))) {
        return false;
    }
    return true;
}


bool USBMSD::readFormatCapacity() {
    uint8_t capacity[] = { 0x00, 0x00, 0x00, 0x08,
                           (uint8_t) ((BlockCount >> 24) & 0xff),
                           (uint8_t) ((BlockCount >> 16) & 0xff),
                           (uint8_t) ((BlockCount >>  8) & 0xff),
                           (uint8_t) ((BlockCount >>  0) & 0xff),

                           0x02,
                           (uint8_t) ((BlockSize >> 16) & 0xff),
                           (uint8_t) ((BlockSize >>  8) & 0xff),
                           (uint8_t) ((BlockSize >>  0) & 0xff),
                         };
    if (!write(capacity, sizeof(capacity))) {
        return false;
    }
    return true;
}


bool USBMSD::readCapacity (void) {
    uint8_t capacity[] = {
        (uint8_t) (((BlockCount - 1) >> 24) & 0xff),
        (uint8_t) (((BlockCount - 1) >> 16) & 0xff),
        (uint8_t) (((BlockCount - 1) >> 8) & 0xff),
        (uint8_t) (((BlockCount - 1) >> 0) & 0xff),

        (uint8_t) ((BlockSize >> 24) & 0xff),
        (uint8_t) ((BlockSize >> 16) & 0xff),
        (uint8_t) ((BlockSize >> 8) & 0xff),
        (uint8_t) ((BlockSize >> 0) & 0xff),
    };
    if (!write(capacity, sizeof(capacity))) {
        return false;
    }
    return true;
}

bool USBMSD::write (uint8_t * buf, uint16_t size) {
    if (size >= cbw.DataLength) {
        size = cbw.DataLength;
    }
    stage = SEND_CSW;

//     iprintf("MSD:write: %u bytes\n", size);

    if (!usb->writeNB(MSC_BulkIn.bEndpointAddress, buf, size, MAX_PACKET_SIZE_EPBULK)) {
        return false;
    }

//     iprintf("MSD:write OK, sending CSW\n");

    csw.DataResidue -= size;
    csw.Status = CSW_PASSED;

    usb->endpointSetInterrupt(MSC_BulkIn.bEndpointAddress, true);

    return true;
}


bool USBMSD::modeSense6 (void) {
    uint8_t sense6[] = { 0x03, 0x00, 0x00, 0x00 };
    if (!write(sense6, sizeof(sense6))) {
        return false;
    }
    return true;
}

void USBMSD::sendCSW() {
    csw.Signature = CSW_Signature;
//     iprintf("MSD:SendCSW:\n\tSignature : %lu\n\tTag       : %lu\n\tDataResidue: %lu\n\tStatus     : %u\n", csw.Signature, csw.Tag, csw.DataResidue, csw.Status);
    usb->writeNB(MSC_BulkIn.bEndpointAddress, (uint8_t *)&csw, sizeof(CSW), MAX_PACKET_SIZE_EPBULK);
    stage = WAIT_CSW;
    usb->endpointSetInterrupt(MSC_BulkIn.bEndpointAddress, true);
}

bool USBMSD::requestSense (void) {
    uint8_t request_sense[] = {
        0x70,
        0x00,
        0x05,   // Sense Key: illegal request
        0x00,
        0x00,
        0x00,
        0x00,
        0x0A,
        0x00,
        0x00,
        0x00,
        0x00,
        0x30,
        0x01,
        0x00,
        0x00,
        0x00,
        0x00,
    };

    if (BlockCount == 0)
    {
        request_sense[ 2] = 0x02; // Not Ready
        request_sense[12] = 0x3A; // Medium not present
        request_sense[13] = 0x00; // No known reason
    }

    if (!write(request_sense, sizeof(request_sense))) {
        return false;
    }

    return true;
}

void USBMSD::fail() {
    csw.Status = CSW_FAILED;
    sendCSW();
}

void USBMSD::CBWDecode(uint8_t * buf, uint16_t size) {
    if (size == sizeof(cbw)) {
        memcpy((uint8_t *)&cbw, buf, size);
        if (cbw.Signature == CBW_Signature) {
            csw.Tag = cbw.Tag;
            csw.DataResidue = cbw.DataLength;
            if ((cbw.CBLength <  1) || (cbw.CBLength > 16) ) {
                iprintf("MSD:Got CBW 0x%02X with invalid CBLength\n", cbw.CB[0]);
                fail();
            } else {
                iprintf("MSD:Got CBW 0x%02X with datalength %lu\n", cbw.CB[0], cbw.DataLength);
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
                    case READ_CAPACITY:
                        readCapacity();
                        break;
                    case READ10:
                    case READ12:
//                         iprintf("MSD:READ10\n");
                        if (infoTransfer()) {
                            if ((cbw.Flags & 0x80)) {
                                iprintf("MSD: Read %lu blocks from LBA %lu\n", blocks, lba);
                                stage = PROCESS_CBW;
//                                 memoryRead();
                                usb->endpointSetInterrupt(MSC_BulkIn.bEndpointAddress, true);
                            } else {
                                usb->stallEndpoint(MSC_BulkOut.bEndpointAddress);
                                csw.Status = CSW_ERROR;
                                sendCSW();
                            }
                        }
                        break;
                    case WRITE10:
                    case WRITE12:
                        if (infoTransfer()) {
                            if (!(cbw.Flags & 0x80)) {
                                iprintf("MSD: Write %lu blocks from LBA %lu\n", blocks, lba);
                                stage = PROCESS_CBW;
                            } else {
                                usb->stallEndpoint(MSC_BulkIn.bEndpointAddress);
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
                                iprintf("MSD: Verify %lu blocks from LBA %lu\n", blocks, lba);
                                stage = PROCESS_CBW;
                                memOK = true;
                            } else {
                                usb->stallEndpoint(MSC_BulkIn.bEndpointAddress);
                                csw.Status = CSW_ERROR;
                                sendCSW();
                            }
                        }
                        break;
                    case START_STOP_UNIT:
                    {
                        switch (cbw.CB[4] & 0x03)
                        {
                            case STARTSTOP_STOPMOTOR:
                                break;
                            case STARTSTOP_STARTMOTOR:
                                break;
                            case STARTSTOP_EJECT:
                                break;
                            case STARTSTOP_LOAD:
                                break;
                        }
                        csw.Status = CSW_PASSED;
                        sendCSW();
                        break;
                    }
                    default:
                        iprintf("MSD: Unhandled SCSI CBW 0x%02X\n", cbw.CB[0]);
                        fail();
                        break;
                }
            }
        }
        else {
            iprintf("MSD:Got CBW 0x%02X with bad signature\n", cbw.CB[0]);
        }
    }
    else {
        iprintf("MSD:Got CBW 0x%02X with bad length: %u (%u)\n", cbw.CB[0], size, sizeof(cbw));
    }
}

void USBMSD::testUnitReady (void) {

    if (cbw.DataLength != 0) {
        if ((cbw.Flags & 0x80) != 0) {
            usb->stallEndpoint(MSC_BulkIn.bEndpointAddress);
        } else {
            usb->stallEndpoint(MSC_BulkOut.bEndpointAddress);
        }
    }

    if (BlockCount > 0)
        csw.Status = CSW_PASSED;
    else
        csw.Status = CSW_ERROR;

    sendCSW();
}

void USBMSD::memoryRead (void) {
    uint32_t n;

    n = (length > MAX_PACKET_SIZE_EPBULK) ? MAX_PACKET_SIZE_EPBULK : length;

    if (lba > BlockCount) {
        iprintf("MSD:Attemt to read beyond end of disk! Read LBA %lu > Disk LBAs %lu\n", lba, BlockCount);
        n = (BlockCount - lba) * BlockSize + addr_in_block;
        stage = ERROR;
    }

    // we read an entire block
    if (addr_in_block == 0)
    {
        iprintf("MSD:LBA %lu:", lba);
        disk->disk_read((char *)page, lba);
    }

    iprintf(" %u", addr_in_block / MAX_PACKET_SIZE_EPBULK);

    // write data which are in RAM
    usb->writeNB(MSC_BulkIn.bEndpointAddress, &page[addr_in_block], n, MAX_PACKET_SIZE_EPBULK);

    addr_in_block += n;

    length -= n;
    csw.DataResidue -= n;

    if (addr_in_block >= BlockSize)
    {
        iprintf("\n");
        addr_in_block = 0;
        lba++;
    }

    if ( !length || (stage != PROCESS_CBW)) {
        csw.Status = (stage == PROCESS_CBW) ? CSW_PASSED : CSW_FAILED;
        stage = (stage == PROCESS_CBW) ? SEND_CSW : stage;
    }
    usb->endpointSetInterrupt(MSC_BulkIn.bEndpointAddress, true);
}

bool USBMSD::infoTransfer (void) {
    // Logical Block Address of First Block
    lba = (cbw.CB[2] << 24) | (cbw.CB[3] << 16) | (cbw.CB[4] <<  8) | (cbw.CB[5] <<  0);

//     addr = lba * BlockSize;

    // Number of Blocks to transfer
    switch (cbw.CB[0]) {
        case READ10:
        case WRITE10:
        case VERIFY10:
            blocks = (cbw.CB[7] <<  8) | (cbw.CB[8] <<  0);
            break;

        case READ12:
        case WRITE12:
            blocks = (cbw.CB[6] << 24) | (cbw.CB[7] << 16) | (cbw.CB[8] <<  8) | (cbw.CB[9] <<  0);
            break;
    }

    if ((lba + blocks) > BlockCount)
    {
        csw.Status = CSW_FAILED;
        sendCSW();
        return false;
    }

    length = blocks * BlockSize;

    if (!cbw.DataLength) {              // host requests no data
        csw.Status = CSW_FAILED;
        sendCSW();
        return false;
    }

    if (cbw.DataLength != length) {
        if ((cbw.Flags & 0x80) != 0) {
            usb->stallEndpoint(MSC_BulkIn.bEndpointAddress);
        } else {
            usb->stallEndpoint(MSC_BulkOut.bEndpointAddress);
        }

        csw.Status = CSW_FAILED;
        sendCSW();
        return false;
    }

    addr_in_block = 0;

//     iprintf("MSD:transferring %lu blocks from LBA %lu.\n", blocks, lba);

    return true;
}

void USBMSD::on_module_loaded()
{
    connect();
}

bool USBMSD::USBEvent_busReset(void)
{
	return true;
}

bool USBMSD::USBEvent_connectStateChanged(bool connected)
{
	return true;
}

bool USBMSD::USBEvent_suspendStateChanged(bool suspended)
{
	return true;
}
