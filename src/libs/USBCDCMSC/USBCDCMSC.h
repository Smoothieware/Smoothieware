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

#ifndef USBCDCMSC_H
#define USBCDCMSC_H

#include <string>
using std::string;
#include "mbed.h"
#include "libs/Module.h"
#include "libs/Kernel.h"


/* These headers are included for child class. */
#include "USBEndpoints.h"
#include "USBDescriptor.h"
#include "USBDevice_Types.h"

#include "USBDevice.h"

//#include "Stream.h"
#include "CircBuffer.h"

#include "libs/RingBuffer.h"

#include "SDFileSystem.h"

#ifdef __GNUC__
    /* Packing for structs in GCC. */
    #define PACK_STRUCT_STRUCT __attribute__((packed))
    #define PACK_STRUCT_BEGIN
    #define PACK_STRUCT_END
#else /* !__GNUC__ */
    /* Packing for structs in ARM compiler. */
    #define PACK_STRUCT_STRUCT
    #define PACK_STRUCT_BEGIN __packed
    #define PACK_STRUCT_END
#endif /* __GNUC__ */

#include "libs/StreamOutput.h"




class USBCDCMSC: public USBDevice, public Module, public StreamOutput {
public:

    /*
    * Constructor
    *
    * @param vendor_id Your vendor_id
    * @param product_id Your product_id
    * @param product_release Your preoduct_release
    */
    USBCDCMSC(SDFileSystem *sd, uint16_t vendor_id = 0x1f00, uint16_t product_id = 0x2012, uint16_t product_release = 0x0001);

    /**
    * Send a character. You can use puts, printf.
    *
    * @param c character to be sent
    * @returns true if there is no error, false otherwise
    */
    virtual int _putc(int c);
    
    /**
     * Send a formatted string.
     */
    int printf(const char* format, ...);

    /**
    * Read a character: blocking
    *
    * @returns character read
    */
    virtual int _getc();
    
    /**
    * Check the number of bytes available.
    *
    * @returns the number of bytes available
    */
    uint8_t available(); 
    
    /**
    * Write a block of data. 
    *
    * For more efficiency, a block of size 64 (maximum size of a bulk endpoint) has to be written.
    *
    * @param buf pointer on data which will be written
    * @param size size of the buffer. The maximum size of a block is limited by the size of the endpoint (64 bytes)
    *
    * @returns true if successfull
    */
    bool writeBlock(uint8_t * buf, uint16_t size);

    /**
     *  Attach a member function to call when a packet is received. 
     *
     *  @param tptr pointer to the object to call the member function on
     *  @param mptr pointer to the member function to be called
     */
    template<typename T>
    void attach(T* tptr, void (T::*mptr)(void)) {
        if((mptr != NULL) && (tptr != NULL)) {
            rx.attach(tptr, mptr);
        }
    }

    /**
     * Attach a callback called when a packet is received
     *
     * @param fptr function pointer
     */
    void attach(void (*fn)(void)) {
        if(fn != NULL) {
            rx.attach(fn);
        }
    }


    /**
    * Connect the USB MSD device. Establish disk initialization before really connect the device.
    *
    * @returns true if successful
    */
    bool connect();
    void on_module_loaded();
    void on_serial_char_received();
    void on_main_loop(void* argument);
    bool has_char(char letter);

    RingBuffer<char,256> buffer;             // Receive buffer

protected:
    
    /*
    * Get device descriptor. Warning: this method has to store the length of the report descriptor in reportLength.
    *
    * @returns pointer to the device descriptor
    */
    virtual uint8_t * deviceDesc();
    
    /*
    * Get string product descriptor
    *
    * @returns pointer to the string product descriptor
    */
    virtual uint8_t * stringIproductDesc();
    
    /*
    * Get string interface descriptor
    *
    * @returns pointer to the string interface descriptor
    */
    virtual uint8_t * stringIinterfaceDesc();

    /*
    * Get configuration descriptor
    *
    * @returns pointer to the configuration descriptor
    */
    virtual uint8_t * configurationDesc();
    
    /*
    * Send a buffer
    *
    * @param endpoint endpoint which will be sent the buffer
    * @param buffer buffer to be sent
    * @param size length of the buffer
    * @returns true if successful
    */
    bool send(uint8_t * buffer, uint16_t size);
    
    /*
    * Read a buffer from a certain endpoint. Warning: blocking
    *
    * @param endpoint endpoint to read
    * @param buffer buffer where will be stored bytes
    * @param size the number of bytes read will be stored in *size
    * @param maxSize the maximum length that can be read
    * @returns true if successful
    */
    bool readEP(uint8_t * buffer, uint16_t * size);
    
    /*
    * Read a buffer from a certain endpoint. Warning: non blocking
    *
    * @param endpoint endpoint to read
    * @param buffer buffer where will be stored bytes
    * @param size the number of bytes read will be stored in *size
    * @param maxSize the maximum length that can be read
    * @returns true if successful
    */
    bool readEP_NB(uint8_t * buffer, uint16_t * size);

    virtual bool USBCallback_request();
    virtual bool USBCallback_setConfiguration(uint8_t configuration);

    virtual bool EP2_OUT_callback();

    /*
    * Callback called when a packet is received
    */
    virtual bool EP5_OUT_callback();

    /*
    * Callback called when a packet has been sent
    */
    virtual bool EP5_IN_callback();

private:

    FunctionPointer rx;
    CircBuffer<uint8_t> cdcbuf;
    int cdcbreak;

    // MSC Bulk-only Stage
    enum Stage {
        READ_CBW,     // wait a CBW
        ERROR,        // error
        PROCESS_CBW,  // process a CBW request
        SEND_CSW,     // send a CSW
        WAIT_CSW,     // wait that a CSW has been effectively sent
    };


    // Bulk-only CBW
    typedef PACK_STRUCT_BEGIN struct {
        uint32_t Signature;
        uint32_t Tag;
        uint32_t DataLength;
        uint8_t  Flags;
        uint8_t  LUN;
        uint8_t  CBLength;
        uint8_t  CB[16];
    } PACK_STRUCT_STRUCT CBW;

    // Bulk-only CSW
    typedef PACK_STRUCT_BEGIN struct {
        uint32_t Signature;
        uint32_t Tag;
        uint32_t DataResidue;
        uint8_t  Status;
    } PACK_STRUCT_STRUCT CSW;

	// sense
    typedef PACK_STRUCT_BEGIN struct {
        uint8_t error;
        uint8_t resvd;
        uint8_t sense_key;
        uint8_t information[4];
        uint8_t additional_sense_length;
        uint8_t resvd2[4];
        uint8_t asc;
        uint8_t ascq;
        uint8_t resvd3[4];
    } PACK_STRUCT_STRUCT SENSE;

    //state of the bulk-only state machine
    Stage stage;

    // current CBW
    CBW cbw;

    // CSW which will be sent
    CSW csw;
	
	// Current sense
	SENSE sense;

    // addr where will be read or written data
    uint32_t addr;

    // length of a reading or writing
    uint32_t length;

    // memory OK (after a memoryVerify)
    bool memOK;

    // cache in RAM before writing in memory. Useful also to read a block.
    uint8_t * page;

    int BlockSize;
    int MemorySize;
    int BlockCount;

    int _status;
    SDFileSystem *_sd;

    void CBWDecode(uint8_t * buf, uint16_t size);
    void sendCSW (void);
    bool inquiryRequest (void);
    bool msd_write (uint8_t * buf, uint16_t size);
    bool readFormatCapacity();
    bool readCapacity (void);
    bool infoTransfer (void);
    void memoryRead (void);
    bool modeSense6 (void);
    void testUnitReady (void);
    bool requestSense (void);
	void setSense (uint8_t sense_key, uint8_t asc, uint8_t ascq);
    void memoryVerify (uint8_t * buf, uint16_t size);
    void memoryWrite (uint8_t * buf, uint16_t size);
    void mediaRemoval(void);
    void reset();
    void fail();

    int isBreak();

    int disk_initialize();
    int disk_write(const char *buffer, int block_number);
    int disk_read(char *buffer, int block_number);    
    int disk_status();
    int disk_sectors();
    int disk_size();

};

#endif
