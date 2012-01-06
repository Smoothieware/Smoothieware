/* mbed Microcontroller Library - can_helper
 * Copyright (c) 2009 ARM Limited. All rights reserved.
 * rmeyer
 */ 

#ifndef MBED_CAN_HELPER_H
#define MBED_CAN_HELPER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum CANFormat CANFormat;
enum CANFormat {
    CANStandard = 0,
    CANExtended = 1
};

typedef enum CANType CANType;
enum CANType {
    CANData   = 0,
    CANRemote = 1
};

typedef struct CAN_Message CAN_Message;
struct CAN_Message {
    unsigned int   id;                 // 29 bit identifier
    unsigned char  data[8];            // Data field
    unsigned char  len;                // Length of data field in bytes
    CANFormat      format;             // 0 - STANDARD, 1- EXTENDED IDENTIFIER
    CANType        type;               // 0 - DATA FRAME, 1 - REMOTE FRAME
};

#ifdef __cplusplus
};
#endif

#endif // MBED_CAN_HELPER_H
