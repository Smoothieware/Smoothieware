#ifndef SERIALMESSAGE_H
#define SERIALMESSAGE_H
#include "mbed.h" // Included because we need the Steam object
#include "libs/StreamOutput.h"
struct SerialMessage {
        StreamOutput* stream;
        std::string message;
};
#endif
