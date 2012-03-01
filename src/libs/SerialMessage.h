#ifndef SERIALMESSAGE_H
#define SERIALMESSAGE_H
#include "mbed.h" // Included because we need the Steam object
struct SerialMessage {
        Stream* stream;
            std::string message;
};
#endif
