#ifndef SERIALMESSAGE_H
#define SERIALMESSAGE_H
#include "libs/Channel.h"
struct SerialMessage {
        Channel* stream;
        std::string message;
};
#endif
