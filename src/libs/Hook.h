#ifndef HOOK_H
#define HOOK_H

#include "mbed.h"

class Hook : public FunctionPointer {
    public:
        Hook();
        double           frequency;
        double           counter;
};




#endif
