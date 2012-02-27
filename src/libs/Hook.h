#ifndef HOOK_H
#define HOOK_H

#include "mbed.h"
// TODO : switch to Fpointer
class Hook : public FunctionPointer {
    public:
        Hook();
        double           frequency;
        double           counter;
};

#endif
