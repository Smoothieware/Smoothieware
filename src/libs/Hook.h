#ifndef HOOK_H
#define HOOK_H
#include "libs/FPointer.h"

class Hook : public FPointer {
    public:
        Hook();
        double           frequency;
        double           counter;
};

#endif
