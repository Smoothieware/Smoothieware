#ifndef HOOK_H
#define HOOK_H
#include "libs/FPointer.h"

class Hook : public FPointer {
    public:
        Hook();
        int     interval;
        int     countdown;
};

#endif
