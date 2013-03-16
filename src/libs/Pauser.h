#ifndef PAUSER_H
#define PAUSER_H

#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"

class Pauser : public Module {
    public:
        Pauser();
        void on_module_loaded();
        void take();
        void release();

        bool paused();

        unsigned short counter;
};

#endif
