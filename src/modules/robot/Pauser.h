#ifndef PAUSER_H
#define PAUSER_H

#include "Module.h"

class Block;

class Pauser : public Module {
    public:
        Pauser();
        void on_module_loaded();
        void on_block_begin(void*);

        void take();
        void release();

        bool paused();

        Block* paused_block;
        unsigned short counter;
};

#endif
