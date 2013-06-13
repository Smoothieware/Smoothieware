#ifndef DIGIPOTBASE_H
#define DIGIPOTBASE_H

#include "libs/Kernel.h"
#include "libs/utils.h"
#include <string>
#include <math.h>

class DigipotBase {
    public:
        DigipotBase(){}
        virtual ~DigipotBase(){}

        virtual void set_current( int channel, double current )= 0;
        virtual double get_current(int channel)= 0;
};


#endif
