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
        void set_max_current(double c) { max_current= c; }
        void set_factor(double f) { factor= f; }

    protected:
        double factor;
        double max_current;
};


#endif
