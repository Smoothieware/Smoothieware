#ifndef AD5206_H
#define AD5206_H

#include "libs/Kernel.h"
#include "libs/utils.h"
#include <string>
#include <math.h>

class AD5206 : public DigipotBase {
    public:
        AD5206(){
        }

        void set_current( int channel, double current )
        {
        }

        double get_current(int channel)
        {
            return currents[channel];
        }

	private:

        double currents[4];
};


#endif
