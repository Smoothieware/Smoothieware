#ifndef DIGIPOT_H
#define DIGIPOT_H

#include "libs/Kernel.h"
#include "I2C.h" // mbed.h lib
#include "libs/utils.h"
#include <string>
#include <math.h>

class Digipot{
    public:
        Digipot(){
            // I2C com
            this->i2c = new mbed::I2C(p9, p10);
            for (int i = 0; i < 4; i++)
                currents[i] = 0.0;
        }

        char current_to_wiper( double current ){
            return char(ceil(double((113.33*current))));
        }

        void i2c_send( char first, char second, char third ){
            this->i2c->start();
            this->i2c->write(first);
            this->i2c->write(second);
            this->i2c->write(third);
            this->i2c->stop();
        }

        void set_current( int channel, double current )
        {
            current = min( max( current, 0.0L ), 2.0L );

            // Initial setup
            this->i2c_send( 0x58, 0x40, 0xff );
            this->i2c_send( 0x58, 0xA0, 0xff );

            // Set actual wiper value
            char adresses[4] = { 0x00, 0x10, 0x60, 0x70 };
            this->i2c_send( 0x58, adresses[channel], this->current_to_wiper(current) );
            currents[channel] = current;
        }

        double get_current(int channel)
        {
            return currents[channel];
        }

        mbed::I2C* i2c;
        double currents[4];
};


#endif
