#ifndef DIGIPOT_H
#define DIGIPOT_H

#include "libs/Kernel.h"
#include "I2C.h" // mbed.h lib
#include "libs/utils.h"
#include <string>
#include <math.h>

class Digipot{
    public:
        Digipot(){ }
      
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

        void set_current( int channel, double current ){ 

            current = min( max( current, 0.0L ), 2.0L );

            // I2C com 
            this->i2c = new mbed::I2C(p9, p10); 
           
            // Initial setup 
            this->i2c_send( 0x58, 0x40, 0xff );   
            this->i2c_send( 0x58, 0xA0, 0xff );   

            // Set actual wiper value
            char adresses[4] = { 0x00, 0x10, 0x60, 0x70 };  
            this->i2c_send( 0x58, adresses[channel], this->current_to_wiper(current) );   

        } 

        mbed::I2C* i2c;
};


#endif
