#ifndef MCP4451_H
#define MCP4451_H

#include "libs/Kernel.h"
#include "I2C.h" // mbed.h lib
#include "libs/utils.h"
#include "DigipotBase.h"
#include <string>
#include <math.h>

class MCP4451 : public DigipotBase {
    public:
        MCP4451(){
            // I2C com
            this->i2c = new mbed::I2C(p9, p10);
            this->i2c->frequency(20000);
            for (int i = 0; i < 8; i++) currents[i] = -1;
        }

        ~MCP4451(){
            delete this->i2c;
        }

        void set_current( int channel, float current )
        {
            if(current < 0) {
                currents[channel]= -1;
                return;
            }
            current = min( (float) max( current, 0.0f ), this->max_current );
            currents[channel] = current;
            char addr = 0x58;
            while(channel > 3){
                addr += 0x02;
                channel -= 4;
            }

            // Initial setup
            this->i2c_send( addr, 0x40, 0xff );
            this->i2c_send( addr, 0xA0, 0xff );

            // Set actual wiper value
            char addresses[4] = { 0x00, 0x10, 0x60, 0x70 };
            this->i2c_send( addr, addresses[channel], this->current_to_wiper(current) );
        }

        float get_current(int channel)
        {
            return currents[channel];
        }

    private:

        void i2c_send( char first, char second, char third ){
            this->i2c->start();
            this->i2c->write(first);
            this->i2c->write(second);
            this->i2c->write(third);
            this->i2c->stop();
        }

        char current_to_wiper( float current ){
            return char(ceil(float((this->factor*current))));
        }

        mbed::I2C* i2c;
        float currents[8];
};


#endif
