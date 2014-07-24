#ifndef AD5206_H
#define AD5206_H

#include "libs/Kernel.h"
#include "libs/utils.h"
#include <libs/Pin.h>
#include "mbed.h"
#include <string>
#include <math.h>

#define max(a,b) (((a) > (b)) ? (a) : (b))

class AD5206 : public DigipotBase {
    public:
        AD5206(){
            this->spi= new mbed::SPI(P0_9,P0_8,P0_7); //should be able to set those pins in config
            cs.from_string("4.29")->as_output(); //this also should be configurable
            cs.set(1);
            for (int i = 0; i < 6; i++) currents[i] = -1;
        }

        void set_current( int channel, float current )
        {
			if(channel<6){
                if(current < 0) {
                    currents[channel]= -1;
                    return;
                }
				current = min( max( current, 0.0L ), 2.0L );
				char adresses[6] = { 0x05, 0x03, 0x01, 0x00, 0x02, 0x04 };
				currents[channel] = current;
				cs.set(0);
				spi->write((int)adresses[channel]);
				spi->write((int)current_to_wiper(current));
				cs.set(1);
			}
        }


        //taken from 4pi firmware
        unsigned char current_to_wiper( float current ){
            unsigned int count = int((current*1000)*100/743); //6.8k resistor and 10k pot

            return (unsigned char)count;
        }

        float get_current(int channel)
        {
            if(channel < 6)
                return currents[channel];
            return -1;
        }

    private:

        Pin cs;
        mbed::SPI* spi;
        float currents[6];
};


#endif
