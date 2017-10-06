#ifndef INSTRUMENT_H
#define INSTRUMENT_H

#include "libs/Kernel.h"
#include "Gcode.h"
#include "StreamOutputPool.h"
#include "libs/utils.h"
#include <libs/Pin.h>
#include "mbed.h"
#include <string>
#include <math.h>


class Instrument : public Module{
    public:

        Instrument(){
        }

        ~Instrument(){
            delete this->i2c;
        }

        void on_module_loaded(){
            // I2C com
            this->i2c = new mbed::I2C(P0_27, P0_28);
            this->i2c->frequency(10000);
            this->register_for_event(ON_GCODE_RECEIVED);
        }

        void on_gcode_received(void *argument) {
            Gcode *gcode = static_cast<Gcode*>(argument);
            if (gcode->has_m) {
                if (gcode->m == 911) {
                    char addr;
                    if (gcode->has_letter('L')) {
                        addr = 'L';
                    }
                    else if (gcode->has_letter('R')) {
                        addr = 'R';
                    }
                    else {
                        return;
                    }

                    // FIXME (andy): for some unknown reason, I need to double
                    // the value of the address here, so the slave device recieves
                    // the request. This shouldn't be possible but it makes it work...
                    addr *= 2;

                    char data[5];
                    this->i2c->read(addr, data, 5);
                    gcode->stream->printf(
                        "Instrument: %d, %d, %d, %d, %d\r\n",
                        data[0], data[1], data[2], data[3], data[4]);
                }
            }
        }

    private:

        mbed::I2C* i2c;
};

#endif