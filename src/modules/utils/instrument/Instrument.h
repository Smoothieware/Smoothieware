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
            // I2C com
            this->i2c = new mbed::I2C(p28, p27);
            this->i2c->frequency(20000);
        }

        ~Instrument(){
            delete this->i2c;
        }

        void on_module_loaded(){
            this->register_for_event(ON_GCODE_RECEIVED);
        }

        void on_gcode_received(void *argument) {
            Gcode *gcode = static_cast<Gcode*>(argument);
            if (gcode->has_m) {
                if (gcode->m == 911) {
                    char addr = 0x00;
                    if (gcode->has_letter('L')) {
                        addr = 0x01;
                    }
                    else if (gcode->has_letter('R')) {
                        addr = 0x02;
                    }
                    else {
                        gcode->stream->printf("M911 must include letter L or R.\n");
                        return;
                    }
                    char data[2];
                    this->i2c->read(addr, data, 2);
                    gcode->stream->printf("Just did I2C{%d, %d}.\n", data[0], data[1]);
                }
            }
        }

    private:

        mbed::I2C* i2c;
};

#endif
