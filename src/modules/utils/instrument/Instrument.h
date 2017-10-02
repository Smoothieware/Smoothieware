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

        Instrument(){}

        ~Instrument(){
            delete this->i2c;
        }

        void on_module_loaded(){
            this->i2c = new mbed::I2C(P0_27, P0_28);
            this->i2c->frequency(1000);
        }

        void on_gcode_received(void *argument) {
            Gcode *gcode = static_cast<Gcode*>(argument);
            if (gcode->has_m) {
                if (gcode->m == 911) {
                    uint8_t addr;
                    if (gcode->has_letter('L')) {
                        // request ID from left pipette
                        addr = 'L';
                    }
                    else if (gcode->has_letter('R')) {
                        // request ID from right pipette
                        addr = 'R';
                    }
                    else {
                        return;
                    }

                    // FIXME (Andy): for reasons unkown, the Arduino's
                    // I2C address must be double for it to work
                    addr *= 2;

                    char data[5];
                    this->i2c->read(addr, data, 5);

                    gcode->stream->printf(
                        "InstrumentData: %d, %d, %d, %d, %d\r\n",
                        data[0], data[1], data[2], data[3], data[4]);
                }
            }
        }

    private:

        mbed::I2C* i2c;
};

#endif
