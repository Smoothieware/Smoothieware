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
            this->memory_addr[0] = 0x00;
            this->memory_addr[1] = 0x00;
            for (int i=0; i<9; i++) this->return_data[i] = 0;
        }

        void on_gcode_received(void *argument) {
            Gcode *gcode = static_cast<Gcode*>(argument);
            if (gcode->has_m) {
                if (gcode->m == 369) {
                    uint8_t device_address = 0xA0;
                    if (gcode->has_letter('R')) device_address &= 0x08;
                    else if(!gcode->has_letter('L')) return;

                    for (int i=0; i<9; i++) this->return_data[i] = 0;

                    // read data from EEPROM at specified memory address
                    this->i2c->write(device_address, this->memory_addr, 2);
                    this->i2c->read(device_address, this->return_data, 9);

                    gcode->stream->printf(
                        "uid: %X%X%X%X, model: %X%X, nL/mm: %X%X%X\r\n",
                        this->return_data[0], this->return_data[1],
                        this->return_data[2], this->return_data[3],
                        this->return_data[4], this->return_data[5],
                        this->return_data[6], this->return_data[7],
                        this->return_data[8]);
                }
            }
        }

    private:

        mbed::I2C* i2c;
        char memory_addr[2];
        char return_data[9];
};

#endif
