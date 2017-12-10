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
            this->i2c = new mbed::I2C(P0_27, P0_28);
            this->i2c->frequency(10000);
            this->register_for_event(ON_GCODE_RECEIVED);
        }

        void on_gcode_received(void *argument) {
            Gcode *gcode = static_cast<Gcode*>(argument);
            if (gcode->has_m && gcode->m == 369) {

                // left instrument
                this->_update_from_i2c_address(0xA6);
                if (this->error) this->_print_error('L', gcode);
                else this->_print_data('L', gcode);

                // right instrument
                this->_update_from_i2c_address(0xAE);
                if (this->error) this->_print_error('R', gcode);
                else this->_print_data('R', gcode);
            }
        }

    private:

        mbed::I2C* i2c;
        char memory_addr[2];
        char return_data[9];
        int error;

        void _update_from_i2c_address(uint8_t device_address) {
            // read data from EEPROM at specified memory address
            int i;
            this->error = 0;
            for (i=0; i<2; i++) this->memory_addr[i] = 0x00; // data's position on EEPROM
            for (i=0; i<9; i++) this->return_data[i] = 0x00;

            // set EEPROM register to register where our data begins
            this->error = this->i2c->write(device_address, this->memory_addr, 2);
            if (this->error == 0) {
                wait(0.1);
                // read 9 bytes from starting from register set above
                this->error = this->i2c->read(device_address, this->return_data, 9);
            }
        }

        void _print_data(char label, Gcode *gcode){
            gcode->stream->printf(
                "[%c] uid:%02X-%02X-%02X-%02X, model:%02X-%02X, nL/mm:%02X-%02X-%02X\r\n",
                label,
                this->return_data[0], this->return_data[1],
                this->return_data[2], this->return_data[3],
                this->return_data[4], this->return_data[5],
                this->return_data[6], this->return_data[7],
                this->return_data[8]
            );
        }

        void _print_error(char label, Gcode *gcode) {
            gcode->stream->printf("[%c] error: %d\r\n", label, this->error);
        }
};

#endif
