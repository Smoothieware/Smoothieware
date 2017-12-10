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

#define EEPROM_LEFT_ADDRESS 0xA6
#define EEPROM_RIGHT_ADDRESS 0xAE

#define EEPROM_SERIAL_LOCATION 0x80
#define EEPROM_SERIAL_LENGTH 16
#define EEPROM_SERIAL_ADDRESS_OFFSET 0x10

#define OT_DATA_LOCATION 0x00
#define OT_DATA_LENGTH 5


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
                this->_detect_instrument(EEPROM_LEFT_ADDRESS, 'L', gcode);
                this->_detect_instrument(EEPROM_RIGHT_ADDRESS, 'R', gcode);
            }
        }

    private:

        mbed::I2C* i2c;

        char serial[EEPROM_SERIAL_LENGTH];
        char data[OT_DATA_LENGTH];
        int error;
        char memory_addr[1];

        void _detect_instrument(uint8_t address, char label, Gcode *gcode) {
            this->_read_data(
                address + EEPROM_SERIAL_ADDRESS_OFFSET,
                EEPROM_SERIAL_LOCATION,
                this->serial,
                EEPROM_SERIAL_LENGTH
            );
            if (this->error) this->_print_error(label, gcode);
            else {
                this->_read_data(
                    address,
                    OT_DATA_LOCATION,
                    this->data,
                    OT_DATA_LENGTH
                );
                if (this->error) this->_print_error(label, gcode);
                else this->_print_data(label, gcode);
            }
        }

        void _read_data(uint8_t address, char mem, char *data, int length) {
            int i;
            this->error = 0;
            this->memory_addr[0] = mem;
            for (i=0; i<length; i++) data[i] = 0x00;

            this->error = this->i2c->write(address, this->memory_addr, 1);
            if (this->error == 0) {
                wait(0.1);
                this->error = this->i2c->read(address, data, length);
            }
        }

        void _print_data(char label, Gcode *gcode){
            gcode->stream->printf(
                "[%c] serial:%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X, model:%02X%02X, nL/mm:%02X%02X%02X\r\n",
                label,
                this->serial[0], this->serial[1], this->serial[2], this->serial[3],
                this->serial[4], this->serial[5], this->serial[6], this->serial[7],
                this->serial[8], this->serial[9], this->serial[10], this->serial[11],
                this->serial[12], this->serial[13], this->serial[14], this->serial[15],
                this->data[0], this->data[1], this->data[2], this->data[3], this->data[4]
            );
        }

        void _print_error(char label, Gcode *gcode) {
            gcode->stream->printf("[%c] error: %d\r\n", label, this->error);
        }
};

#endif
