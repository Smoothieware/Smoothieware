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

using std::string;

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
            else if (gcode->has_m && gcode->m == 370) {
                if (gcode->has_letter('L')) {
                    this->_save_ot_data(EEPROM_LEFT_ADDRESS, 'L', gcode);
                }
                else if (gcode->has_letter('R')) {
                    this->_save_ot_data(EEPROM_RIGHT_ADDRESS, 'R', gcode);
                }
            }
        }

    private:

        mbed::I2C* i2c;

        char serial[EEPROM_SERIAL_LENGTH];
        char read_data[OT_DATA_LENGTH];
        char write_data[OT_DATA_LENGTH + 1];
        int error;
        char memory_addr[1];

        void _detect_instrument(uint8_t address, char label, Gcode *gcode) {
            // read unique serial number
            this->_read_data(
                address + EEPROM_SERIAL_ADDRESS_OFFSET,
                EEPROM_SERIAL_LOCATION, this->serial, EEPROM_SERIAL_LENGTH);
            if (this->error) {
                this->_print_error(label, gcode);
                return;
            }
            // read ot-data bytes
            this->_read_data(
                address, OT_DATA_LOCATION, this->read_data, OT_DATA_LENGTH);
            if (this->error) {
                this->_print_error(label, gcode);
                return;
            }
            // print results
            this->_print_data(label, gcode);
        }

        void _read_data(uint8_t address, char mem, char *data, int length) {
            this->memory_addr[0] = mem;
            for (int i=0; i<length; i++) data[i] = 0x00; // erase buffer

            this->error = 0;
            this->error = this->i2c->write(address, this->memory_addr, 1);
            if (this->error == 0) {
                wait(0.1);
                this->error = this->i2c->read(address, data, length);
            }
        }

        void _save_ot_data(uint8_t address, char label, Gcode *gcode) {

            for (int i=0; i<OT_DATA_LENGTH + 1; i++) this->write_data[i] = 0x00; // erase buffer
            this->_parse_hex_from_gcode(label, gcode);
            if (this->error) {
                this->_print_error(label, gcode);
                return;
            }

            this->write_data[0] = OT_DATA_LOCATION;
            this->error = 0;
            this->error = this->i2c->write(address, this->write_data, OT_DATA_LENGTH + 1);
            if (this->error) {
                this->_print_error(label, gcode);
                return;
            }
            gcode->stream->printf(
                "%c: model:%02X%02X nL/mm:%02X%02X%02X\r\n",
                label,
                this->write_data[1], this->write_data[2],
                this->write_data[3], this->write_data[4], this->write_data[5]
            );
        }

        void _parse_hex_from_gcode(char label, Gcode *gcode) {
            unsigned int i;
            int increment = 1;
            char c = 0x00;
            bool msb = true;
            for (i=0;i<strlen(gcode->command);i++){
                if (gcode->command[i] == label) {
                    for (i=i+1;i<strlen(gcode->command);i++) {
                        if (this->_is_hex_ascii(gcode->command[i])) {
                            c += this->_decode_ascii(gcode->command[i]);
                            if (msb == true) {
                                c *= 0x10;
                                msb = false;
                            }
                            else if (msb == false) {
                                this->write_data[increment] = c;
                                increment++;
                                msb = true;
                                c = 0x00;
                                if (increment == OT_DATA_LENGTH + 1) {
                                    break;
                                }
                            }
                        }
                    }
                    break;
                }
            }
            this->error = 0;
            if (increment < OT_DATA_LENGTH + 1) {
                this->error = 42;
            }
        }

        char _decode_ascii(char c) {
            if (c >= 'a' && c <= 'f') {
                c -= 'a';
                c += 0x0A;
            }
            else if (c >= 'A' && c <= 'F') {
                c -= 'A';
                c += 0x0A;
            }
            else if (c >= '0' && c <= '9') {
                c -= '0';
            }
            return c;
        }

        char _is_hex_ascii(char c) {
            if (c >= 'a' && c <= 'f') return true;
            else if (c >= 'A' && c <= 'F') return true;
            else if (c >= '0' && c <= '9') return true;
            return false;
        }

        void _print_data(char label, Gcode *gcode){
            gcode->stream->printf(
                "%c: serial:%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X model:%02X%02X nL/mm:%02X%02X%02X\r\n",
                label,
                this->serial[0], this->serial[1], this->serial[2], this->serial[3],
                this->serial[4], this->serial[5], this->serial[6], this->serial[7],
                this->serial[8], this->serial[9], this->serial[10], this->serial[11],
                this->serial[12], this->serial[13], this->serial[14], this->serial[15],
                this->read_data[0], this->read_data[1], this->read_data[2], this->read_data[3], this->read_data[4]
            );
        }

        void _print_error(char label, Gcode *gcode) {
            gcode->stream->printf("%c: error:%d\r\n", label, this->error);
        }
};

#endif
