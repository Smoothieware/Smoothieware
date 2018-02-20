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

#define EEPROM_LEFT_ADDRESS 0xA0
#define EEPROM_RIGHT_ADDRESS 0xA2

// 8-byte ID (2=model, 6=unique)
#define OT_ID_LOCATION 0x80
#define OT_ID_LENGTH 8

// 16 bytes for any custom data to be saved on this instrument
#define OT_DATA_LOCATION 0x10
#define OT_DATA_LENGTH 16

// known series of bytes that must be on all instruments
#define OT_SIGNATURE_LOCATION 0x00
#define OT_SIGNATURE_LENGTH 4
const uint8_t OT_SIGNATURE[OT_SIGNATURE_LENGTH] {0xCA, 0xFE, 0xFE, 0xFE};

#define NO_ERROR 0
#define WRITE_ARGUMENTS_ERROR 1
#define NO_DATA_ERROR 2
#define I2C_ERROR 32


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
                this->_read_ot_id(EEPROM_LEFT_ADDRESS, 'L', gcode);
                this->_read_ot_id(EEPROM_RIGHT_ADDRESS, 'R', gcode);
            }
            else if (gcode->has_m && gcode->m == 370) {
                if (gcode->has_letter('L')) {
                    this->_write_ot_data(EEPROM_LEFT_ADDRESS, 'L', gcode);
                }
                else if (gcode->has_letter('R')) {
                    this->_write_ot_data(EEPROM_RIGHT_ADDRESS, 'R', gcode);
                }
            }
            else if (gcode->has_m && gcode->m == 371) {
                if (gcode->has_letter('L')) {
                    this->_read_ot_data(EEPROM_LEFT_ADDRESS, 'L', gcode);
                }
                else if (gcode->has_letter('R')) {
                    this->_read_ot_data(EEPROM_RIGHT_ADDRESS, 'R', gcode);
                }
            }
        }

    private:

        mbed::I2C* i2c;

        char unique_id[OT_ID_LENGTH];
        char read_data[OT_DATA_LENGTH];
        char write_data[OT_DATA_LENGTH + 1];
        char signature_data[OT_SIGNATURE_LENGTH];

        int error;
        char memory_addr[1];

        void _read_ot_id(uint8_t address, char label, Gcode *gcode) {
            this->_verify_signature(address, label, gcode);
            if (this->error) {
                this->_print_error(label, gcode);
                return;
            }
            this->_i2c_read(
                address,
                OT_ID_LOCATION, this->unique_id, OT_ID_LENGTH);
            if (this->error) {
                this->_print_error(label, gcode);
                return;
            }
            this->_print_id(label, gcode);
        }

        void _read_ot_data(uint8_t address, char label, Gcode *gcode) {
            this->_verify_signature(address, label, gcode);
            if (this->error) {
                this->_print_error(label, gcode);
                return;
            }
            this->_i2c_read(
                address, OT_DATA_LOCATION, this->read_data, OT_DATA_LENGTH);
            if (this->error) {
                this->_print_error(label, gcode);
                return;
            }
            this->_print_data(label, gcode);
        }

        void _verify_signature(uint8_t address, char label, Gcode *gcode) {
            this->_i2c_read(
                address, OT_SIGNATURE_LOCATION, this->signature_data, OT_SIGNATURE_LENGTH);
            if (this->error) {
                return;
            }
            else {
                for (uint8_t n=0;n<OT_SIGNATURE_LENGTH;n++) {
                    if (this->signature_data[n] != OT_SIGNATURE[n]) {
                        this->error = NO_DATA_ERROR;
                        return;
                    }
                }
            }
        }

        void _write_ot_data(uint8_t address, char label, Gcode *gcode) {

            // first, save the ot signature
            this->_write_signature(address, label, gcode);
            if (this->error) {
                this->_print_error(label, gcode);
                return;
            }

            // erase buffer, then dump gcode into the buffer
            for (int i=0; i<OT_DATA_LENGTH + 1; i++) {
                this->write_data[i] = 0x00;
            }
            this->_parse_hex_from_gcode(label, gcode);
            if (this->error) {
                this->_print_error(label, gcode);
                return;
            }

            // set memory address
            this->write_data[0] = OT_DATA_LOCATION;
            this->_i2c_write(address, OT_DATA_LENGTH + 1);
            if (this->error) {
                this->_print_error(label, gcode);
                return;
            }
        }

        void _write_signature(uint8_t address, char label, Gcode *gcode) {
            // erase buffer
            int i;
            for (i=0; i<OT_DATA_LENGTH + 1; i++) {
                this->write_data[i] = 0x00;
            }
            for (i=0; i<OT_SIGNATURE_LENGTH; i++) {
                this->write_data[i + 1] = OT_SIGNATURE[i];
            }

            // set memory address
            this->write_data[0] = OT_SIGNATURE_LOCATION;
            this->_i2c_write(address, OT_SIGNATURE_LENGTH + 1);
            if (this->error) {
                return;
            }
            this->_verify_signature(address, label, gcode);
            if (this->error) {
                return;
            }
        }

        void _i2c_write(uint8_t address, int length) {
            this->error = NO_ERROR;
            this->error = this->i2c->write(address, this->write_data, length);
            if (this->error == NO_ERROR) {
                wait(0.05);
            }
        }

        void _i2c_read(uint8_t address, char mem, char *data, int length) {
            this->memory_addr[0] = mem;
            for (int i=0; i<length; i++) data[i] = 0x00; // erase buffer

            this->error = NO_ERROR;
            this->error = this->i2c->write(address, this->memory_addr, 1);
            wait(0.05);
            if (this->error == NO_ERROR) {
                this->error = this->i2c->read(address, data, length);
                wait(0.05);
            }
        }

        void _parse_hex_from_gcode(char label, Gcode *gcode) {
            unsigned int i;
            char len = strlen(gcode->command);
            for (i=0;i<len;i++){
                if (gcode->command[i] == label) {
                    break;  // start following loop where this one broke
                }
            }
            int increment = 1;  // start and 2nd index, because 1st must be memory address
            bool msb = true;
            char c = 0x00;
            char h = 0x00;
            for (i=i+1;i<len;i++) {  // begins after "label" character
                c = gcode->command[i];
                if (this->_is_hex_ascii(c)) {
                    h += this->_decode_ascii(c);
                    if (msb == true) {
                        h *= 0x10;
                        msb = false;
                    }
                    else if (msb == false) {
                        this->write_data[increment] = h;
                        increment++;
                        msb = true;
                        h = 0x00;
                        if (increment == OT_DATA_LENGTH + 1) {
                            break;
                        }
                    }
                }
            }
            this->error = NO_ERROR;
            if (increment < OT_DATA_LENGTH + 1) {
                this->error = WRITE_ARGUMENTS_ERROR;
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
                "%c: data:%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X\r\n",
                label,
                this->read_data[0], this->read_data[1], this->read_data[2], this->read_data[3], this->read_data[4],
                this->read_data[5], this->read_data[6], this->read_data[7], this->read_data[8], this->read_data[9],
                this->read_data[10], this->read_data[11], this->read_data[12], this->read_data[13], this->read_data[14],
                this->read_data[15]
            );
        }

        void _print_id(char label, Gcode *gcode){
            gcode->stream->printf(
                "%c: id:%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X\r\n",
                label,
                this->unique_id[0], this->unique_id[1], this->unique_id[2], this->unique_id[3],
                this->unique_id[4], this->unique_id[5], this->unique_id[6], this->unique_id[7]
            );
        }

        void _print_error(char label, Gcode *gcode) {
            if (this->error == NO_ERROR){
                return;
            }
            else if (this->error == I2C_ERROR) {
                gcode->stream->printf("Error (%c): No instrument found\r\n", label);
            }
            else if (this->error == NO_DATA_ERROR){
                gcode->stream->printf("Error (%c): No data found on connected instrument\r\n", label);
            }
            else if (this->error == WRITE_ARGUMENTS_ERROR) {
                gcode->stream->printf("Error (%c): Data must be %d bytes bytes long\r\n", label, OT_DATA_LENGTH);
            }
        }
};

#endif
