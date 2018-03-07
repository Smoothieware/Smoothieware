#include "Instrument.h"
#include "libs/Kernel.h"
#include "Gcode.h"
#include "StreamOutputPool.h"
#include "libs/utils.h"
#include <libs/Pin.h>
#include "mbed.h"
#include <string>
#include <math.h>

using std::string;


Instrument::Instrument(){
}

void Instrument::on_module_loaded(){
    this->i2c = new mbed::I2C(P0_27, P0_28);
    this->i2c->frequency(10000);
    this->register_for_event(ON_GCODE_RECEIVED);
}

void Instrument::on_gcode_received(void *argument) {
    Gcode *gcode = static_cast<Gcode*>(argument);
    uint8_t address = 0;
    char label = 0;
    if (gcode->has_letter('L')) {
        address = EEPROM_LEFT_ADDRESS;
        label = 'L';
    }
    else if (gcode->has_letter('R')) {
        address = EEPROM_RIGHT_ADDRESS;
        label = 'R';
    }
    if (label != 0 && address != 0 && gcode->has_m) {
        if (gcode->m == GCODE_READ_ID) {
            this->_read_from_location(OT_ID_LOCATION, address, label, gcode);
        }
        else if (gcode->m == GCODE_WRITE_ID) {
            this->_write_at_location(OT_ID_LOCATION, address, label, gcode);
        }
        else if (gcode->m == GCODE_READ_MODEL) {
            this->_read_from_location(OT_MODEL_LOCATION, address, label, gcode);
        }
        else if (gcode->m == GCODE_WRITE_MODEL) {
            this->_write_at_location(OT_MODEL_LOCATION, address, label, gcode);
        }
        else if (gcode->m == GCODE_READ_DATA) {
            this->_read_from_location(OT_DATA_LOCATION, address, label, gcode);
        }
        else if (gcode->m == GCODE_WRITE_DATA) {
            this->_write_at_location(OT_DATA_LOCATION, address, label, gcode);
        }
    }
}

void Instrument::_read_from_location(uint8_t location, uint8_t address, char label, Gcode *gcode) {
    this->_check_ot_signature(address, label, gcode);
    if (this->error) {
        this->_print_error(label, gcode);
        return;
    }
    this->_i2c_read(
        address, location, this->read_data, OT_DATA_LENGTH);
    if (this->error) {
        this->_print_error(label, gcode);
        return;
    }
    this->_print_data(label, gcode);
}

void Instrument::_write_at_location(uint8_t location, uint8_t address, char label, Gcode *gcode) {
    this->_write_ot_signature(address, label, gcode);
    if (this->error) {
        this->_print_error(label, gcode);
        return;
    }
    this->_parse_hex_from_gcode(label, gcode, OT_DATA_LENGTH);
    if (this->error) {
        this->_print_error(label, gcode);
        return;
    }
    this->_i2c_write(address, location, OT_DATA_LENGTH);
    if (this->error) {
        this->_print_error(label, gcode);
        return;
    }
}

void Instrument::_check_ot_signature(uint8_t address, char label, Gcode *gcode) {
    this->_i2c_read(
        address, OT_SIGNATURE_LOCATION, this->signature_data, OT_SIGNATURE_LENGTH);
    if (this->error) {
        return;
    }
    else {
        for (uint8_t n=0;n<OT_SIGNATURE_LENGTH;n++) {
            if (this->signature_data[n] != OT_INSTRUMENT_DATA_SIGNATURE[n]) {
                this->error = NO_DATA_ERROR;
                return;
            }
        }
    }
}

void Instrument::_write_ot_signature(uint8_t address, char label, Gcode *gcode) {
    // erase buffer
    int i;
    this->_erase_i2c_buffer();
    for (i=0; i<OT_SIGNATURE_LENGTH; i++) {
        this->write_data[i + 1] = OT_INSTRUMENT_DATA_SIGNATURE[i];
    }

    // set memory address
    this->_i2c_write(address, OT_SIGNATURE_LOCATION, OT_SIGNATURE_LENGTH);
    if (this->error) {
        return;
    }
    this->_check_ot_signature(address, label, gcode);
    if (this->error) {
        return;
    }
}

void Instrument::_i2c_write(uint8_t address, char mem, int length) {
    const uint8_t sub_length = 8;
    char sub_data[sub_length + 1];
    for (uint8_t l=0;l<length;l+=sub_length) {
        this->error = NO_ERROR;
        sub_data[0] = mem + l;
        for (int i=0; i<sub_length; i++) {
            sub_data[i + 1] = this->write_data[l + i + 1];
        }
        this->error = this->i2c->write(address, sub_data, sub_length + 1);
        if (this->error != NO_ERROR) break;
        this->_i2c_delay();
    }
}

void Instrument::_i2c_read(uint8_t address, char mem, char *data, int length) {
    const uint8_t sub_length = 8;
    char sub_data[sub_length];
    for (int i=0; i<length; i++) data[i] = 0x00;
    for (uint8_t l=0;l<length;l+=sub_length) {
        this->error = NO_ERROR;
        // set address to write to
        this->memory_addr[0] = mem + l;
        this->error = this->i2c->write(address, this->memory_addr, 1);
        if (this->error != NO_ERROR) break;
        this->_i2c_delay();

        // read 8-bytes from that address
        for (int i=0; i<sub_length; i++) sub_data[i] = 0x00; // erase buffer
        this->error = this->i2c->read(address, sub_data, sub_length);
        if (this->error != NO_ERROR) break;
        this->_i2c_delay();

        // copy the 8-bytes just ready to the 
        for (int i=0; i<sub_length; i++) {
            data[i + l] = sub_data[i];
        }
    }
}

void Instrument::_parse_hex_from_gcode(char label, Gcode *gcode, uint8_t data_len) {
    unsigned int i;
    char len = strlen(gcode->command);
    for (i=0;i<len;i++){
        if (gcode->command[i] == label) {
            break;  // start following loop where this one broke
        }
    }
    this->_erase_i2c_buffer();
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
                if (increment == data_len + 1) {
                    break;
                }
            }
        }
    }
    // gcode->stream->printf("Parsed: ");
    // for (uint8_t i=0;i<data_len;i++) {
    //     gcode->stream->printf("%02X-", this->write_data[i + 1]);
    // }
    // gcode->stream->printf("\r\n");
}

void Instrument::_i2c_delay() {
    wait(0.01);
}

char Instrument::_decode_ascii(char c) {
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

void Instrument::_erase_i2c_buffer() {
    for (int i=0; i<OT_DATA_LENGTH + 1; i++) {
        this->write_data[i] = 0x00;
    }
}

char Instrument::_is_hex_ascii(char c) {
    if (c >= 'a' && c <= 'f') return true;
    else if (c >= 'A' && c <= 'F') return true;
    else if (c >= '0' && c <= '9') return true;
    return false;
}

void Instrument::_print_data(char label, Gcode *gcode){
    gcode->stream->printf("%c:", label);
    for (uint8_t i=0;i<OT_DATA_LENGTH;i++) {
        gcode->stream->printf("%02X", this->read_data[i]);
    }
    gcode->stream->printf("\r\n");
}

void Instrument::_print_error(char label, Gcode *gcode) {
    if (this->error == NO_ERROR){
        return;
    }
    else if (this->error == I2C_ERROR) {
        gcode->stream->printf("error:no %c instrument found\r\n", label);
    }
    else if (this->error == NO_DATA_ERROR){
        gcode->stream->printf("error:no data found on %c instrument\r\n", label);
    }
    else if (this->error == WRITE_ARGUMENTS_ERROR) {
        gcode->stream->printf("error:wrong number of bytes\r\n");
    }
}
