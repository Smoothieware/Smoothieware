#ifndef INSTRUMENT_H
#define INSTRUMENT_H

#include "Module.h"
#include "Instrument.h"
#include "libs/Kernel.h"
#include "Gcode.h"
#include "StreamOutputPool.h"
#include "libs/utils.h"
#include <libs/Pin.h>
#include "mbed.h"
#include <string>
#include <math.h>

#define EEPROM_LEFT_ADDRESS 0xA0
#define EEPROM_RIGHT_ADDRESS 0xA2

#define GCODE_READ_ID 369      // return IDs of all attached instruments
#define GCODE_WRITE_ID 370     // write ID to 1 attached instrument
#define GCODE_READ_MODEL 371   // return IDs of all attached instruments
#define GCODE_WRITE_MODEL 372  // write ID to 1 attached instrument
#define GCODE_READ_DATA 373    // read 16-byte data from 1 attached instrument
#define GCODE_WRITE_DATA 374   // write 16-byte data to 1 attached instrument

// known series of bytes, used to verify data is valid
#define OT_SIGNATURE_LOCATION 0x00
#define OT_SIGNATURE_LENGTH 8
const uint8_t OT_INSTRUMENT_DATA_SIGNATURE[OT_SIGNATURE_LENGTH] {
    0x6f, 0x70, 0x65, 0x6e, 0x74, 0x72, 0x6f, 0x6e
};

// 32-byte ID
#define OT_ID_LOCATION 0x30
#define OT_ID_LENGTH 32

// 32-bytes for any data to be saved on this instrument
#define OT_MODEL_LOCATION 0x60
#define OT_MODEL_LENGTH 32

// 32-bytes for any data to be saved on this instrument
#define OT_DATA_LOCATION 0xA0
#define OT_DATA_LENGTH 32

#define NO_ERROR 0
#define WRITE_ARGUMENTS_ERROR 1
#define NO_DATA_ERROR 2
#define I2C_ERROR 32

class Instrument : public Module{
    public:

        Instrument();
        virtual ~Instrument(){
            delete this->i2c;
        }
        void on_module_loaded();
        void on_gcode_received(void *argument);

    private:

        mbed::I2C* i2c;

        char unique_id[OT_ID_LENGTH];
        char read_data[OT_DATA_LENGTH];
        char write_data[OT_DATA_LENGTH + 1];
        char signature_data[OT_SIGNATURE_LENGTH];

        int error;
        char memory_addr[1];

        void _read_ot_id(uint8_t address, char label, Gcode *gcode);
        void _write_ot_id(uint8_t address, char label, Gcode *gcode);
        void _read_ot_model(uint8_t address, char label, Gcode *gcode);
        void _write_ot_model(uint8_t address, char label, Gcode *gcode);
        void _read_ot_data(uint8_t address, char label, Gcode *gcode);
        void _write_ot_data(uint8_t address, char label, Gcode *gcode);
        void _check_ot_signature(uint8_t address, char label, Gcode *gcode);
        void _write_ot_signature(uint8_t address, char label, Gcode *gcode);
        void _write_hex_from_gcode(uint8_t address, char label, Gcode *gcode, char mem, uint8_t len);
        void _i2c_write(uint8_t address, char mem, int length);
        void _i2c_read(uint8_t address, char mem, char *data, int length);
        void _parse_hex_from_gcode(char label, Gcode *gcode, uint8_t data_len);
        void _i2c_delay();
        char _decode_ascii(char c);
        void _erase_i2c_buffer();
        char _is_hex_ascii(char c);
        void _print_data(char label, Gcode *gcode);
        void _print_id(char label, Gcode *gcode);
        void _print_error(char label, Gcode *gcode);
};

#endif
