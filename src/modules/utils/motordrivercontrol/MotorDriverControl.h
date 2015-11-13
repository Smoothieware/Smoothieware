#pragma once

#include "Module.h"
#include "Pin.h"

#include <stdint.h>

namespace mbed {
    class SPI;
}

class DRV8711DRV;
class TMC26X;
class StreamOutput;

class MotorDriverControl : public Module {
    public:
        MotorDriverControl(uint8_t id);
        virtual ~MotorDriverControl();

        void on_module_loaded();
        void on_gcode_received(void *);
        void on_halt(void *argument);

    private:
        bool config_module(uint16_t cs);
        void initialize_chip();
        void set_current( uint32_t current );
        uint32_t set_microstep( uint32_t ms );
        void set_decay_mode( uint8_t dm );
        void dump_status(StreamOutput*);
        void enable(bool on);
        int sendSPI(uint8_t *b, int cnt, uint8_t *r);

        Pin spi_cs_pin;
        mbed::SPI *spi;

        enum CHIP_TYPE {
            DRV8711,
            TMC2660
        };
        CHIP_TYPE chip;

        // one of these drivers
        union {
            DRV8711DRV *drv8711;
            TMC26X *tmc26x;
        };

        float current_factor;
        uint32_t max_current; // in milliamps
        uint32_t current; // in milliamps
        float torque{-1}, gain{-1};
        uint32_t microsteps;

        char designator;

        struct{
            uint8_t id:4;
            uint8_t decay_mode:4;
            bool rawreg:1;
        };

};