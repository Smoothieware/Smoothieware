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
        MotorDriverControl(uint16_t cs, uint8_t id);
        virtual ~MotorDriverControl();

        void on_module_loaded();
        void on_gcode_received(void *);
        void on_gcode_execute(void *);

    private:
        bool config_module();
        void initialize_chip();
        void set_current( uint32_t current );
        void set_microstep( uint32_t ms );
        void set_decay_mode( uint8_t dm );
        void set_torque(float torque, float gain);
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

        char designator;
        uint16_t cs;
        uint8_t id;
        uint8_t microsteps;
        uint8_t decay_mode;

};
