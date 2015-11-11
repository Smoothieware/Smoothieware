#pragma once

#include "Module.h"
#include "Pin.h"

#include <stdint.h>

namespace mbed {
    class SPI;
}

class MotorDriverControl : public Module {
    public:
        MotorDriverControl(uint16_t cs, uint8_t id);
        virtual ~MotorDriverControl();

        void on_module_loaded();
        void on_gcode_received(void *);

    private:
        bool config_module();
        void set_current( float current );
        void set_microstep( uint8_t ms );
        void set_decay_mode( uint8_t dm );
        void set_torque(float torque, float gain);

        Pin spi_cs_pin;
        mbed::SPI *spi;

        enum CHIP_TYPE {
            DRV8711,
            TMC2660
        };
        CHIP_TYPE chip;

        float current_factor;
        float max_current;
        float current;
        float torque{-1}, gain{-1};

        char designator;
        uint16_t cs;
        uint8_t id;
        uint8_t microsteps;
        uint8_t decay_mode;

};
