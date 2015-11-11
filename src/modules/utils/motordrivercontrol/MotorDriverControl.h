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
        float get_current(int channel) const { return current; };

    protected:
        float current_factor;
        float max_current;
        float current;
        mbed::SPI *spi;
        Pin spi_cs_pin;
        char designator;
        uint16_t cs;
        uint8_t id;
};
