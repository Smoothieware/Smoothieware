#ifndef MODULE_PREVENT_COLD_EXTRUSION_H_
#define MODULE_PREVENT_COLD_EXTRUSION_H_

#include "Module.h"
#include <stdint.h>

#define FLAG_ENABLE      0x01
#define FLAG_READ_TEMP   0x02
#define FLAG_TEMP_OK     0x04
#define FLAG_SECOND_TICK 0x08
#define FLAG_KILL_NOW    0x10
#define FLAG_WAS_KILLED  0x20

class ColdExtrusionPrevention : public Module {
    public:
        ColdExtrusionPrevention(uint16_t identifier);

        void on_module_loaded();
        void on_gcode_execute(void* argument);
        void on_config_reload(void* argument);
        void on_idle(void* argument);
        void on_second_tick(void* argument);
        void on_gcode_received(void* argument);
        void on_halt(void* argument);

        uint8_t get_highest_temperature();

        void set_flag(uint8_t flag, bool enabled);
        bool read_flag(uint8_t flag);

    private:
        uint8_t  flags;
        uint8_t  minExtrusionTemperature;

        uint16_t temperatureController;

        uint16_t identifier;
};

#endif

